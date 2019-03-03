/*
 * ============================================================================
 * ==== Pololu 70:1 Metal Gearmotor 37Dx70Lmm with 64 CPR encoder control =====
 * ============================================================================
 */
/**
  * @file
  * @copyright MIT License
  * @author Vincent RICCHI
  * @date March 2019
  *
  * @mainpage Pololu 70:1 Metal Gearmotor 37Dx70Lmm with 64 CPR encoder control
  *
  * Aim
  * ---
  *
  * This is a simple Motor control program for Pololu 70:1 Metal Gearmotor
  * 37Dx70Lmm with 64 CPR encoder. It uses an Adafruit P377 Rotary encoder to
  * set the speed and enable/disable the output.
  *
  * The motor power is set using a Graupner 40R Speed Profi regulator. the
  * reason behind this coice is quite simple : easy PWM command + 5V power
  * to the system coming from the regulator. Another important point is that it
  * speeds up a lot the prototyping phase and we're short on time.
  *
  * The main idea is to use the motor encoder to measure the speed. Every pulse
  * from motor encoder will be counted on an interruption to ensure relevant
  * value update. Button click will be counted the same way. The control loop
  * will update the speed command value once every 1/50 second with a timer
  * interruption.
  *
  * Wiring
  * ------
  *
  * @verbatim
  *            STM32L432KC
  *            Nucleo board
  *              +-----+
  *          +---| USB |---+
  *          |   +-----+   |
  *         [ ] D1    Vin [ ]
  *         [ ] D0    GND [ ]
  *         [ ] NRST NRST [ ]
  *         [ ] GND    5V [ ]
  *         [ ] D2     A7 [ ]
  *         [ ] D3     A6 [ ]
  *         [ ] D4     A5 [ ]
  *         [ ] D5     A4 [ ]
  *         [ ] D6     A3 [ ]
  *         [ ] D7     A2 [ ]
  *         [ ] D8     A1 [ ]
  *         [ ] D9     A0 [ ]
  *         [ ] D10  AREF [ ]
  *         [ ] D11   3V3 [ ]
  *         [ ] D12   D13 [ ]
  *          |             |
  *          +-------------+
  * @endverbatim
  *
  * Notes
  * -----
  *
  * At the moment we only use one of the two motor encoder ways.
  *
  * Using the 2 ways might allow us to improve the precision along with
  * enabling us to check if the wheel is turning in the direction the
  * command is set.
  *
  */

/*
 * ============================================================================
 * ================================= INCLUDES =================================
 * ============================================================================
 */

#include "mbed.h"
#include <string.h>

/*
 * ============================================================================
 * ============================ MACRO & CONSTANTS =============================
 * ============================================================================
 */

const unsigned int COMPUTER_BUFFER_LENGTH = 256;
const unsigned int DUTY_CYCLE_PERIOD_ms   = 1;  //!< Graupner 40R regulator runs at 1kHz (i.e. 1/1ms)
const float TIMER_PERIOD_sec              = 0.02f;  //!< Set timer period to 20ms

// User rotary encoder boundaries
const int USER_INPUT_VALUE_MAX            =  100;
const int USER_INPUT_VALUE_MIN            =  -100;

// Duty cycle boundaries
const float DUTY_CYCLE_MAX                = 1.00f;
const float DUTY_CYCLE_MIN                = 0.00f;
const float DUTY_CYCLE_MIDDLE             = 0.50f;

/*
 * ============================================================================
 * =============================== STRUCTURES =================================
 * ============================================================================
 */

/**
  * @brief Makes it more easy to use send & receive buffer with any
  *        communication bus that uses 2 buffers (UART, SPI, I2C, ...)
  *        for these purposes.
  */
typedef struct _inout_buff
{
    char Tx[ COMPUTER_BUFFER_LENGTH ];
    char Rx[ COMPUTER_BUFFER_LENGTH ];
} inout_buff_t;

/*
 * ============================================================================
 * ================================= GLOBALS ==================================
 * ============================================================================
 */

inout_buff_t computer; //!< input/output buffers for PC
PwmOut motor( D6 ); //!< PWM output to DC/DC controller for motor control


/**
  * Since we may want to turn the wheel in the opposite direction we better
  * use an int value than an unsigned int for user interface encoder.
  *
  * An int on 8 bits is wide enough for values within [ -100 ; 100 ] boundaries
  * that's 200 steps to go from 0 to 100% of the duty cycle.
  */
volatile int8_t user_encoder_value = 0;

/**
  * Flag used to tell if A interrupt has happend before or after B.
  *
  * @verbatim
  *    signals
  *       ^
  *       |     _:__      ____      ____
  *  in A |____| :  |____|    |____|    |__
  *       |      :
  *       |      :____      ____      ____
  *  in B |______|    |____|    |____|    |__
  *       |      :
  *       +------:----------------------------> time
  *            ----> Turning Clockwise
  *
  *    signals
  *       ^
  *       |      :____      ____      ____
  *  in A |______|    |____|    |____|    |__
  *       |      :
  *       |     _:__      ____      ____
  *  in B |____| :  |____|    |____|    |__
  *       |      :
  *       +------:----------------------------> time
  *            ----> Turning Counter Clockwise
  * @endverbatim
  */
volatile bool inA_flag = false;
volatile bool inB_flag = false;

/**
  * Inputs & interruptions for user encoder.
  *
  * We use 2 to know if we're incrementing or decrementing encoder value
  * counter.
  */
DigitalIn UserEncoder_inA( D11 );
DigitalIn UserEncoder_inB( D12 );
InterruptIn inA_triger( D11 );

// User Button related globals
volatile bool motor_output_enable = false; //!< Ouput enable flag (value set by rotary encoder)
DigitalIn UserEncoder_button( D9 ); //!< Input for PWM output enable
InterruptIn button_triger( D9 ); //!< Button interruption object

/**
  * Duty cycle for moto is updated once every 1/50 seconds. This value is a
  * float which value is to be within [ 0.00 ; 1.00 ] bounadaries. 1.00 being
  * equivalent to 100 % of the duty cycle
  */
volatile float motor_duty_cycle = DUTY_CYCLE_MIDDLE;

Ticker timer; //!< Timer objet used to update motor duty cycle

DigitalIn motor_ticks_A( D4 );
InterruptIn motor_ticks( D4 );
DigitalIn motor_ticks_B( D5 );
volatile int motor_pulse_count = 0; //!< variable used to monitor motor turns

/*
 * ============================================================================
 * ================================ FUNCTIONS =================================
 * ============================================================================
 */

/**
  * @brief Map a float value within defined range.
  * @param [in] in      Input value to map on range
  * @param [in] inMin   Minimum accepted input value
  * @param [in] inMax   Maximum accepted input value
  * @param [in] outMin  Low range boundary. Output will not be inferior to this.
  * @param [in] outMax  High range boundary. Output will not be greater than this.
  * @return A float value within the mapping range.
  */
float map_value( float in, float inMin, float inMax, float outMin, float outMax )
{
    // check it's within the range
    if( inMin < inMax )
    {
        if( in <= inMin )
        {
            return outMin;
        }

        if( in >= inMax )
        {
            return outMax;
        }
    }
    else
    {
        // cope with input range being backwards.
        if( in >= inMin )
        {
            return outMin;
        }

        if( in <= inMax )
        {
            return outMax;
        }
    }

    // calculate how far into the range we are
    float scale = (in-inMin)/(inMax-inMin);

    // calculate the output.
    return outMin + scale*(outMax-outMin);
}

/**
  * @brief Timer interrupt handler to update motor's PWM duty cycle
  */
void motor_update( void )
{
    /**
      * Remap `user_encoder_value` in [ 0.00 ; 1.00 ] range
      * to create new value for `motor_duty_cycle`.
      */
    motor_duty_cycle = map_value( (float)user_encoder_value,
                                  USER_INPUT_VALUE_MIN,
                                  USER_INPUT_VALUE_MAX,
                                  DUTY_CYCLE_MIN,
                                  DUTY_CYCLE_MAX );

    //! update motor duty cycle output if output enabled
    if( true == motor_output_enable )
    {
        motor.write( motor_duty_cycle );
    }
    else
    {
        // Motor output disabled => back to middle ground
        motor.write( DUTY_CYCLE_MIDDLE );
    }
}

/**
  * @brief Update `motor_output_enable` value. PWM output status will be
  *        updated upon  next `motor_update` call on timer interruption.
  */
void enable_button_behavior( void )
{
    if( true == motor_output_enable )
    {
        motor_output_enable = false;
    }
    else
    {
        motor_output_enable = true;
    }
}

/**
  * @brief Update `motor_output_enable` value. PWM output status will be
  *        updated upon  next `motor_update` call on timer interruption.
  */
void user_encoder_pulse_inA( void )
{
    if( 0 == UserEncoder_inB.read( ) )
    {
        // B is already down => B before A => decrement
        user_encoder_value--;
        if( USER_INPUT_VALUE_MIN > user_encoder_value )
        {
            user_encoder_value = USER_INPUT_VALUE_MIN;
        }
    }
    else
    {
        //! A down before B => increment `user_encoder_value`
        user_encoder_value++;
        if( USER_INPUT_VALUE_MAX < user_encoder_value )
        {
            user_encoder_value = USER_INPUT_VALUE_MAX;
        }
    }
}

/**
  * @brief Count pulses from motor
  */
void motor_pulses( void )
{
    if( 1 == motor_ticks_B.read( ) )
    {
        // B is already down => B before A => decrement
        motor_pulse_count--;
    }
    else
    {
        //! A down before B => increment `user_encoder_value`
        motor_pulse_count++;
    }
}

/*
 * ============================================================================
 * ============================== MAIN FUNCTION ===============================
 * ============================================================================
 */

/**
  * @brief Main function
  */
int main( void )
{
    // ---- Setup -----

    // PC communication init to null character
    memset( &computer, 0x00, sizeof( computer ) );

    // ~~~ Duty Cycle pin & update interrupt ~~~
    // Set period of PWM for Graupner 40R regulator at 1kHz (i.e. 1/1ms)
    motor.period_ms( DUTY_CYCLE_PERIOD_ms );
    // Set PWM to middle ground for Graupner 40R regulator
    motor.write( DUTY_CYCLE_MIDDLE ); // 50 % of duty cycle
    // Timer interrupt timing and interruption handler
    timer.attach( motor_update, TIMER_PERIOD_sec );

    // ~~~ Setup digital inputs for user rotary encoder ~~~
    UserEncoder_inA.mode( PullUp );
    UserEncoder_inB.mode( PullUp );
    inA_triger.fall( &user_encoder_pulse_inA );

    // ~~~ Setup User Button mode and attach interrupt handler ~~~
    UserEncoder_button.mode( PullUp );
    button_triger.fall( &enable_button_behavior );

    // ~~~ Setup inputs and interupts for motor control ~~~
    motor_ticks_A.mode( PullUp );
    motor_ticks_B.mode( PullUp );
    motor_ticks.rise( &motor_pulses );

    // Setup is finished => send splashscreen to terminal
    printf( "----------------------------\n"
            "--- Pololu Motor Control ---\n"
            "----------------------------\n" );

    // ---- Infinite loop ----
    while( 1 )
    {
        wait( 1 ); // wait for 1 second

        // Reset PC communication Tx buffer
        memset( &computer, 0x00, sizeof( computer ) );

        // copy sentence to output buffer
        snprintf( computer.Tx, sizeof( computer.Tx ),
                  "Encoder : %d\nDuty cycle: %+1.2f\nMotor Ticks : %d\n",
                  user_encoder_value, motor_duty_cycle, motor_pulse_count );

        // Conditionnal append to buffer
        if( true == motor_output_enable )
        {
            strcat( computer.Tx, "Output : ENABLED\n" );
        }
        else
        {
            strcat( computer.Tx, "Output : DISABLED\n" );
        }

        // Throw output sentence to PC
        printf( computer.Tx );
    }
}
