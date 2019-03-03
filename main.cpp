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
 
//#define __ALTERNATIVE_DEBOUNCE__
#define __THIRD_DEBOUNCE__

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

/**
  * @brief values used to define state of user rotary encoder
  */
typedef enum
{
    IN_DEFAULT,
    IN_A,
    IN_B,
} encoder_channel_t;

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

volatile encoder_channel_t first_flag = IN_DEFAULT;

/**
  * Inputs & interruptions for user encoder.
  *
  * We use 2 to know if we're incrementing or decrementing encoder value
  * counter.
  */
DigitalIn UserEncoder_inA( D11 );
DigitalIn UserEncoder_inB( D12 );
InterruptIn inA_triger( D11 );
#ifndef __THIRD_DEBOUNCE__
InterruptIn inB_triger( D12 );
#ifdef __ALTERNATIVE_DEBOUNCE__
Ticker encoder_update_timer;
#endif // __ALTERNATIVE_DEBOUNCE__
#endif // __THIRD_DEBOUNCE__

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
#ifndef __THIRD_DEBOUNCE__
#ifndef __ALTERNATIVE_DEBOUNCE__
    /**
      * Debounce A :
      *   - A is set if B is false and clears B everytime.
      *   - B clears A everytime
      *   => If A is set, then we re-enter A event handler witout going into B
      *      which means we do not advance the encoder position
      */
    if( true == inA_flag )
    {
        return;
    }
    
    //! Test whether B has been triggered before A
    if( false == inB_flag )
    {
        //! A event has happend before B => increment `user_encoder_value`
        user_encoder_value++;
        if( USER_INPUT_VALUE_MAX < user_encoder_value )
        {
            user_encoder_value = USER_INPUT_VALUE_MAX; 
        }
        
        // Set A flag for coming B event
        inB_flag = true;
    }
    else
    {
        //! B event has happend before A => decrement `user_encoder_value`
        user_encoder_value--;
        if( USER_INPUT_VALUE_MIN > user_encoder_value )
        {
            user_encoder_value = USER_INPUT_VALUE_MIN;
        }
    }
    
    //! Clear B flags for next round
    inB_flag = false;
#else // means __ALTERNATIVE_DEBOUNCE__ is defined
    if( false == inB_flag )
    {
        first_flag = IN_A; 
    }
    
    inA_flag = true;
#endif // __ALTERNATIVE_DEBOUNCE__
#else // i.e. __THIRD_DEBOUNCE__ is defined
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
#endif // __THIRD_DEBOUNCE__
}

/**
  * @brief Update `motor_output_enable` value. PWM output status will be 
  *        updated upon  next `motor_update` call on timer interruption.
  */
void user_encoder_pulse_inB( void )
{
#ifndef __ALTERNATIVE_DEBOUNCE__
    /**
      * Debounce B :
      *   - B is set if A is false and clears A everytime.
      *   - A clears B everytime
      *   => If B is set, then we re-enter B event handler witout going into A
      *      which means we do not advance
      */
    if( true == inB_flag )
    {
        return;
    }
    
    // Set B flag if A event has not happened yet
    if( false == inA_flag )
    {
        inB_flag = true;
    }
    
    // Clear A flag for next round
    inA_flag = false;
#else // means __ALTERNATIVE_DEBOUNCE__ is defined
    if( false == inA_flag )
    {
        first_flag = IN_B; 
    }
    
    inB_flag = true;
#endif // __ALTERNATIVE_DEBOUNCE__
}

void user_encoder_value_update( void )
{
#ifdef __ALTERNATIVE_DEBOUNCE__
    // Check that the 2 flags have been set (for debounce purpose)
    if( ( true == inA_flag ) &&
        ( true == inB_flag )   )
    {
        /**
          * Increment or decrement `user_encoder_value` depending on if the
          * first event to happen is A or B.
          * 
          * Do not use an `else` statement to avoid potential case where
          * `first_flag` is `IN_DEFAULT`
          */
        if( IN_A == first_flag )
        {
            user_encoder_value++;
        }
        
        if( IN_B == first_flag )
        {
            user_encoder_value--;
        }
    }
    
    // Clear all flags for next round
    first_flag = IN_DEFAULT;
    inA_flag = false;
    inB_flag = false;
#endif // __ALTERNATIVE_DEBOUNCE__
}

/** 
  * @brief Count pulses from motor
  */
void motor_pulses( void )
{
#ifdef __THIRD_DEBOUNCE__
        if( 1 == motor_ticks_A.read( ) )
    {
        // B is already down => B before A => decrement
        motor_pulse_count--;
    }
    else
    {
        //! A down before B => increment `user_encoder_value`
        motor_pulse_count++;
    }
#else // __THIRD_DEBOUNCE__ is not defined
     motor_pulse_count++;
#endif
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
    
    // Set period of PWM for Graupner 40R regulator at 1kHz (i.e. 1/1ms)
    motor.period_ms( DUTY_CYCLE_PERIOD_ms );
    // Set PWM to middle ground for Graupner 40R regulator
    motor.write( DUTY_CYCLE_MIDDLE ); // 50 % of duty cycle
    
    timer.attach( motor_update, TIMER_PERIOD_sec );
    
    // Setup digital inputs
    UserEncoder_inA.mode( PullUp );
    UserEncoder_inB.mode( PullUp );
    UserEncoder_button.mode( PullUp );
    
    inA_triger.fall( &user_encoder_pulse_inA );
#ifndef __THIRD_DEBOUNCE__
    inA_triger.rise( &user_encoder_value_update );
#endif // __THIRD_DEBOUNCE__
#ifdef __ALTERNATIVE_DEBOUNCE__
    inB_triger.fall( &user_encoder_pulse_inB );
#endif // __ALTERNATIVE_DEBOUNCE__

#ifdef  __ALTERNATIVE_DEBOUNCE__
    //setup `encoder_update_timer` for buton state update
    encoder_update_timer.attach( &user_encoder_value_update, ( TIMER_PERIOD_sec / 2 ) );
#endif // __ALTERNATIVE_DEBOUNCE__

    button_triger.fall( &enable_button_behavior );
    
    // setup inputs and interupts for motor control
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
