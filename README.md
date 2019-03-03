Asservissement_Pololu
=====================

 Aim
 ---

 This is a simple Motor control program for Pololu 70:1 Metal Gearmotor
 37Dx70Lmm with 64 CPR encoder. It uses an Adafruit P377 Rotary encoder to
 set the speed and enable/disable the output.

 The motor power is set using a Graupner 40R Speed Profi regulator. the
 reason behind this coice is quite simple : easy PWM command + 5V power
 to the system coming from the regulator. Another important point is that it
 speeds up a lot the prototyping phase and we're short on time.

 The main idea is to use the motor encoder to measure the speed. Every pulse
 from motor encoder will be counted on an interruption to ensure relevant
 value update. Button click will be counted the same way. The control loop
 will update the speed command value once every 1/50 second with a timer
 interruption.

 Wiring
 ------

 ```
            STM32L432KC
            Nucleo board
              +-----+
          +---| USB |---+
          |   +-----+   |
         [ ] D1    Vin [ ]
         [ ] D0    GND [ ]
         [ ] NRST NRST [ ]
         [ ] GND    5V [ ]
         [ ] D2     A7 [ ]
         [ ] D3     A6 [ ]
         [ ] D4     A5 [ ]
         [ ] D5     A4 [ ]
         [ ] D6     A3 [ ]
         [ ] D7     A2 [ ]
         [ ] D8     A1 [ ]
         [ ] D9     A0 [ ]
         [ ] D10  AREF [ ]
         [ ] D11   3V3 [ ]
         [ ] D12   D13 [ ]
          |             |
          +-------------+
 ```

 Notes
 -----

 At the moment we only use one of the two motor encoder ways.

 Using the 2 ways might allow us to improve the precision along with
 enabling us to check if the wheel is turning in the direction the
 command is set.
