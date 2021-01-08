# SatNOGS Rotator Firmware for Wemos D1 R1 Microcontroller Board, using Telnet for wireless serial connection

Repository includes all source files for the SatNOGS rotator controller Firmware, instead of the custom SatNOGS PCB.

## Easycomm implemantation

* AZ, Azimuth, number - 1 decimal place [deg]
* EL, Elevation, number - 1 decimal place [deg]
* SA, Stop azimuth moving
* SE, Stop elevation moving
* RESET, Move to home position
* PARK, Move to park position
* IP, Read an input, number
    * Temperature = 0
    * SW1 = 1
    * SW2 = 2
    * Encoder1 = 3
    * Encoder2 = 4
* VE, Request Version
* GS, Get status register, number
    * idle = 1
    * moving = 2
    * pointing = 4
    * error = 8
* GE, Get error register, number
    * no_error = 1
    * sensor_error = 2
    * homing_error = 4
    * motor_error = 8
* RB, custom command to reboot controller

## Controller Configurations
* Stepper Motor
    * Endstops
    * Encoders, optional

## Pins Configuration

```
#define M1IN1 D4 ///< AZ Motor STEP
#define M1IN2 D7  ///< AZ Motor DIR

#define M2IN1 D3 ///< EL Motor STEP
#define M2IN2 D6  ///< EL Motor DIR

#define MOTOR_EN D8 ///< Digital output, EN pins for both A4988 drivers

#define SW1 D5 ///< Digital input, to read the status of end-stop for AZ motor
#define SW2 D2 ///< Digital input, to read the status of end-stop for EL motor
```

```
#define SAMPLE_TIME        0.1   // Control loop in s
#define RATIO              54    // Gear ratio of rotator gear box for V3.1 Satnogs
#define MICROSTEP          16    // Set Microstep, A4988 1/16 ustep MS1 H, MS2 H, MS3 H. 
#define MIN_PULSE_WIDTH    1     // In microsecond for AccelStepper, the default for genuine A4988 is 1uS
#define MAX_SPEED          8000  // In steps/s, consider the microstep, For 1/8 ustep it is 3200, For 1/16 8000
#define MAX_ACCELERATION   6400  // In steps/s^2, consider the microstep, For 1/8 ustep it is 1600, For 1/16 6400
#define SPR                3200L // Step Per Revolution, consider the microstep, For 1/8 ustep it is 1600L, For 1/16 3200L
#define MIN_M1_ANGLE       0     // Minimum angle of azimuth
#define MAX_M1_ANGLE       360   // Maximum angle of azimuth
#define MIN_M2_ANGLE       0     // Minimum angle of elevation
#define MAX_M2_ANGLE       360   // Maximum angle of elevation
#define DEFAULT_HOME_STATE LOW  // Change to LOW according to Home sensor
#define HOME_DELAY         100 // Time for homing Deceleration in millisecond
#define SerialPort         Serial
```
