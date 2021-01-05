/*!
* @file rotator_pins.h
*
* It is a header file for pin mapping.
*
* Licensed under the GPLv3
*
*/

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

#define M1IN1 D4 ///< Motor 1 PWM pin
#define M1IN2 D7  ///< Motor 1 PWM pin

#define M2IN1 D3 ///< Motor 2 PWM pin
#define M2IN2 D6  ///< Motor 2 PWM pin

#define MOTOR_EN D8 ///< Digital output, to enable the motors

#define SW1 D13 ///< Digital input, to read the status of end-stop for motor 1
#define SW2 D2 ///< Digital input, to read the status of end-stop for motor 2

#endif /* ROTATOR_PINS_H_ */
