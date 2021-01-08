#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

#define M1IN1 D4 ///< AZ Motor STEP
#define M1IN2 D7  ///< AZ Motor DIR

#define M2IN1 D3 ///< EL Motor STEP
#define M2IN2 D6  ///< EL Motor DIR

#define MOTOR_EN D8 ///< Digital output, EN pins for both A4988 drivers

#define SW1 D13 ///< Digital input, to read the status of end-stop for AZ motor
#define SW2 D2 ///< Digital input, to read the status of end-stop for EL motor

#endif /* ROTATOR_PINS_H_ */
