#ifndef CONFIG_H
#define CONFIG_H

#include "motor.h"
#include <Encoder.h>

// Configurable mapping values
const float minInputSpeed = 0.0;   // Minimum expected input speed
const float maxInputSpeed = 16.0;  // Maximum expected input speed
const int minPWM = 50;             // Minimum PWM value to prevent stalling
const int maxPWM = 255;            // Maximum PWM value (full speed)

#if defined(__AVR_ATmega2560__)
// Front left motor (Motor 0)
#define MOTOR0_PIN_EN 2 
#define MOTOR0_PIN_IN1 35
#define MOTOR0_PIN_IN2 37
#define ENCODER0_PIN_A 18   // Blue wire
#define ENCODER0_PIN_B 23   // Yellow wire

// Front right motor (Motor 1)
#define MOTOR1_PIN_EN 3
#define MOTOR1_PIN_IN1 41
#define MOTOR1_PIN_IN2 39
#define ENCODER1_PIN_A 19
#define ENCODER1_PIN_B 25

// Back left motor (Motor 2)
#define MOTOR2_PIN_EN 4
#define MOTOR2_PIN_IN1 31
#define MOTOR2_PIN_IN2 33
#define ENCODER2_PIN_A 20
#define ENCODER2_PIN_B 27

// Back right motor (Motor 3)
#define MOTOR3_PIN_EN 5
#define MOTOR3_PIN_IN1 45
#define MOTOR3_PIN_IN2 43
#define ENCODER3_PIN_A 21
#define ENCODER3_PIN_B 29
#endif

// Create encoder objects
Encoder enc0(ENCODER0_PIN_A, ENCODER0_PIN_B);
Encoder enc1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder enc2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder enc3(ENCODER3_PIN_A, ENCODER3_PIN_B);

// Create motor objects
Motor m0(MOTOR0_PIN_EN, MOTOR0_PIN_IN1, MOTOR0_PIN_IN2, enc0, true);
Motor m1(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2, enc1);
Motor m2(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2, enc2, true);
Motor m3(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2, enc3);

// Array of motors for easy management
Motor motors[] = {m0, m1, m2, m3};

// Interrupt pin mapping (for reference)
// INT0 on Pin 21
// INT1 on Pin 20
// INT2 on Pin 19
// INT3 on Pin 18
// INT4 on Pin 2
// INT5 on Pin 3

#endif // CONFIG_H
