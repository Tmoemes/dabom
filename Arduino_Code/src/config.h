#ifndef CONFIG_H
#define CONFIG_H
#include "motor.h"
#include <Encoder.h>
// int pulsesPerRevolution = 1440;

#if defined(__AVR_ATmega2560__)
//front left
// Motor 0 Pins
#define MOTOR0_PIN_EN 2 
#define MOTOR0_PIN_IN1 35
#define MOTOR0_PIN_IN2 37
// Encoder 0 Pins
#define ENCODER0_PIN_A 18   //blue 
#define ENCODER0_PIN_B 23   //yellow

//front right
// Motor 1 Pins
#define MOTOR1_PIN_EN 3
#define MOTOR1_PIN_IN1 41
#define MOTOR1_PIN_IN2 39
// Encoder 1 Pins
#define ENCODER1_PIN_A 19
#define ENCODER1_PIN_B 25

//back left
// Motor 2 Pins
#define MOTOR2_PIN_EN 4
#define MOTOR2_PIN_IN1 31
#define MOTOR2_PIN_IN2 33
// Encoder 2 Pins
#define ENCODER2_PIN_A 20
#define ENCODER2_PIN_B 27

//back right
// Motor 3 Pins
#define MOTOR3_PIN_EN 5
#define MOTOR3_PIN_IN1 45
#define MOTOR3_PIN_IN2 43
// Encoder 3 Pins
#define ENCODER3_PIN_A 21
#define ENCODER3_PIN_B 29
#endif

Encoder enc0(ENCODER0_PIN_A, ENCODER0_PIN_B);
Encoder enc1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder enc2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder enc3(ENCODER3_PIN_A, ENCODER3_PIN_B);

Motor m0(MOTOR0_PIN_EN, MOTOR0_PIN_IN1, MOTOR0_PIN_IN2, enc0, true);
Motor m1(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2, enc1);
Motor m2(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2, enc2, true);
Motor m3(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2, enc3);

Motor motors[] = {m0,m1,m2,m3};

// INT.0 on Pin 21
// INT.1 on pin 20
// INT.2 on pin 19
// INT.3 on Pin 18
// INT.4 on Pin 2
// INT.5 on Pin 3
#endif // CONFIG_H