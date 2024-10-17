#ifndef CONFIG_H
#define CONFIG_H
#include "motor.h"
#include <Encoder.h>

// PI COMM constants
#define PI_COMM_FREQ 20 
#define PI_COMM_TIMEOUT 300
#define PI_BAUD_RATE 38400

// DEBUG constants
#define DEBUG_FREQ 1000 // time in ms between debug prints, 0 to disable
#define DEBUG_BAUD_RATE 9600

// Motor 0:Front left
#define MOTOR0_PIN_EN 2 
#define MOTOR0_PIN_IN1 35
#define MOTOR0_PIN_IN2 37
#define ENCODER0_PIN_A 18 // INT.3 on Pin 18
#define ENCODER0_PIN_B 23   

// Motor 1: Front right
#define MOTOR1_PIN_EN 3
#define MOTOR1_PIN_IN1 41
#define MOTOR1_PIN_IN2 39
#define ENCODER1_PIN_A 19 // INT.2 on pin 19
#define ENCODER1_PIN_B 25

// Motor 2: Back left
#define MOTOR2_PIN_EN 4
#define MOTOR2_PIN_IN1 31
#define MOTOR2_PIN_IN2 33
#define ENCODER2_PIN_A 20 // INT.1 on pin 20
#define ENCODER2_PIN_B 27
 
// Motor 3: Back right
#define MOTOR3_PIN_EN 5
#define MOTOR3_PIN_IN1 45
#define MOTOR3_PIN_IN2 43
#define ENCODER3_PIN_A 21 // INT.0 on Pin 21
#define ENCODER3_PIN_B 29

// Defining and setting up the motors and encoders
Encoder enc0(ENCODER0_PIN_A, ENCODER0_PIN_B);
Encoder enc1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder enc2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder enc3(ENCODER3_PIN_A, ENCODER3_PIN_B);

Motor m0(MOTOR0_PIN_EN, MOTOR0_PIN_IN1, MOTOR0_PIN_IN2, enc0, true);
Motor m1(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2, enc1);
Motor m2(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2, enc2, true);
Motor m3(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2, enc3);

Motor motors[] = {m0,m1,m2,m3};

#endif // CONFIG_H