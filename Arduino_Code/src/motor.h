#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>

class Motor {
public:
    Motor(int enA, int in1, int in2,Encoder &encoder, bool inverted = false);
    void begin();
    void tick();
    void setSpeed(int speed);
    void forward();
    void backward();
    void stop();
    long readEncoder();
    void resetEncoder();
    double getSpeed();

private:
    //motor driver pins
    int _enA;
    int _in1;
    int _in2;
    //encoder pins
    int _encoderA;
    int _encoderB;

    //encoder variables
    volatile long _encoderCount;

    //pid variables
    long _lastTime;
    long _lastPosition;

    //pid variables
    double Kp = 3.0, Ki = 5.0, Kd = 1.0; 
    double _targetSpeed;
    double _currentOutput;
    double prevError = 0;
    double integral = 0;

    //wheel specifications
    const double wheelDiameter = 0.080;  // Wheel diameter in meters (80mm)
    const int pulsesPerRevolution = 1440; // Number of encoder ticks per revolution
    Encoder& _encoder;
    bool _inverted;
    void PID();
    double calculateAngularVelocity(unsigned long currentTime);
    static void encoderISR();
};

#endif // MOTOR_H
