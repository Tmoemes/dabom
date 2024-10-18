#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>

class Motor
{
public:
    Motor(int enA, int in1, int in2, Encoder &encoder, bool inverted = false);
    void begin();
    void setSpeed(int speed);
    void forward();
    void backward();
    void stop();
    long readEncoder();
    void resetEncoder();
    double getSpeed();

private:
    // motor driver pins
    int _enA;
    int _in1;
    int _in2;

    // encoder pins
    int _encoderA;
    int _encoderB;

    // encoder variables
    volatile long _encoderCount;

    // velocity variables
    long _lastTime;
    long _lastPosition;

    Encoder &_encoder;

    /// @brief inverts the encoder reading sign
    bool _inverted;

    static void encoderISR();
};

#endif // MOTOR_H
