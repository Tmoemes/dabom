#include "motor.h"

Motor::Motor(int enA, int in1, int in2, Encoder &encoder, bool inverted)
    : _enA(enA), _in1(in1), _in2(in2), _encoder(encoder), _inverted(inverted), _encoderCount(0) {}

void Motor::begin()
{
    pinMode(_enA, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
}

void Motor::setSpeed(int speed)
{
    analogWrite(_enA, abs(speed));
}

void Motor::forward()
{
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
}

void Motor::backward()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
}

void Motor::stop()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}


/// @brief 
/// @return (long) encoder count value
long Motor::readEncoder()
{
    return _inverted ? _encoder.read() * -1 : _encoder.read();
}

/// @brief sets encoder count to 0
void Motor::resetEncoder()
{
    _encoder.write(0);
}

/// @brief get the speed of the motor according to encoder readings
/// @return angular velocity (rad/s)
double Motor::getSpeed()
{
    long currentTime = millis();
    long elapsedTime = currentTime - _lastTime;
    long currentPosition = _encoderCount;
    long deltaPosition = currentPosition - _lastPosition;
    double speed = (double)deltaPosition / elapsedTime;
    _lastTime = currentTime;
    _lastPosition = currentPosition;
    return speed;
}
