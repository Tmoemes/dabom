#include <Arduino.h>
#include "config.h"

void processBinarySerialInput();
void sendEncoderData();
void controlMotors(float motorVelocities[4]);
int mapFloat(float x, float in_min, float in_max, int out_min, int out_max);

long currentTime = 0;
long lastUpdateTime = 0;
long lastReceivedTime = 0;

const long updateInterval = 8; // 8 ms for 120 Hz

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial Monitor to open
  }

  // Initialize motors
  for (size_t i = 0; i < 4; i++) {
    motors[i].begin();
    Serial.print("Motor ");
    Serial.print(i);
    Serial.println(" initialized");
  }

  Serial.println("Setup Complete");
  lastReceivedTime = millis();
  lastUpdateTime = millis();
}

void loop()
{
  currentTime = millis();

  // Read serial data as soon as it's available
  processBinarySerialInput();

  // Update at 120 Hz
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    sendEncoderData();

    // Stop motors if no command is received within 300 ms
    if (currentTime - lastReceivedTime >= 300) {
      for (int i = 0; i < 4; i++) {
        motors[i].stop();
      }
    }
  }
}

void processBinarySerialInput() {
  const byte START_MARKER = 0x02; // STX (Start of Text)
  const byte END_MARKER = 0x03;   // ETX (End of Text)

  static boolean recvInProgress = false;
  static byte receivedBytes[17]; // 16 data bytes + 1 checksum
  static byte index = 0;

  while (Serial.available() > 0) {
    byte rb = Serial.read();

    if (recvInProgress) {
      if (rb == END_MARKER) {
        recvInProgress = false;
        if (index == 17) {
          // Verify checksum
          byte calculatedChecksum = 0;
          for (int i = 0; i < 16; i++) {
            calculatedChecksum ^= receivedBytes[i];
          }
          if (calculatedChecksum == receivedBytes[16]) {
            float motorVelocities[4];
            memcpy(motorVelocities, receivedBytes, 16);
            controlMotors(motorVelocities);
            lastReceivedTime = currentTime; // Update last received time
          } else {
            Serial.println("Error: Checksum mismatch");
          }
        } else {
          Serial.println("Error: Incorrect message length");
        }
        index = 0; // Reset index for next message
      } else {
        // Collect data bytes
        if (index < sizeof(receivedBytes)) {
          receivedBytes[index++] = rb;
        } else {
          // Buffer overflow, discard data and reset
          recvInProgress = false;
          index = 0;
          Serial.println("Error: Buffer overflow");
        }
      }
    } else if (rb == START_MARKER) {
      recvInProgress = true;
      index = 0; // Reset index when a new message starts
    }
  }
}

void controlMotors(float motorVelocities[4]) {
  for (int i = 0; i < 4; i++) {
    float speed = motorVelocities[i];

    // Limit the input speed to the expected range
    float limitedSpeed = constrain(abs(speed), minInputSpeed, maxInputSpeed);

    // Map the speed to a PWM range (minPWM - maxPWM)
    int targetSpeed = mapFloat(limitedSpeed, minInputSpeed, maxInputSpeed, minPWM, maxPWM);

    motors[i].setSpeed(targetSpeed);
    if (speed > 0) {
      motors[i].forward();
    } else if (speed < 0) {
      motors[i].backward();
    } else {
      motors[i].stop();
    }
  }
}

// Helper function to map float values
int mapFloat(float x, float in_min, float in_max, int out_min, int out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void sendEncoderData() {
  long encoderCounts[4];
  for (int i = 0; i < 4; i++) {
    encoderCounts[i] = motors[i].readEncoder();
  }
  // Send the encoder counts as binary data (4 long integers)
  Serial.write((uint8_t*)encoderCounts, sizeof(encoderCounts));
}
