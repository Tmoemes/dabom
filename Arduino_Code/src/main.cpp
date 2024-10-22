#include <Arduino.h>
#include "config.h"

// Uncomment the following line to enable debugging
// #define DEBUG_ENABLED

void processBinarySerialInput();
void sendEncoderData();
void controlMotors(float motorVelocities[4]);
int mapFloat(float x, float in_min, float in_max, int out_min, int out_max);
void sendDebugMessage(const char* message);

long currentTime = 0;
long lastUpdateTime = 0;
long lastReceivedTime = 0;

const long updateInterval = 8; // 8 ms for 125 Hz

// Communication markers
const byte START_MARKER = 0x02;  // For binary data
const byte END_MARKER = 0x03;
const byte DEBUG_MARKER = 0x04;  // For debug messages

void setup()
{
  // Initialize serial communication with the ROS2 node
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial port to be ready
  }

  // Initialize motors
  for (size_t i = 0; i < 4; i++) {
    motors[i].begin();
    delay(10); // Small delay for safety
  }

#ifdef DEBUG_ENABLED
  sendDebugMessage("Setup Complete");
#endif

  lastReceivedTime = millis();
  lastUpdateTime = millis();
}

void loop()
{
  currentTime = millis();

  // Read serial data as soon as it's available
  processBinarySerialInput();

  // Update at 125 Hz
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
#ifdef DEBUG_ENABLED
            sendDebugMessage("Error: Checksum mismatch");
#endif
          }
        } else {
#ifdef DEBUG_ENABLED
          sendDebugMessage("Error: Incorrect message length");
#endif
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
#ifdef DEBUG_ENABLED
          sendDebugMessage("Error: Buffer overflow");
#endif
        }
      }
    } else if (rb == START_MARKER) {
      recvInProgress = true;
      index = 0; // Reset index when a new message starts
    } else if (rb == DEBUG_MARKER) {
      // Ignore debug markers received from ROS2 node (if any)
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

void sendEncoderData() {
  long encoderCounts[4];
  for (int i = 0; i < 4; i++) {
    encoderCounts[i] = motors[i].readEncoder();
  }
  // Send the encoder counts as binary data (4 long integers) with markers and checksum
  byte dataBytes[16];
  memcpy(dataBytes, encoderCounts, 16);

  // Calculate checksum
  byte checksum = 0;
  for (int i = 0; i < 16; i++) {
    checksum ^= dataBytes[i];
  }

  // Construct packet
  Serial.write(START_MARKER);
  Serial.write(dataBytes, 16);
  Serial.write(checksum);
  Serial.write(END_MARKER);

#ifdef DEBUG_ENABLED
  // Send encoder values as a debug message
  char debugMsg[100];
  snprintf(debugMsg, sizeof(debugMsg), "Encoder values: %ld, %ld, %ld, %ld", 
           encoderCounts[0], encoderCounts[1], encoderCounts[2], encoderCounts[3]);
  sendDebugMessage(debugMsg);
#endif
}

int mapFloat(float x, float in_min, float in_max, int out_min, int out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void sendDebugMessage(const char* message) {
  byte length = strlen(message);  // Length of the debug message
  Serial.write(DEBUG_MARKER);
  Serial.write(length);
  Serial.write((const uint8_t*)message, length);
  Serial.write(END_MARKER);
}
