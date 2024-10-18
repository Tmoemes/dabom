#include <Arduino.h>
#include "config.h"

void processSerialInput(HardwareSerial &thisserial);
void getSerialInput();
void printMotorInfo();
void sendEncoder();

long currentTime = 0;

// debug output timing
long d_previousTime = 0;

// pi comm timing
long p_previousTime = 0;

long lastReceivedTime = 0;

void setup()
{
  // Used to display debug information
  Serial.begin(DEBUG_BAUD_RATE);

  // Serial for communication with Raspberry Pi
  Serial2.begin(PI_BAUD_RATE);

  lastReceivedTime = millis();
  // Wait for Serial Monitor to be opened
  while (!Serial || !Serial2)
  {
    // do nothing
  }
  Serial.println("Serial Ready");

  for (size_t i = 0; i < 4; i++)
  {
    motors[i].begin();
    Serial.println("Motor ");
    Serial.print(i);
    Serial.println(" started");
  }

  Serial.println("Setup Complete");
}

void loop()
{
  currentTime = millis();

  if (currentTime - d_previousTime >= DEBUG_FREQ && DEBUG_FREQ != 0)
  {
    printMotorInfo();
    d_previousTime = currentTime;
  }

  if (currentTime - lastReceivedTime >= PI_COMM_TIMEOUT)
  {
    Serial.println("No command received, stopping motors");
    lastReceivedTime = currentTime;
    for (int i = 0; i < 4; i++)
    {
      motors[i].stop();
    }
  }

  if (currentTime - p_previousTime >= PI_COMM_FREQ)
  {

    p_previousTime = currentTime;
    sendEncoder();
  }

  // processSerialInput(Serial);
  processSerialInput(Serial2);
}

void processSerialInput(HardwareSerial &thisserial)
{
  if (thisserial.available())
  {
    char input[32] = {0};
    thisserial.readBytesUntil('\n', input, 31);
    input[31] = '\0';                   // Ensure null-terminated string
    char *command = strtok(input, " "); // tokenise input on spaces
    if (command != nullptr && strcmp(command, "m") == 0)
    {
      char *motorStr = strtok(NULL, " ");
      char *speedStr = strtok(NULL, " ");
      if (motorStr != nullptr && speedStr != nullptr)
      {

        unsigned int motor = atoi(motorStr);

        // check if motor number is valid
        if (motor < 0 || motor >= sizeof(motors) / sizeof(Motor))
        {
          Serial.println("Invalid motor number");
          return;
        }

        // convert speed string to double
        double speed = atof(speedStr);

        // map input speed to 0-255
        int targetSpeed = map(abs(speed), 0, 10, 50, 255);

        // set motor speed and direction
        motors[motor].setSpeed(targetSpeed);
        if (speed > 0){motors[motor].forward();}
        else if (speed < 0){motors[motor].backward();}
        else{motors[motor].stop();}

        // update last received time
        lastReceivedTime = currentTime;
      }
      else
      {
        Serial.println("Invalid input format");
      }
    }
    else
    {
      // Serial.print("Unknown command: ");
      // Serial.print(input);
      // Serial.println();
    }
  }
}

void printMotorInfo()
{
  char message[320] = {0};
  int x = snprintf(message, sizeof(message), "\nCurrent motor Values---------------\n");
  for (size_t i = 0; i < 4; i++)
  {
    long count = motors[i].readEncoder();
    double speed = motors[i].getSpeed();
    x += snprintf(message + x, sizeof(message), "Motor %i: \n-Speed: %d \n-Encoder Value: %li\n",i, speed, count);
    Serial.print(message);
  }
}

void sendEncoder()
{
  long count = motors[0].readEncoder();
  long count1 = motors[1].readEncoder();
  long count2 = motors[2].readEncoder();
  long count3 = motors[3].readEncoder();

  char message[64] = {0};
  snprintf(message, sizeof(message), "%li,%li,%li,%li", count, count1, count2, count3);
  Serial2.println(message);
}
