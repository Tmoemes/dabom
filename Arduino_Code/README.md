# Hardware Installation Instructions
## Software install
### Step-by-Step Guide to Install and Upload the Project using PlatformIO

1. **Install PlatformIO:**
    - Download and install [Visual Studio Code](https://code.visualstudio.com/).
    - Open Visual Studio Code and go to the Extensions view by clicking on the square icon in the sidebar or pressing `Ctrl+Shift+X`.
    - Search for "PlatformIO IDE" and click "Install".

2. **Open the Project in Visual Studio Code:**
    - Open Visual Studio Code.
    - Click on `File` > `Open Folder...` and select the `Arduino_Code` directory from the cloned repository.

3. **Install Dependencies:**
    - Open the PlatformIO Home by clicking on the house icon in the sidebar.
    - Click on "Open Project" and select the `Arduino_Code` directory.
    - PlatformIO will automatically detect the `platformio.ini` file and install the necessary dependencies.

4. **Configure the Project:**
    - Open the `platformio.ini` file in the `Arduino_Code` directory.
    - Ensure the configuration matches your setup. For example:
      ```ini
      [env:megaatmega2560]
        platform = atmelavr
        board = megaatmega2560
        framework = arduino
        monitor_speed = 19200
        lib_deps = 
	        paulstoffregen/Encoder@^1.4.4
      ```

5. **Upload the Code to Arduino Mega:**
    - Connect your Arduino Mega to your computer using a USB cable.
    - In Visual Studio Code, open the PlatformIO sidebar by clicking on the alien icon.
    - Click on the "Upload" button (arrow pointing right) to compile and upload the code to the Arduino Mega.

6. **Monitor Serial Output (Optional):**
    - To monitor the serial output from the Arduino, click on the "Serial Monitor" button (plug icon) in the PlatformIO sidebar.
    - Ensure the baud rate matches the one set in your code (e.g., `19200`).

Your project should now be uploaded to the Arduino Mega and running as expected.

## Wiring for Power and Signals

### Arduino <-> Raspberry Pi Communication
The Pi and Arduino communicate using the hardware serial of the Pi to the Serial2 of the Arduino Mega.
In this case we are also powering the Arduino from the Pi 5V pins.
Make sure the Pi and Arduino are grounded together for serial communication to funcion properly 
1. **Raspberry Pi GPIO Connections:**
    - `GPIO 14 (TXD)` to `Arduino Pin 16 (TX2)`
    - `GPIO 15 (RXD)` to `Arduino Pin 17 (RX2)`

### Example Wiring Diagram for Arduino <-> Raspberry Pi Communication
```
+------------------+       +------------------+
| Raspberry Pi     |       | Arduino Mega     |
|                  |       |                  |
|   GPIO 14 (TXD)  +------>+ 17 (RX2)         |
|   GPIO 15 (RXD)  +------>+ 16 (TX2)         |
+------------------+       +------------------+
```

### Motor and Encoder Connections
For the motors, it is necessary to connect the `EN` connection to PWM-capable pins to modulate speed. On the Arduino Mega, the PWM-capable pins are `D2-D13` and `D44-D46`.

For the encoders, it is important to connect at least one of the channels to an interrupt-capable pin. The Arduino Mega has 6 available interrupts:
- `INT.0` on Pin `21`
- `INT.1` on Pin `20`
- `INT.2` on Pin `19`
- `INT.3` on Pin `18`
- `INT.4` on Pin `2`
- `INT.5` on Pin `3`

Make sure the Arduino is grounded together with the motor drivers and the encoders to ensure proper signaling.

1. **Motor Connections:**  
    - **Motor 0 (Front Left):**
        - `EN` pin to `D2`
        - `IN1` pin to `D35`
        - `IN2` pin to `D37`
    - **Motor 1 (Front Right):**
        - `EN` pin to `D3`
        - `IN1` pin to `D41`
        - `IN2` pin to `D39`
    - **Motor 2 (Back Left):**
        - `EN` pin to `D4`
        - `IN1` pin to `D31`
        - `IN2` pin to `D33`
    - **Motor 3 (Back Right):**
        - `EN` pin to `D5`
        - `IN1` pin to `D45`
        - `IN2` pin to `D43`

2. **Encoder Connections:**
    - **Encoder 0 (Front Left):**
        - `A` pin to `D18`
        - `B` pin to `D23`
    - **Encoder 1 (Front Right):**
        - `A` pin to `D19`
        - `B` pin to `D25`
    - **Encoder 2 (Back Left):**
        - `A` pin to `D20`
        - `B` pin to `D27`
    - **Encoder 3 (Back Right):**
        - `A` pin to `D21`
        - `B` pin to `D29`

### Example Wiring Diagram for Motors and Encoders
```
+------------------+       +------------------+
| Motor 0          |       | Arduino          |
|                  |       |                  |
|   EN             +------>+ D2               |
|   IN1            +------>+ D35              |
|   IN2            +------>+ D37              |
+------------------+       +------------------+

+------------------+       +------------------+
| Encoder 0        |       | Arduino          |
|                  |       |                  |
|   A              +------>+ D18              |
|   B              +------>+ D23              |
+------------------+       +------------------+

+------------------+       +------------------+
| Motor 1          |       | Arduino          |
|                  |       |                  |
|   EN             +------>+ D3               |
|   IN1            +------>+ D41              |
|   IN2            +------>+ D39              |
+------------------+       +------------------+

+------------------+       +------------------+
| Encoder 1        |       | Arduino          |
|                  |       |                  |
|   A              +------>+ D19              |
|   B              +------>+ D25              |
+------------------+       +------------------+

+------------------+       +------------------+
| Motor 2          |       | Arduino          |
|                  |       |                  |
|   EN             +------>+ D4               |
|   IN1            +------>+ D31              |
|   IN2            +------>+ D33              |
+------------------+       +------------------+

+------------------+       +------------------+
| Encoder 2        |       | Arduino          |
|                  |       |                  |
|   A              +------>+ D20              |
|   B              +------>+ D27              |
+------------------+       +------------------+

+------------------+       +------------------+
| Motor 3          |       | Arduino          |
|                  |       |                  |
|   EN             +------>+ D5               |
|   IN1            +------>+ D45              |
|   IN2            +------>+ D43              |
+------------------+       +------------------+

+------------------+       +------------------+
| Encoder 3        |       | Arduino          |
|                  |       |                  |
|   A              +------>+ D21              |
|   B              +------>+ D29              |
+------------------+       +------------------+
```