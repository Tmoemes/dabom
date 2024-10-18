# Hardware Installation Instructions

## Wiring for Power and Signals

### Motor and Encoder Connections
For the motors, it is necessary to connect the `EN` connection to PWM-capable pins to modulate speed. On the Arduino Mega, the PWM-capable pins are `D2-D13` and `D44-D46`.

For the encoders, it is important to connect at least one of the channels to an interrupt-capable pin. The Arduino Mega has 6 available interrupts:
- `INT.0` on Pin `21`
- `INT.1` on Pin `20`
- `INT.2` on Pin `19`
- `INT.3` on Pin `18`
- `INT.4` on Pin `2`
- `INT.5` on Pin `3`

1. **Motor Connections:**  - **Motor 0 (Front Left):**
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