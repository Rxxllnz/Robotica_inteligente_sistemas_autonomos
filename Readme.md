% RingBots — Autonomous Robot Pushing Competition

RingBots is an educational project that implements autonomous robots competing inside a ring arena. The goal of this repository is to provide firmware and helper modules for the robot units; vision processing (camera, detection and pose estimation) is implemented in a separate repository which sends high-level commands to the robots.

Status
------
Active development. Several hardware-specific details and the external vision repository link are still TODO. The firmware includes a built-in `TEST_MODE` to support offline testing on a single board.

Repository layout
-----------------
- `Robot_code/`
  - `Robot_code.ino` — main Arduino-style sketch containing the non-blocking state machine
  - `get_out/` — small helpers for sensor interpolation and stepping used during scans and maneuvers
  - `movement/` — movement utilities and kinematic conversions
  - `robot_shared.h` — lightweight context structs used to pass dependencies into modules
  - `robot_state.h` — `RobotMode` enum used by the main loop
- `movement_v1/` — legacy/experimental movement sketch
- `Sensores-actuadores/` — individual sensor/actuator examples

Hardware overview (fill in exact models)
---------------------------------------
- MCU: ESP8266 or ESP32 (select appropriate board core in Arduino IDE)
- Steppers: stepper driver and motor model (TBD)
- Servo: model used to sweep the distance sensor (TBD)
- Distance sensor: analog sensor with a calibration table included in code
- Power: battery and power distribution (TBD)

Pins used by the firmware
-------------------------
- `ledPin` — D0 (status LED)
- `servoPin` — D7 (servo used for sensor sweeping)
- `sensorPin` — A0 (analog input for distance sensor)

Key configuration values
------------------------
- Wheel radius: 3.2 cm
- Wheel base (axis): 16.0 cm
- Stepper resolution: 256 (`stepperResolution`)
- Microsteps multiplier: `stepToMicrostep = 8`
- Default stepper base speed: `baseSpeed = 100` (driver-specific units)

High-level architecture
-----------------------
1. External vision system (separate repository) processes camera frames and computes robot poses.
2. Vision → Master ESP: vision transmits poses/commands (planned transport: UDP).
3. Master ESP forwards high-level commands to robot ESPs.
4. Robot ESPs (this repo) execute a non-blocking state machine (scan, align, drive) and perform low-level stepping and safety checks.

Firmware behaviour and modes
---------------------------
- Non-blocking state machine with modes: `IDLE`, `ROTATING`, `MOVING`, `TURN_180`, `SCAN`, `TURN_TO_TARGET`, `DRIVE_FORWARD`, `TURN_BACK`.
- Servo scans and linear interpolation are used to map analog readings to distances using a calibration table.
- Movement calculations convert distances and angles into micro-step counts using wheel geometry and a function that returns steps per revolution.

Quickstart — build & flash (Arduino IDE)
--------------------------------------
Prerequisites:
- Arduino IDE (or PlatformIO)
- Board core for the chosen ESP (ESP8266/ESP32)
- USB drivers
- Required libraries: `DMotor_mod`, `Servo_ESP8266`, `AF_Stepper` (verify versions and compatibility)

Steps:
1. Open `Robot_code/Robot_code.ino` in the Arduino IDE.
2. Select board and port (Tools → Board, Tools → Port).
3. Verify / Compile.
4. Upload to the board.
5. Open the Serial Monitor at `115200` baud to view runtime logs.

Notes
-----
- The sketch enables `TEST_MODE` by default. Disable it for real competition and provide remote command input instead.
- The Arduino IDE compiles `movement.cpp` and `get_out.cpp` because they are `#include`d by the sketch. If you reorganize files, ensure the translation units are compiled or convert them into a library for cleaner builds.

Development notes
-----------------
- The code uses context structs (`MovementContext`, `SensorContext`) so modules receive only the dependencies they need. This reduces coupling and clarifies ownership.
- Keep any ISR handlers minimal; the main loop performs non-blocking scheduling using `millis()`.


Maintainers / Authors
---------------------
TODO
