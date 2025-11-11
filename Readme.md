# RingBots — Autonomous Robot Pushing Competition

Short description
-----------------
RingBots is an educational autonomous robotics project where two robots compete in a ring to push the opponent out. The system is split between a vision repository (external camera software, Python) that computes positions/angles/distances and this repository containing the robot firmware and helpers. The project is intended for students, educators and makers learning embedded systems, motor control and vision-based coordination.

Status
------
Active development. Hardware details (ESP model, motors, batteries), arena rules and the external vision repo URL are TODO. This README documents current firmware structure and how the robot code expects remote commands.

Repository layout (current)
---------------------------
- Robot_code/
  - Robot_code.ino         (main Arduino-style firmware)
  - get_out/               (module for "get out" or special maneuvers)
  - movement/              (movement helpers included by the sketch)
  - robot_shared.h         (shared state struct)
  - robot_state.h          (movement state enum)


Architecture overview
---------------------
- External vision system (separate repo): processes frames, detects robots and ring, and computes position/orientation data.
- Vision -> Master ESP: UDP is the planned transport (low-latency). Vision sends structured messages with robot poses and more info.
- Master ESP: receives vision data and forwards commands to the other two robots.
- Robot ESPs (this firmware): receive high-level commands and execute low-level motor control and safety logic.

Firmware (concrete details)
-----------------------------------------------------
From Robot_code/Robot_code.ino and headers:

Pins and identifiers
- ledPin: D0
- servoPin: D7
- sensorPin: A0

Actuators and sensors
- Motors: TODO
- Servo: TODO
- Sensor distance 

Movement & kinematics
- Wheel radius: 3.2 cm
- Wheel base (axis): 16.0 cm
- Stepper resolution: 256 (variable stepperResolution)
- Microsteps multiplier: stepToMicrostep = 8
- Base speed (stepper): baseSpeed = 100 (%)

Modes (RobotMode enum)
- IDLE, ROTATING, MOVING, TURN_180, SCAN, TURN_TO_TARGET, DRIVE_FORWARD, TURN_BACK


Remote command interfaces present in code
- processRemoteCommand(dist, angle, out) is invoked in test/serial flows (implemented in included modules).
- getRemoteCommand: TODO

Timing and rates
- Code sampling intervals:
  - Servo scan dwell: ~250 ms per angle
  - Motor micro-step executor runs at motorDelay (1 ms default) 
- Vision send rate: TODO

Quickstart — build & flash (Arduino IDE)
----------------------------------------
Prerequisites:
- Arduino IDE
- Appropriate board core for the chosen ESP module (install when you decide the model)
- USB drivers for your board
- Required Arduino libraries: DMotor_mod, Servo_ESP8266, AF_Stepper (ensure versions compatible with ESP)

Build & flash:
1. Open Robot_code/Robot_code.ino in Arduino IDE.
2. Select the correct board (Tools → Board) for your ESP.
3. Select the correct Port (Tools → Port).
4. Verify / Compile.
5. Upload.

Notes:
- The sketch includes TEST_MODE by default (#define TEST_MODE 1) which runs the internal test command list. Disable TEST_MODE for real operation and enable appropriate network command input in your master code.
- The sketch currently includes movement/movement.cpp and get_out/get_out.cpp via #include lines. If you reorganize files, ensure those translation units are compiled or converted into libraries.

Maintainers / Authors
---------------------
TODO
