// robot_state.h
// Public definitions for the robot behaviour state machine.
// This header only defines the possible high-level modes the robot
// can be in and exposes the shared variable used by the sketch.
//
// Notes:
// - The enum lists abstract states used by the main control loop
//   (e.g. IDLE, ROTATING, SCAN). Keep values stable if you persist
//   state or communicate modes across boards.
// - The `currentMode` variable is declared extern here and defined in
//   the main sketch file. Consider encapsulating mode in a controller
//   object for larger projects.

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

// High-level robot movement modes used by the control state machine.
enum RobotMode { IDLE, ROTATING, MOVING, TURN_180, SCAN, TURN_TO_TARGET, DRIVE_FORWARD, TURN_BACK };

// Shared state variable that represents the robot's current mode.
// It is defined in the main sketch (Robot_code.ino) and referenced
// by other modules via this extern declaration.
extern RobotMode currentMode;

#endif // ROBOT_STATE_H
