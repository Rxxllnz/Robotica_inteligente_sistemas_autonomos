#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

// Shared robot movement states
enum RobotMode { IDLE, ROTATING, MOVING, TURN_180, SCAN, TURN_TO_TARGET, DRIVE_FORWARD, TURN_BACK };

// single shared state variable (defined in Robot_code.ino)
extern RobotMode currentMode;

#endif // ROBOT_STATE_H
