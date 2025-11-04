#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "../robot_state.h"
#include "../robot_shared.h"

// Functions moved from Robot_code.ino
long degreesToSteps(float angleDeg);
long distanceCmToSteps(float distCm);
float shortestAngle(float fromDeg, float toDeg);
void processRemoteCommand(float distCm, float angleDeg, float out);

// movement public API (the implementation will use the centralized `robot` struct
// declared in `robot_shared.h` to access shared globals)

#endif // MOVEMENT_H