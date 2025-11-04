#ifndef GET_OUT_H
#define GET_OUT_H

#include <Arduino.h>
#include <DMotor_mod.h>
#include "../robot_shared.h"

// Prototypes for helper functions moved from Robot_code.ino
float getSensorDistance(float mV);
float getDistanceFromSensor();
long computeSteps(float angle);
void stepMotors(char dir);

#warning "get_out: use 'robot' from robot_shared.h to access shared globals"
#endif // GET_OUT_H
