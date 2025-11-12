// GetOut.h
// Small helper responsible for reading a distance sensor, converting
// analog readings into distance values using a lookup table and
// providing simple motor stepping utilities used during scans and
// short movements.
//
// This class uses a SensorContext pointer (non-owning) that points to
// calibration data and the hardware objects it needs (servo, motors).

#ifndef GETOUT_H
#define GETOUT_H

#include <Arduino.h>
#include "../robot_shared.h"

class GetOut {
public:
  // Construct with a SensorContext that must remain valid for the
  // lifetime of this object.
  GetOut(SensorContext* ctx);

  // Convert a voltage in mV (from the analog sensor) to a distance in cm
  // using the linear interpolation of the calibration table.
  float getSensorDistance(float mV);

  // Read the analog sensor and return interpolated distance in cm.
  float getDistanceFromSensor();

  // Compute the number of micro-steps needed to rotate the robot by
  // the given angle (degrees). Uses wheel geometry from the context.
  long computeSteps(float angle);

  // Issue a single-step command to both steppers in the specified
  // direction: 'F' forward, 'B' backward, 'R' rotate right, 'L' rotate left.
  void stepMotors(char dir);

private:
  SensorContext* ctx; // non-owning pointer to sensor-related data
};

#endif // GETOUT_H
