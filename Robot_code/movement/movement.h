// Movement.h
// Helper class responsible for converting high-level movement
// commands (angles and distances) into low-level stepper commands
// and for handling remote movement requests.
//
// Design notes:
// - This class receives only a MovementContext pointer which contains
//   references to the minimal set of variables and hardware objects
//   required for movement calculations. That keeps the interface small
//   and testable.

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "../robot_shared.h"
#include "../robot_state.h"

class Movement {
public:
  // Construct with a MovementContext which must remain valid for the
  // lifetime of this object. No ownership is taken; the caller owns
  // the context storage.
  Movement(MovementContext* ctx);

  // Convert an in-place rotation angle (degrees) into the number of
  // micro-steps required by the wheel steppers.
  long degreesToSteps(float angleDeg);

  // Convert a forward travel distance (cm) into micro-steps.
  long distanceCmToSteps(float distCm);

  // Compute the shortest signed rotation (degrees) from `fromDeg` to
  // `toDeg` in the range [-180, 180]. Static because it does not
  // depend on instance data.
  static float shortestAngle(float fromDeg, float toDeg);

  // Process a remote command: distance (cm), angle (deg) and an
  // optional out-flag. This method updates shared movement fields
  // via the MovementContext so the main loop can act on them.
  void processRemoteCommand(float distCm, float angleDeg, float out);

  void setGetOutMode(bool mode) { getoutMode = mode;}
  bool getGetOutMode() { return getoutMode; }

private:
  MovementContext* ctx; // non-owning pointer to movement-related fields
  bool lastOutFlag = false; // track last out-flag state
  bool getoutMode = false; // track if in get-out mode
};

#endif // MOVEMENT_H
