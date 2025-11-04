#include "get_out.h"
#include <math.h>

// Interpola la tabla sensorDistance a partir del valor en mV.
// Entrada: mV (milivoltios) medidos en el pin analógico.
// Salida: distancia estimada en cm basada en la tabla de calibración.
float getSensorDistance(float mV) {
	int te = *robot.tableEntries;
	int mvint = *robot.mVinterval;
	// Si el mV es mayor que el máximo registrado, devolvemos la última entrada
	if (mV > mvint * te - 1) return robot.sensorDistance[te - 1];
	int index = mV / mvint;
	float frac = fmod(mV, (float)mvint) / (float)mvint;
	return robot.sensorDistance[index] - ((robot.sensorDistance[index] - robot.sensorDistance[index + 1]) * frac);
}

// Lee el ADC y convierte a distancia en cm usando la tabla/interpolación.
float getDistanceFromSensor() {
	float referenceMv = 1000.0; // referencia de mV usada en el mapeo ADC->mV
	float val = analogRead(robot.sensorPin);
	float mV = (val * referenceMv) / 1023.0;
	return getSensorDistance(mV);
}

// Convierte un ángulo (grados) a pasos del motor necesarios para girar el
// robot en ese ángulo (aproximación basada en la geometría y resolución).
long computeSteps(float angle) {
	return (long)(angle * (*robot.stepperResolution) * (*robot.stepToMicrostep) * (*robot.WHEELS_AXIS_CM) / ((*robot.WHEEL_RADIUS_CM) * 720.0));
}

// Realiza un micro-paso en los motores en la dirección indicada.
// dir: 'F' forward, 'B' backward, 'R' rotate right (in-place), 'L' rotate left
void stepMotors(char dir) {
	AF_Stepper* r = robot.motorR;
	AF_Stepper* l = robot.motorL;
	if (dir == 'F') { r->step(1, FORWARD, SINGLE); l->step(1, FORWARD, SINGLE); }
	if (dir == 'B') { r->step(1, BACKWARD, SINGLE); l->step(1, BACKWARD, SINGLE); }
	if (dir == 'R') { r->step(1, BACKWARD, SINGLE); l->step(1, FORWARD, SINGLE); }
	if (dir == 'L') { r->step(1, FORWARD, SINGLE);  l->step(1, BACKWARD, SINGLE); }
}

