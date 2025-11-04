// Include the header file for external references
#include "movement.h"
#include <DMotor_mod.h>
#include <math.h>

// Convierte grados de giro a número de micro-pasos por rueda (rotación in-place)
// Entrada:
//  - angleDeg: ángulo de rotación (grados). Puede ser negativo para indicar
//    sentido contrario.
// Salida:
//  - número de micro-pasos (long) que debe girar cada rueda para lograr el
//    giro in-place pedido.
// Nota: usa las constantes físicas (WHEEL_BASE_MM, WHEEL_DIAMETER_MM) y la
// función stepsPerRevMicro() para calcular la resolución real de los steppers.
long degreesToSteps(float angleDeg) {
	float angle = fabs(angleDeg);
	// Longitud del arco que recorre cada rueda (mm)
	float arc_mm = (PI * (*robot.WHEEL_BASE_MM)) * (angle / 360.0);
	// vueltas de rueda necesarias
	float rotations = arc_mm / (PI * (*robot.WHEEL_DIAMETER_MM));
	float steps = rotations * (float)robot.stepsPerRevMicro();
	return (long)(steps + 0.5);
}

// Convierte distancia en cm a pasos por rueda (avance linear)
// Convierte una distancia (cm) en pasos de microstepping por rueda.
// Entrada:
//  - distCm: distancia a recorrer en centímetros (lineal).
// Salida:
//  - número de micro-pasos por rueda necesarios para desplazar el robot
long distanceCmToSteps(float distCm) {
	float dist_mm = distCm * 10.0;
	float rotations = dist_mm / (PI * (*robot.WHEEL_DIAMETER_MM));
	float steps = rotations * (float)robot.stepsPerRevMicro();
	return (long)(steps + 0.5);
}

// Normaliza la diferencia entre dos headings a un rango de [-180, 180] grados.
// Util: para elegir el giro más corto entre dos orientaciones.
float shortestAngle(float fromDeg, float toDeg) {
	float d = toDeg - fromDeg;
	while (d > 180.0) d -= 360.0;
	while (d < -180.0) d += 360.0;
	return d;
}

// ------------------------------------------------------------------
// Procesa un comando remoto (distancia y ángulo) y prepara la máquina de
// estados para ejecutar la maniobra.
// - distCm: distancia objetivo en centímetros (si 0, no avanzar)
// - angleDeg: valor del giro solicitado (grados, puede ser positivo/negativo)
// - out: flag auxiliar (p.e. 1 para modo especial TURN_180)
// Efectos secundarios:
//  - actualiza campos en `robot` (desiredDistanceCm, desiredHeading, etc.)
//  - ajusta `currentMode`, `targetSteps*`, `dir*` y resets de `doneSteps*`.

void processRemoteCommand(float distCm, float angleDeg, float out) {
	if (out == 1) {
		// Modo especial: forzar un giro de 180° (ejemplo de comando remoto)
		Serial.print(out);
		currentMode = TURN_180;
	} else {
		// Guardar pedido de distancia
		*robot.desiredDistanceCm = distCm;

		// El ángulo recibido es directamente cuánto girar
		float dAng = angleDeg;
		// Calcular heading absoluto deseado (normalizado 0..360)
		*robot.desiredHeading = fmod((*robot.currentHeading) + dAng, 360.0);
		if (*robot.desiredHeading < 0) *robot.desiredHeading += 360.0;
		long rotSteps = degreesToSteps(dAng);

		if (rotSteps > 0) {
			// Preparar rotación in-place: ruedas en sentidos opuestos
			if (dAng > 0) { *robot.dirR = BACKWARD; *robot.dirL = FORWARD; }
			else { *robot.dirR = FORWARD; *robot.dirL = BACKWARD; }
			*robot.targetStepsR = rotSteps; *robot.targetStepsL = rotSteps;
			*robot.doneStepsR = 0; *robot.doneStepsL = 0;
			currentMode = ROTATING;
			Serial.print("[REMOTE] Preparado ROTATE pasos="); Serial.println(rotSteps);
		} else {
			// Ya orientado: preparar avance
			long moveSteps = distanceCmToSteps(*robot.desiredDistanceCm);
			if (moveSteps > 0) {
				*robot.dirR = FORWARD; *robot.dirL = FORWARD;
				*robot.targetStepsR = moveSteps; *robot.targetStepsL = moveSteps;
				*robot.doneStepsR = 0; *robot.doneStepsL = 0;
				currentMode = MOVING;
				Serial.print("[REMOTE] Preparado MOVE pasos="); Serial.println(moveSteps);
			} else {
				currentMode = IDLE;
			}
		}
	}
}

