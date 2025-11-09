// === Librerías necesarias ===
#include <DMotor_mod.h>
#include <Servo_ESP8266.h>
#include <math.h>
// Movement helpers moved to movement/movement.cpp
#include "movement/movement.h"
// Helpers moved to get_out module
#include "get_out/get_out.h"
// Centralized shared pointers for other modules
#include "robot_shared.h"

// ---------------------------------------------------------------------------
// Nota: las implementaciones de los módulos `movement` y `get_out` están
// ubicadas en subcarpetas. Para garantizar que el linkeo incluya sus símbolos
// cuando se compila el sketch con el IDE de Arduino, incluimos los .cpp aquí.
// Esto preserva la organización en carpetas sin requerir crear una librería
// separada. Si más adelante conviertes esto en una librería, puedes quitar
// estas inclusiones y dejar los .cpp como archivos compilables independientes.
// ---------------------------------------------------------------------------
#include "movement/movement.cpp"
#include "get_out/get_out.cpp"

// === Pines ===
#define ledPin D0
#define servoPin D7
#define sensorPin A0

// === Identificador del robot ===
const unsigned int id = 0x01; // ID único del robot
bool outFlag = false; // flag auxiliar para modo especial

// === Motores y servo ===
AF_Stepper motorR(256, 1);    // Motor derecho
AF_Stepper motorL(256, 2);    // Motor izquierdo
Servo_ESP8266 sensorServo;    // Servo del sensor IR

// === Parámetros de movimiento ===
int stepToMicrostep = 8;      // 8 micro-pasos por paso
int baseSpeed = 100;          // velocidad base de los stepper

// === Variables para control de velocidad ===
unsigned long previousMillisSensor = 0;
unsigned long previousMillisMotors = 0;
const unsigned long intervalSensor = 50;  // lectura sensor cada 50 ms
unsigned long motorDelay = 1;            // delay entre pasos (MENOS = MÁS RÁPIDO)

// === MODO DE PRUEBA (TEST_MODE) ===
// Activa un simulador local de comandos (distancia cm, angulo °).
#define TEST_MODE 1

// TEST_MODE funciona en modo auto-run 
const unsigned long testAutoInterval = 10000; // ms entre comandos
bool testAutoRun = true; // si true avanza automáticamente cada testAutoInterval

// SERIAL STUB: leer comandos desde el Monitor Serie cuando TEST_MODE == 0
// Formato: "<dist_cm> <angle_deg>" por línea, p.e. "30 90" o "D:30,A:90"
#define SERIAL_STUB 0

// Lista de comandos de prueba: {distancia_cm, angulo_deg}
const int TEST_CMD_COUNT = 6;
float testCommands[TEST_CMD_COUNT][3] = {
  {30.0, 0.0, 0.0},   // avanzar 30cm, heading 0°
  {20.0, 90.0, 0.0},  // avanzar 20cm, heading 90°
  {15.0, 180.0, 0.0}, // avanzar 15cm, heading 180°
  {10.0, 270.0, 0.0}, // avanzar 10cm, heading 270°
  {5.0, 45.0, 0.0},   // avanzar 5cm, heading 45°
  {0.0, 0.0, 1.0}     // giro a 0°, sin avance
};
int testIndex = 0;
unsigned long previousMillisTest = 0;

// === Parámetros del robot  ===
// Datos físicos del LLUBot (valores proporcionados):
const float WHEEL_RADIUS_CM = 3.2;      // en cm (radio de rueda)
const float WHEELS_AXIS_CM = 16.0;      // distancia entre ejes en cm
// convertimos a mm y diámetro
const float WHEEL_DIAMETER_MM = WHEEL_RADIUS_CM * 2.0 * 10.0; // cm->mm
const float WHEEL_BASE_MM = WHEELS_AXIS_CM * 10.0;          // cm->mm
float twoPi = 2 * 3.1416;

// Resolución del stepper (proporcionada)
int stepperResolution = 256;  // 8 bits, corresponde al primer parámetro de AF_Stepper

// Pasos por revolución (incluyendo microsteps)
long stepsPerRevMicro() { return (long)stepperResolution * (long)stepToMicrostep; }

// Máquina de estados para movimiento (no bloqueante)
// RobotMode enum and extern are declared in robot_state.h
RobotMode currentMode = IDLE;

// Objetivos de pasos para cada rueda
long targetStepsR = 0;
long targetStepsL = 0;
long doneStepsR = 0;
long doneStepsL = 0;

// Direcciones actuales para stepper
int dirR = FORWARD;
int dirL = FORWARD;

// Heading actual (0..360), y objetivo absoluto
float currentHeading = 0.0;
float desiredHeading = 0.0;

// Distancia objetivo en cm (último comando recibido)
float desiredDistanceCm = 0.0;

// === Parámetros del sensor ===
const int tableEntries = 10;
const int mVinterval = 100;
static float sensorDistance[tableEntries] = {80.0,70.0,60.0,50.0,40.0,30.0,25.0,20.0,15.0,10.0};

// --- Parameters ---
const float SAFE_DISTANCE = 27.0; // Stop distance before wall
const int SCAN_ANGLES[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180};
const int NUM_SCAN_ANGLES = sizeof(SCAN_ANGLES) / sizeof(SCAN_ANGLES[0]);

// --- Movement tracking ---
bool moving = false;
unsigned long lastActionTime = 0;
unsigned long lastStepTime = 0;
unsigned long servoMoveStart = 0;
int currentScanIndex = 0;
float scanDistances[NUM_SCAN_ANGLES];
float lastDistance = 0;
int targetAngle = 90;
long currentSteps = 0;
long targetSteps = 0;

// Helper functions moved to Robot_code/get_out/get_out.cpp
// Initialize the robot shared struct so other translation units can access
// the actual globals via the single extern `robot`.
RobotShared robot = {
  stepsPerRevMicro,
  &WHEEL_BASE_MM,
  &WHEEL_DIAMETER_MM,
  &currentHeading,
  &desiredHeading,
  &desiredDistanceCm,
  &targetStepsR,
  &targetStepsL,
  &doneStepsR,
  &doneStepsL,
  &dirR,
  &dirL,
  &stepperResolution,
  &stepToMicrostep,
  &tableEntries,
  &mVinterval,
  sensorDistance,
  sensorPin,
  &WHEEL_RADIUS_CM,
  &WHEELS_AXIS_CM,
  &motorR,
  &motorL,
  &sensorServo,
  &outFlag
};

// Movement helpers were moved to movement/movement.cpp

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Inicializar servo en 90º
  sensorServo.attach(servoPin);
  sensorServo.write(90);

  // Configurar velocidad de motores
  motorR.setSpeed(baseSpeed);
  motorL.setSpeed(baseSpeed);

  #if TEST_MODE
    previousMillisTest = millis();
    Serial.println("*** TEST_MODE habilitado: usando lista interna de comandos (auto-run) ***");
    Serial.print("Auto-run cada (ms): "); Serial.println(testAutoInterval);
  #endif
}

void loop() {
  unsigned long currentMillis = millis();

  // ----------------------
  // Punto de integración remoto
  // ----------------------
  // if ( getRemoteCommand(desiredDistanceCm, desiredHeading, outFlag, id) ) { // devuelve true si hay comando nuevo || bool getRemoteCommand(float& distanceCm, float& angleDeg, bool& outFlag, unsigned int robotId)
  //   processRemoteCommand(desiredDistanceCm, desiredHeading, outFlag);
  // }

  // SERIAL_STUB: si está activado y TEST_MODE==0, leer línea desde Serial
  // y llamar a processRemoteCommand(dist, angle).
  #if SERIAL_STUB && !TEST_MODE
    static String serialBuf = "";
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue; // ignorar CR
      if (c == '\n') {
        String line = serialBuf;
        serialBuf = "";
        line.trim();
        if (line.length() > 0) {
          // normalizar separadores
          line.replace(',', ' ');
          line.replace('D', ' ');
          line.replace('d', ' ');
          line.replace('A', ' ');
          line.replace('a', ' ');
          // tokenizar
          float vals[3]; int cnt = 0;
          int i = 0;
          while (i < line.length() && cnt < 3) {
            while (i < line.length() && isSpace(line[i])) i++;
            if (i >= line.length()) break;
            int j = i;
            while (j < line.length() && !isSpace(line[j])) j++;
            String tok = line.substring(i, j);
            char buf[32]; tok.toCharArray(buf, sizeof(buf));
            vals[cnt++] = atof(buf);
            i = j;
          }
          if (cnt == 3) {
            float rxDist = vals[0];
            float rxAng = vals[1];
            float out = vals[2];
            Serial.print("[STUB] Recibido: "); Serial.print(rxDist); Serial.print(" cm, "); Serial.print(rxAng); Serial.println(" deg");
            processRemoteCommand(rxDist, rxAng, out);
            Serial.println("[STUB] ACK");
          } else {
            Serial.print("[STUB] Formato inválido: '"); Serial.print(line); Serial.println("' (usar: '<dist_cm> <angle_deg>' )");
          }
        }
      } else {
        serialBuf += c;
        // limitar longitud para evitar overflow
        if (serialBuf.length() > 200) serialBuf = serialBuf.substring(serialBuf.length()-200);
      }
    }
  #endif

    // En modo TEST, obtener comandos desde la lista interna (simula la comunicación)
  #if TEST_MODE
    bool haveNewCommand = false;
    // Auto-run únicamente: generar nuevo comando cuando corresponde
    if (testAutoRun && (currentMillis - previousMillisTest >= testAutoInterval)) {
      haveNewCommand = true;
    }

    if (haveNewCommand) {
      if (testCommands[testIndex][2] == 1){
        processRemoteCommand(0.0, 0.0, 1);
        testAutoRun = false; // stop auto run after get out command
      } else{
        processRemoteCommand(testCommands[testIndex][0], testCommands[testIndex][1], 0);
      }
      previousMillisTest = currentMillis;
      /*
      // cargar comando
      desiredDistanceCm = testCommands[testIndex][0];
      desiredHeading = fmod(testCommands[testIndex][1], 360.0);
      if (desiredHeading < 0) desiredHeading += 360.0;
      Serial.print("[TEST] Comando #"); Serial.print(testIndex); Serial.print(" -> dist="); Serial.print(desiredDistanceCm);
      Serial.print(" cm, ang="); Serial.println(desiredHeading);

      // Preparar rotación
      float dAng = shortestAngle(currentHeading, desiredHeading);
      long steps = degreesToSteps(dAng);
      if (steps > 0) {
        if (dAng > 0) { dirR = BACKWARD; dirL = FORWARD; }
        else { dirR = FORWARD; dirL = BACKWARD; }
        targetStepsR = steps; targetStepsL = steps; doneStepsR = 0; doneStepsL = 0; currentMode = ROTATING;
        Serial.print("[TEST] Iniciando giro (pasos): "); Serial.println(steps);
      } else {
        long moveSteps = distanceCmToSteps(desiredDistanceCm);
        if (moveSteps > 0) {
          dirR = FORWARD; dirL = FORWARD; targetStepsR = moveSteps; targetStepsL = moveSteps; doneStepsR = 0; doneStepsL = 0; currentMode = MOVING;
          Serial.print("[TEST] Iniciando avance (pasos): "); Serial.println(moveSteps);
        } else {
          currentMode = IDLE;
        }
      }*/

      // avanzar índice de test
      testIndex++;
      if (testIndex >= TEST_CMD_COUNT) testIndex = 0;
    }
  #endif


  // Máquina de stepping no bloqueante: realiza 1 micro-paso por rueda cada motorDelay ms
  // Esta sección actúa como el executor de movimientos: cada vez que toca dar
  // un micro-paso (según motorDelay) se comprueba el modo actual y se avanza
  // la máquina de estados correspondiente.
  if (currentMillis - previousMillisMotors >= motorDelay) {
    previousMillisMotors = currentMillis;

    // 1) ROTATING: ejecución de giro in-place (ruedas en sentidos opuestos)
    //    - avanzamos paso a paso hasta alcanzar targetStepsR/L
    //    - cuando ambos completan, actualizamos heading y pasamos a MOVING o IDLE
    if (currentMode == ROTATING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) {
        // Giro completado: actualizamos heading y, si hay distancia objetivo,
        // pasamos a MOVING para avanzar; en caso contrario, IDLE.
        currentHeading = desiredHeading;
        Serial.print("[TEST] Giro completado. Heading ahora: "); Serial.println(currentHeading);
        long moveSteps = distanceCmToSteps(desiredDistanceCm);
        if (moveSteps > 0) {
          // preparar avance lineal
          dirR = FORWARD; dirL = FORWARD;
          targetStepsR = moveSteps; targetStepsL = moveSteps;
          doneStepsR = 0; doneStepsL = 0;
          currentMode = MOVING;
          Serial.print("[TEST] Iniciando avance (pasos): "); Serial.println(moveSteps);
        } else {
          currentMode = IDLE;
        }
      }

    // 2) MOVING: avance lineal hasta completar los pasos calculados
    } else if (currentMode == MOVING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) {
        // movimiento lineal completado
        Serial.println("[TEST] Movimiento completado.");
        currentMode = IDLE;
      }

    // 3) TURN_180: maniobra que gira 180° y a continuación lanza un escaneo
    } else if (currentMode == TURN_180) {
      if (!moving) {
        // inicializamos la rutina de giro a 180° (calcular pasos y marcar moving)
        Serial.println("🔄 Turning 180°...");
        targetSteps = computeSteps(180);
        currentSteps = 0;
        moving = true;
      }
      // ejecutar micro-pasos con un pequeño intervalo (lastStepTime)
      if (moving && (currentMillis - lastStepTime >= 2)) {
        stepMotors('R');
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          // giro completado: preparar scan rotacional
          moving = false;
          currentScanIndex = 0;
          currentMode = SCAN;
          Serial.println("✅ Turn 180° done, starting scan.");
        }
      }

    // 4) SCAN: barrido del sensor montado en servo para medir distancias
    } else if (currentMode == SCAN) {
      if (currentScanIndex < NUM_SCAN_ANGLES) {
        // Mover el servo a la siguiente posición (non-blocking)
        if (servoMoveStart == 0) {
          int angle = SCAN_ANGLES[currentScanIndex];
          sensorServo.write(angle);
          servoMoveStart = currentMillis;
        }
        // Esperar ~250 ms para que el servo quede estable y tomar lectura
        if (currentMillis - servoMoveStart >= 250) {
          scanDistances[currentScanIndex] = getDistanceFromSensor();
          Serial.print("Angle ");
          Serial.print(SCAN_ANGLES[currentScanIndex]);
          Serial.print("° -> ");
          Serial.print(scanDistances[currentScanIndex]);
          Serial.println(" cm");
          currentScanIndex++;
          servoMoveStart = 0;
        }
      } else {
        // Barrido completado: buscar el ángulo con la distancia mínima
        float minDist = scanDistances[0];
        int minIdx = 0;
        for (int i = 1; i < NUM_SCAN_ANGLES; i++) {
          if (scanDistances[i] < minDist) {
            minDist = scanDistances[i];
            minIdx = i;
          }
        }
        // Guardar ángulo del muro más cercano y centrar el sensor
        targetAngle = SCAN_ANGLES[minIdx];
        Serial.print("Closest wall at ");
        Serial.print(targetAngle);
        Serial.print("° (");
        Serial.print(minDist);
        Serial.println(" cm)");
        sensorServo.write(90); // volver servo al centro
        currentMode = TURN_TO_TARGET; // alinear robot hacia el objetivo
      }

    // 5) TURN_TO_TARGET: girar hacia el ángulo detectado desde el servo
    } else if (currentMode == TURN_TO_TARGET) {
      if (!moving) {
        // Calcular cuánto hay que girar respecto al frente del robot (90° servo)
        float diff = fabs(targetAngle - 90);
        if (diff < 3) {
          // ya alineado
          Serial.println("Already facing target.");
          currentMode = DRIVE_FORWARD;
        } else {
          // preparar giro por pasos
          char dir = (targetAngle < 90) ? 'R' : 'L';
          targetSteps = computeSteps(diff);
          currentSteps = 0;
          moving = true;
          Serial.print("Turning ");
          Serial.print((dir == 'R') ? "right " : "left ");
          Serial.print(diff);
          Serial.println("°");
        }
      } else if (currentMillis - lastStepTime >= 2) {
        // ejecutar micro-pasos hasta completar targetSteps
        char dir = (targetAngle < 90) ? 'R' : 'L';
        stepMotors(dir);
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          moving = false;
          currentMode = DRIVE_FORWARD;
          Serial.println("✅ Aligned to target.");
        }
      }

    // 6) DRIVE_FORWARD: avanzar hacia la pared/objetivo monitorizando distancia
    } else if (currentMode == DRIVE_FORWARD) {
      // avanzar en micro-pasos de forma rítmica
      if (currentMillis - lastStepTime >= 2) {
        stepMotors('F');
        lastStepTime = currentMillis;
      }
      // tomar lecturas periódicas del sensor para detectar la pared
      if (currentMillis - lastActionTime >= 100) {
        lastDistance = getDistanceFromSensor();
        Serial.print("Distance: ");
        Serial.println(lastDistance);
        lastActionTime = currentMillis;
      }
      // si estamos demasiado cerca, paramos y preparamos el giro de vuelta
      if (lastDistance <= SAFE_DISTANCE && lastDistance > 0) {
        Serial.println("🧱 Close to wall. Stopping and preparing to turn back.");
        currentMode = TURN_BACK;
        moving = false;
      }

    // 7) TURN_BACK: girar 180° para volver a una orientación segura y pasar a IDLE
    } else if (currentMode == TURN_BACK) {
      if (!moving) {
        Serial.println("🔁 Turning back 180°...");
        targetSteps = computeSteps(180);
        currentSteps = 0;
        moving = true;
      }
      if (moving && (currentMillis - lastStepTime >= 2)) {
        stepMotors('R');
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          moving = false;
          currentMode = IDLE;
          lastActionTime = currentMillis;
          Serial.println("✅ Turn back done. Waiting...");
        }
      }
    }
  }
}