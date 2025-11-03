// === Librerías necesarias ===
#include <DMotor_mod.h>
#include <Servo_ESP8266.h>

// === Pines ===
#define ledPin D0
#define servoPin D7
#define sensorPin A0

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
#define TEST_MODE 0

// TEST_MODE funciona en modo auto-run 
const unsigned long testAutoInterval = 10000; // ms entre comandos
bool testAutoRun = true; // si true avanza automáticamente cada testAutoInterval

// SERIAL STUB: leer comandos desde el Monitor Serie cuando TEST_MODE == 0
// Formato: "<dist_cm> <angle_deg>" por línea, p.e. "30 90" o "D:30,A:90"
#define SERIAL_STUB 1

// Lista de comandos de prueba: {distancia_cm, angulo_deg}
const int TEST_CMD_COUNT = 6;
float testCommands[TEST_CMD_COUNT][2] = {
  {30.0, 0.0},   // avanzar 30cm, heading 0°
  {20.0, 90.0},  // avanzar 20cm, heading 90°
  {15.0, 180.0}, // avanzar 15cm, heading 180°
  {10.0, 270.0}, // avanzar 10cm, heading 270°
  {5.0, 45.0},   // avanzar 5cm, heading 45°
  {0.0, 0.0}     // giro a 0°, sin avance
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

// Resolución del stepper (proporcionada)
int stepperResolution = 256;  // 8 bits, corresponde al primer parámetro de AF_Stepper

// Pasos por revolución (incluyendo microsteps)
long stepsPerRevMicro() { return (long)stepperResolution * (long)stepToMicrostep; }

// Máquina de estados para movimiento (no bloqueante)
enum RobotMode { IDLE, ROTATING, MOVING };
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
// === Uli ===
// --- Helper functions ---
float getSensorDistance(float mV) {
  if (mV > mVinterval * tableEntries - 1) return sensorDistanceTable[tableEntries - 1];
  int index = mV / mVinterval;
  float frac = fmod(mV, (float)mVinterval) / (float)mVinterval;
  return sensorDistanceTable[index] - ((sensorDistanceTable[index] - sensorDistanceTable[index + 1]) * frac);
}

float getDistanceFromSensor() {
  float referenceMv = 1000.0;
  float val = analogRead(sensorPin);
  float mV = (val * referenceMv) / 1023.0;
  return getSensorDistance(mV);
}

long computeSteps(float angle) {
  return angle * stepperResolution * stepToMicrostep * wheelsAxisDis / (wheelRadius * 720);
}

void stepMotors(char dir) {
  if (dir == 'F') { motorR.step(1, FORWARD, SINGLE); motorL.step(1, FORWARD, SINGLE); }
  if (dir == 'B') { motorR.step(1, BACKWARD, SINGLE); motorL.step(1, BACKWARD, SINGLE); }
  if (dir == 'R') { motorR.step(1, BACKWARD, SINGLE); motorL.step(1, FORWARD, SINGLE); }
  if (dir == 'L') { motorR.step(1, FORWARD, SINGLE);  motorL.step(1, BACKWARD, SINGLE); }
}

// === Raul ===
// === Función para convertir lectura a distancia ===
float getsensorDistance(int mV) {
   if (mV >= mVinterval * (tableEntries - 1)) {
      return sensorDistance[tableEntries - 1];
   } else {
      int index = mV / mVinterval;
      index = constrain(index, 0, tableEntries - 2);
      float frac = (mV % mVinterval) / (float)mVinterval;
      return sensorDistance[index] - ((sensorDistance[index] - sensorDistance[index + 1]) * frac);
   }
}

// === Función para leer distancia ===
float sensorDistanceRead() { 
  int val = analogRead(sensorPin);
  float mV = (val * 1000.0) / 1023.0;
  float cm = getsensorDistance((int)mV);
  return cm;
}

// Función auxiliar para mapeo de floats, sacado de foros
long mapFloat(float x, float in_min, float in_max, long out_min, long out_max) {
  return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Convierte grados de giro a número de micro-pasos por rueda (rotación in-place)
long degreesToSteps(float angleDeg) {
  float angle = fabs(angleDeg);
  // Longitud del arco que recorre cada rueda (mm)
  float arc_mm = (PI * WHEEL_BASE_MM) * (angle / 360.0);
  // vueltas de rueda necesarias
  float rotations = arc_mm / (PI * WHEEL_DIAMETER_MM);
  float steps = rotations * (float)stepsPerRevMicro();
  return (long)(steps + 0.5);
}

// Convierte distancia en cm a pasos por rueda (avance linear)
long distanceCmToSteps(float distCm) {
  float dist_mm = distCm * 10.0;
  float rotations = dist_mm / (PI * WHEEL_DIAMETER_MM);
  float steps = rotations * (float)stepsPerRevMicro();
  return (long)(steps + 0.5);
}

// Normaliza ángulo a [-180,180]
float shortestAngle(float fromDeg, float toDeg) {
  float d = toDeg - fromDeg;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// ------------------------------------------------------------------
// comandos remotos
// Aquí  estará la funcion de lectura de comandos remotos
// ------------------------------------------------------------------
void processRemoteCommand(float distCm, float angleDeg) {
  // Guardar pedido de distancia
  desiredDistanceCm = distCm;

  // El ángulo recibido es directamente cuánto girar
  float dAng = angleDeg;
  // Actualizar el heading final
  desiredHeading = fmod(currentHeading + dAng, 360.0);
  if (desiredHeading < 0) desiredHeading += 360.0;
  long rotSteps = degreesToSteps(dAng);

  if (rotSteps > 0) {
    // Preparar rotación in-place: ruedas en sentidos opuestos
    if (dAng > 0) { dirR = BACKWARD; dirL = FORWARD; }
    else { dirR = FORWARD; dirL = BACKWARD; }
    targetStepsR = rotSteps; targetStepsL = rotSteps;
    doneStepsR = 0; doneStepsL = 0;
    currentMode = ROTATING;
    Serial.print("[REMOTE] Preparado ROTATE pasos="); Serial.println(rotSteps);
  } else {
    // Ya orientado: preparar avance
    long moveSteps = distanceCmToSteps(desiredDistanceCm);
    if (moveSteps > 0) {
      dirR = FORWARD; dirL = FORWARD;
      targetStepsR = moveSteps; targetStepsL = moveSteps;
      doneStepsR = 0; doneStepsL = 0;
      currentMode = MOVING;
      Serial.print("[REMOTE] Preparado MOVE pasos="); Serial.println(moveSteps);
    } else {
      currentMode = IDLE;
    }
  }
}

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

  Serial.println("=== LLUBot Optimizado ===");
  Serial.println("Esperando lecturas del sensor...");
}

void loop() {
  unsigned long currentMillis = millis();

  // ----------------------
  // Punto de integración remoto
  // ----------------------
  // float rxDist, rxAng;
  // if ( getRemoteCommand(rxDist, rxAng) ) { // devuelve true si hay comando nuevo || bool getRemoteCommand(float &outDist, float &outAng)
  //   processRemoteCommand(rxDist, rxAng);
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
        float vals[2]; int cnt = 0;
        int i = 0;
        while (i < line.length() && cnt < 2) {
          while (i < line.length() && isSpace(line[i])) i++;
          if (i >= line.length()) break;
          int j = i;
          while (j < line.length() && !isSpace(line[j])) j++;
          String tok = line.substring(i, j);
          char buf[32]; tok.toCharArray(buf, sizeof(buf));
          vals[cnt++] = atof(buf);
          i = j;
        }
        if (cnt == 2) {
          float rxDist = vals[0];
          float rxAng = vals[1];
          Serial.print("[STUB] Recibido: "); Serial.print(rxDist); Serial.print(" cm, "); Serial.print(rxAng); Serial.println(" deg");
          processRemoteCommand(rxDist, rxAng);
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
    previousMillisTest = currentMillis;
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
    }

    // avanzar índice de test
    testIndex++;
    if (testIndex >= TEST_CMD_COUNT) testIndex = 0;
  }
#endif

  // Lectura del sensor cada 50ms
#if !TEST_MODE && !SERIAL_STUB
  if (currentMillis - previousMillisSensor >= intervalSensor) {
    previousMillisSensor = currentMillis;
    
    float distance = sensorDistanceRead();
    Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.print(" cm - ");

    // LÓGICA DE VELOCIDAD: Más cerca = más rápido
    if (distance < 15.0) {
      // MUY CERCA - MÁXIMA VELOCIDAD (delay mínimo)
      motorDelay = 1;
      digitalWrite(ledPin, HIGH); // Alerta - OBJETO CERCA
      Serial.print("ALTA velocidad");
    } 
    else if (distance < 25.0) {
      // CERCA - velocidad media
      motorDelay = mapFloat(distance, 15.0, 25.0, 1, 15);
      digitalWrite(ledPin, LOW);
      Serial.print("MEDIA velocidad");
    } 
    else if (distance < 40.0) {
      // MEDIA DISTANCIA - velocidad baja
      motorDelay = mapFloat(distance, 25.0, 40.0, 15, 40);
      digitalWrite(ledPin, LOW);
      Serial.print("BAJA velocidad");
    } 
    else {
      // LEJOS - velocidad mínima
      motorDelay = 50;
      digitalWrite(ledPin, LOW);
      Serial.print("VELOCIDAD mínima");
    }

    // Asegurar que no tenemos valores extremos
    motorDelay = constrain(motorDelay, 1, 100);
    
    Serial.print(" - Delay: ");
    Serial.println(motorDelay);
  }
#else
  // TEST_MODE activo: ignorar sensor y no modificar motorDelay
#endif

  // Máquina de stepping no bloqueante: realiza 1 micro-paso por rueda cada motorDelay ms
  if (currentMillis - previousMillisMotors >= motorDelay) {
    previousMillisMotors = currentMillis;

    if (currentMode == ROTATING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) {
        // actualizar heading aproximado y pasar a MOVING o IDLE
        currentHeading = desiredHeading;
        Serial.print("[TEST] Giro completado. Heading ahora: "); Serial.println(currentHeading);
        long moveSteps = distanceCmToSteps(desiredDistanceCm);
        if (moveSteps > 0) { dirR = FORWARD; dirL = FORWARD; targetStepsR = moveSteps; targetStepsL = moveSteps; doneStepsR = 0; doneStepsL = 0; currentMode = MOVING; Serial.print("[TEST] Iniciando avance (pasos): "); Serial.println(moveSteps); }
        else { currentMode = IDLE; }
      }
    } else if (currentMode == MOVING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) { Serial.println("[TEST] Movimiento completado."); currentMode = IDLE; }
    } else {
      // IDLE: no steps
    }
  }
}