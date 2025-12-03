

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// --- CALIBRACIÓN DE PCA ---
#define SERVO_MIN 150
#define SERVO_MAX 600

// --- MAPA DE MOTORES ---
#define CH_BASE    0 // Pin 0: Eje X
#define CH_HOMBRO  1 // Pin 1: Eje Z (Distancia)
#define CH_CODO    2 // Pin 2: Eje Y (Altura)
#define CH_MUNECA  3 // Pin 3: Muñeca (Eje horizontal)
#define CH_AUX     4 // Pin 4: Auxiliar
#define CH_GRIPPER 5 // Pin 5: Pinza (Cerrar/Abrir)

// --- POSICIÓN HOME (LISTO PARA COSECHAR) ---
int angBase_HOME   = 80;
int angHombro_HOME = 135;  // HOME hombro
int angCodo_HOME   = 125;  // HOME codo
int angMuneca_HOME = 90;
int angAux_HOME    = 90;

// PINZA
// Gancho en HOME: 130° (abierto en modo trabajo)
// Gancho para agarrar fresa: 84°
// Gancho en REPOSO: 90°
int angPinza_OPEN   = 150; // “abierto” (HOME y cuando suelta la fresa)
int angPinza_CLOSED = 84;  // “cerrado” (agarrando fresa)

// --- POSICIÓN DE REPOSO ---
int angBase_REPOSO    = 0;    // Pin 0  
int angHombro_REPOSO  = 180;  // Pin 1
int angCodo_REPOSO    = 160;  // Pin 2
int angMuneca_REPOSO  = 90;   // Pin 3
int angAux_REPOSO     = 90;   // Pin 4
int angGripper_REPOSO = 90;   // Pin 5 (gancho en 90° en reposo)

// --- POSICIÓN GUARDAR FRUTA (NUEVA) ---
int angBase_DROP   = 180; // Pin 0
int angHombro_DROP = 85;  // Pin 1
int angCodo_DROP   = 120; // Pin 2
int angMuneca_DROP = 90;  // asumimos igual
int angAux_DROP    = 90;  // asumimos igual

// --- VARIABLES DE ESTADO ---
int angBase    = angBase_REPOSO;
int angHombro  = angHombro_REPOSO;
int angCodo    = angCodo_REPOSO;
int angMuneca  = angMuneca_REPOSO;
int angAux     = angAux_REPOSO;
int angGripper = angGripper_REPOSO;

int step = 1;

// ------------------------------------
// FUNCIONES DE CONTROL
// ------------------------------------

// Mueve todos los servos a sus ángulos actuales
void moverServos() {
  pca.setPWM(CH_BASE,    0, map(angBase,    0, 180, SERVO_MIN, SERVO_MAX));
  pca.setPWM(CH_HOMBRO,  0, map(angHombro,  0, 180, SERVO_MIN, SERVO_MAX));
  pca.setPWM(CH_CODO,    0, map(angCodo,    0, 180, SERVO_MIN, SERVO_MAX));
  pca.setPWM(CH_MUNECA,  0, map(angMuneca,  0, 180, SERVO_MIN, SERVO_MAX));
  pca.setPWM(CH_AUX,     0, map(angAux,     0, 180, SERVO_MIN, SERVO_MAX));
  pca.setPWM(CH_GRIPPER, 0, map(angGripper, 0, 180, SERVO_MIN, SERVO_MAX));
}

// Mueve la pinza de forma lenta y controlada
void moveGripperGradual(int targetAngle, int delayMs) {
  while (angGripper != targetAngle) {
    if (angGripper < targetAngle) {
      angGripper++;
    } else {
      angGripper--;
    }

    pca.setPWM(CH_GRIPPER, 0, map(angGripper, 0, 180, SERVO_MIN, SERVO_MAX));
    delay(delayMs);
  }
}

// Mueve TODOS los servos hacia una pose objetivo, paso a paso
void moveToPoseSlow(
  int targetBase,
  int targetHombro,
  int targetCodo,
  int targetMuneca,
  int targetAux,
  int targetGripper,
  int delayMs
) {
  bool done;

  do {
    done = true;

    // BASE
    if (angBase < targetBase)       { angBase++;   done = false; }
    else if (angBase > targetBase)  { angBase--;   done = false; }

    // HOMBRO
    if (angHombro < targetHombro)       { angHombro++;   done = false; }
    else if (angHombro > targetHombro)  { angHombro--;   done = false; }

    // CODO
    if (angCodo < targetCodo)       { angCodo++;   done = false; }
    else if (angCodo > targetCodo)  { angCodo--;   done = false; }

    // MUÑECA
    if (angMuneca < targetMuneca)       { angMuneca++;   done = false; }
    else if (angMuneca > targetMuneca)  { angMuneca--;   done = false; }

    // AUX
    if (angAux < targetAux)       { angAux++;   done = false; }
    else if (angAux > targetAux)  { angAux--;   done = false; }

    // PINZA
    if (angGripper < targetGripper)       { angGripper++;   done = false; }
    else if (angGripper > targetGripper)  { angGripper--;   done = false; }

    // Límites de seguridad
    angBase   = constrain(angBase,   0, 180);
    angHombro = constrain(angHombro, 50, 180);
    angCodo   = constrain(angCodo,   50, 160);

    moverServos();
    delay(delayMs);

  } while (!done);
}

// --- HOME LENTO ---
// HOME = pose de trabajo, gancho en 130°
void goHome() {
  moveToPoseSlow(
    angBase_HOME,
    angHombro_HOME,
    angCodo_HOME,
    angMuneca_HOME,
    angAux_HOME,
    angPinza_OPEN,   // 130°
    25
  );
}

// --- REPOSO LENTO ---
// REPOSO = pose de descanso, gancho en 90°
void goReposoSlow() {
  moveToPoseSlow(
    angBase_REPOSO,
    angHombro_REPOSO,
    angCodo_REPOSO,
    angMuneca_REPOSO,
    angAux_REPOSO,
    angGripper_REPOSO, // 90°
    25
  );
}

// --- GUARDAR FRUTA (NUEVA POSICIÓN) ---
// Si mantenerPinzaCerrada = true → se queda en 84° todo el camino
void goDropPoseSlow(bool mantenerPinzaCerrada) {
  int targetGripper = mantenerPinzaCerrada ? angPinza_CLOSED : angPinza_OPEN;

  moveToPoseSlow(
    angBase_DROP,
    angHombro_DROP,
    angCodo_DROP,
    angMuneca_DROP,
    angAux_DROP,
    targetGripper,
    25
  );
}

// ------------------------------------
// SETUP
// ------------------------------------
void setup() {
  Serial.begin(115200);
  pca.begin();
  pca.setPWMFreq(60);

  moverServos();
  delay(500);
}

// ------------------------------------
// LOOP
// ------------------------------------
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // 1. COMANDO HOME (H)
    if (cmd == 'H') {
      goHome();
    }

    // 2. COMANDO GRAB (G) - AGARRAR → ESPERAR → GUARDAR → SOLTAR → HOME → REPOSO
    else if (cmd == 'G') {
      // a) CERRAR PINZA (a 84°) LENTO → AGARRAR FRESA
      moveGripperGradual(angPinza_CLOSED, 20);

      // Mantenerla agarrando 2 segundos antes de moverse
      delay(2000);

      // b) IR A POSICIÓN DE GUARDAR FRUTA, con pinza cerrada (84° todo el trayecto)
      goDropPoseSlow(true);
      delay(500);

      // c) SOLTAR FRESA (abrir gancho a 130°)
      moveGripperGradual(angPinza_OPEN, 20);
      delay(500);

      // d) VOLVER A HOME LENTO (con gancho en 130°)
      goHome();
      delay(500);

      // e) IR A REPOSO LENTO (con gancho en 90°)
      goReposoSlow();
      delay(500);
    }

    // 3. COMANDO REPOSO FINAL (P) - Ir a REPOSO de manera lenta (botón de emergencia)
    else if (cmd == 'P') {
      goReposoSlow();
    }

    // 4. COMANDOS DE SEGUIMIENTO (R, L, U, D, F, B)
    else if (cmd == 'R' || cmd == 'L' || cmd == 'U' || cmd == 'D' || cmd == 'F' || cmd == 'B') {

      if (cmd == 'L')      angBase += step;
      else if (cmd == 'R') angBase -= step;

      else if (cmd == 'F') angHombro -= step;
      else if (cmd == 'B') angHombro += step;

      else if (cmd == 'U') angCodo -= step;
      else if (cmd == 'D') angCodo += step;

      angBase   = constrain(angBase,   0, 180);
      angHombro = constrain(angHombro, 50, 180);
      angCodo   = constrain(angCodo,   50, 160);

      moverServos();
    }

    delay(10);
  }
}
