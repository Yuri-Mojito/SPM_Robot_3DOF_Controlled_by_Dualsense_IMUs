#include <Arduino.h>
// ===== CONFIGURACIÓN DE MOTORES =====
// ===== CONFIGURACIÓN DE MOTORES =====
#define MOTOR_STEPS 200
#define MICROSTEPS 16
#define STEP_DELAY 100  // microsegundos entre pulsos

// Pines de los motores
#define DIR1 8
#define STEP1 9
#define DIR2 2
#define STEP2 3
#define DIR3 5
#define STEP3 4
#define ENABLE 7

// ===== LÍMITES FÍSICOS (ajusta según tu mecánica) =====
const long LIMITE_MIN = 0;
const long LIMITE_MAX = 1400;

// ===== VARIABLES =====
long posMotor1 = 0;
long posMotor2 = 0;
long posMotor3 = 0;

// ===== PROTOTIPOS =====
void moverATargets(long target1, long target2, long target3);

void setup() {
  Serial.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  digitalWrite(ENABLE, LOW); // Activar drivers
  Serial.println("Arduino listo. Esperando targets absolutos tipo: target1,target2,target3");
}

void loop() {
  if (!Serial.available()) {
  digitalWrite(LED_BUILTIN, LOW);
} else {
  digitalWrite(LED_BUILTIN, HIGH);
}
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() == 0) return;

    int sep1 = data.indexOf(',');
    int sep2 = data.indexOf(',', sep1 + 1);

    if (sep1 >= 0 && sep2 > sep1) {
      long target1 = data.substring(0, sep1).toInt();
      long target2 = data.substring(sep1 + 1, sep2).toInt();
      long target3 = data.substring(sep2 + 1).toInt();

      // Limitar targets dentro de los topes físicos
      target1 = constrain(target1, LIMITE_MIN, LIMITE_MAX);
      target2 = constrain(target2, LIMITE_MIN, LIMITE_MAX);
      target3 = constrain(target3, LIMITE_MIN, LIMITE_MAX);

      moverATargets(target1, target2, target3);

      // Reporte
      Serial.print("→ M1: "); Serial.print(posMotor1);
      Serial.print(" | M2: "); Serial.print(posMotor2);
      Serial.print(" | M3: "); Serial.println(posMotor3);
    } else {
      Serial.print("Formato inválido: ");
      Serial.println(data);
    }
  }
}

void moverATargets(long target1, long target2, long target3) {
  // Calcular deltas relativos que realmente hay que mover
  long delta1 = target1 - posMotor1;
  long delta2 = target2 - posMotor2;
  long delta3 = target3 - posMotor3;

  // Si no hay movimiento para los 3, salir
  if (delta1 == 0 && delta2 == 0 && delta3 == 0) {
    Serial.println("No hay movimiento necesario (targets ya alcanzados).");
    return;
  }

  // Direcciones
  digitalWrite(DIR1, delta1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR2, delta2 >= 0 ? HIGH : LOW);
  digitalWrite(DIR3, delta3 >= 0 ? HIGH : LOW);

  long pasos1 = abs(delta1);
  long pasos2 = abs(delta2);
  long pasos3 = abs(delta3);
  long maxPasos = max(pasos1, max(pasos2, pasos3));

  // Realizar movimiento sincronizado aproximado
  for (long i = 0; i < maxPasos; i++) {
    if (i < pasos1) digitalWrite(STEP1, HIGH);
    if (i < pasos2) digitalWrite(STEP2, HIGH);
    if (i < pasos3) digitalWrite(STEP3, HIGH);

    delayMicroseconds(STEP_DELAY);

    if (i < pasos1) digitalWrite(STEP1, LOW);
    if (i < pasos2) digitalWrite(STEP2, LOW);
    if (i < pasos3) digitalWrite(STEP3, LOW);

    delayMicroseconds(STEP_DELAY);
  }

  // Actualizar posiciones a las targets (ya estuvieron limitadas)
  posMotor1 = target1;
  posMotor2 = target2;
  posMotor3 = target3;
}
