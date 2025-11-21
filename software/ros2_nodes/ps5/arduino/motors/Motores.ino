#include <Arduino.h>

// ===== CONFIGURACIÓN DE MOTORES =====
#define MOTOR_STEPS 200      // pasos por revolución
#define MICROSTEPS 16        // microstepping
#define STEP_DELAY 700       // microsegundos entre pasos

// Pines de los motores
#define DIR1 8
#define STEP1 9

#define DIR2 2
#define STEP2 3

#define DIR3 5
#define STEP3 4

#define ENABLE 7

// ===== VARIABLES =====
long posMotor1 = 0;
long posMotor2 = 0;
long posMotor3 = 0;

bool movimientoRealizado = false;

// ===== CONSTANTES =====
const float INIT_ANGLE = 75.0; // grados deseados al inicio

// Calcular pasos equivalentes (considerando microstepping)
const long INIT_STEPS = (long)((INIT_ANGLE / 360.0) * MOTOR_STEPS * MICROSTEPS);

// ===== PROTOTIPOS =====
void moverMotoresSimultaneamente(bool sentidoHorario, long pasos);
void mostrarPosiciones();

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // Activar drivers (LOW = habilitado)
  digitalWrite(ENABLE, LOW);

  Serial.println("Inicializando motores a la posición de 550°...");
}

// ===== LOOP =====
void loop() {
  if (!movimientoRealizado) {
    moverMotoresSimultaneamente(true, INIT_STEPS);
    mostrarPosiciones();
    movimientoRealizado = true;  // Solo se ejecuta una vez
  }
}

/void moverMotores(long step1, long step2, long step3) {
  // Determinar sentido de cada motor
  digitalWrite(DIR1, step1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR2, step2 >= 0 ? HIGH : LOW);
  digitalWrite(DIR3, step3 >= 0 ? HIGH : LOW);

  // Convertir a valores absolutos
  long pasos1 = abs(step1);
  long pasos2 = abs(step2);
  long pasos3 = abs(step3);

  // Encontrar el mayor número de pasos para sincronizar los motores
  long maxPasos = max(pasos1, max(pasos2, pasos3));

  // Movimiento simultáneo aproximado
  for (long i = 0; i < maxPasos; i++) {
    if (i < pasos1) {
      digitalWrite(STEP1, HIGH);
    }
    if (i < pasos2) {
      digitalWrite(STEP2, HIGH);
    }
    if (i < pasos3) {
      digitalWrite(STEP3, HIGH);
    }

    delayMicroseconds(STEP_DELAY);

    digitalWrite(STEP1, LOW);
    digitalWrite(STEP2, LOW);
    digitalWrite(STEP3, LOW);

    delayMicroseconds(STEP_DELAY);
  }

  // Actualizar posiciones globales
  posMotor1 += step1;
  posMotor2 += step2;
  posMotor3 += step3;

  Serial.print("→ M1: "); Serial.print(posMotor1);
  Serial.print(" | M2: "); Serial.print(posMotor2);
  Serial.print(" | M3: "); Serial.println(posMotor3);
}


// Muestra la posición final de cada motor
void mostrarPosiciones() {
  float radianesPorPaso = (2.0 * PI) / (MOTOR_STEPS * MICROSTEPS);

  Serial.println("----- Posiciones finales (radianes) -----");
  Serial.print("Motor 1 -> Pasos: "); Serial.print(posMotor1);
  Serial.print(" | Radianes: "); Serial.println(posMotor1 * radianesPorPaso, 4);

  Serial.print("Motor 2 -> Pasos: "); Serial.print(posMotor2);
  Serial.print(" | Radianes: "); Serial.println(posMotor2 * radianesPorPaso, 4);

  Serial.print("Motor 3 -> Pasos: "); Serial.print(posMotor3);
  Serial.print(" | Radianes: "); Serial.println(posMotor3 * radianesPorPaso, 4);

  Serial.println("------------------------------------------");
}
