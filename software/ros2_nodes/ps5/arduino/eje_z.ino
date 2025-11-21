// =====================================================
// Movimiento 3 motores en el eje Z (simultáneo)
// =====================================================

#define STEP_DELAY 700
#define DIR1 8
#define STEP1 9
#define DIR2 3
#define STEP2 2
#define DIR3 4
#define STEP3 5
#define ENABLE 7

long pos1 = 0, pos2 = 0, pos3 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(DIR1, OUTPUT); pinMode(STEP1, OUTPUT);
  pinMode(DIR2, OUTPUT); pinMode(STEP2, OUTPUT);
  pinMode(DIR3, OUTPUT); pinMode(STEP3, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  Serial.println("Movimiento en eje Z listo");
}

void loop() {
  long posiciones[] = {0, 250, 500, 250, 0};  // posiciones en eje Z
  int num = sizeof(posiciones) / sizeof(posiciones[0]);
  int espera = 1000;

  for (int i = 0; i < num; i++) {
    long objetivo = posiciones[i];
    moverMotoresEje(DIR1, STEP1, pos1, objetivo);
    moverMotoresEje(DIR2, STEP2, pos2, objetivo);
    moverMotoresEje(DIR3, STEP3, pos3, objetivo);

    pos1 = objetivo; pos2 = objetivo; pos3 = objetivo;
    delay(espera);
  }
}

void moverMotoresEje(int pinDIR, int pinSTEP, long actual, long objetivo) {
  if (objetivo > actual) digitalWrite(pinDIR, HIGH);
  else digitalWrite(pinDIR, LOW);

  int pasos = abs(objetivo - actual);
  for (int i = 0; i < pasos; i++) {
    digitalWrite(pinSTEP, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(pinSTEP, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}
