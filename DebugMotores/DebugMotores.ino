// ============================================================
// DEBUG MOTORES - Seguidor de Línea ESP32-C3 Super Mini
// Secuencia para identificar y validar los motores:
//   1) M_IZQ: ADELANTE
//   2) M_IZQ: ATRAS
//   3) M_DER: ADELANTE
//   4) M_DER: ATRAS
// Si el sentido no coincide con lo esperado, intercambiar los
// pines A y B del motor correspondiente (o sus cables).
// ============================================================

// Pines de los motores (ESP32-C3 Super Mini)
// M_IZQ = motor 1 (pines A=9, B=10)
// M_DER = motor 2 (pines A=21, B=20)
#define pinMizqA 9
#define pinMizqB 10
#define pinMderA 21
#define pinMderB 20

// Velocidad de prueba (%)
int velPrueba = 60;

void setMotors(float mIzq, float mDer) {
  // Limitar velocidades a ±100
  if (abs(mIzq) > 100) mIzq = 100 * (mIzq / abs(mIzq));
  if (abs(mDer) > 100) mDer = 100 * (mDer / abs(mDer));

  // Convertir a rango PWM (0-255)
  mIzq = mIzq * 255 / 100;
  mDer = mDer * 255 / 100;

  // Motor IZQ
  if (mIzq >= 0) {
    analogWrite(pinMizqA, mIzq);
    analogWrite(pinMizqB, 0);
  } else {
    analogWrite(pinMizqA, 0);
    analogWrite(pinMizqB, abs(mIzq));
  }

  // Motor DER
  if (mDer >= 0) {
    analogWrite(pinMderA, mDer);
    analogWrite(pinMderB, 0);
  } else {
    analogWrite(pinMderA, 0);
    analogWrite(pinMderB, abs(mDer));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pinMizqA, OUTPUT);
  pinMode(pinMizqB, OUTPUT);
  pinMode(pinMderA, OUTPUT);
  pinMode(pinMderB, OUTPUT);
  setMotors(0, 0);

  Serial.println("===== DEBUG MOTORES =====");
  Serial.println("Secuencia:");
  Serial.println("1) M_IZQ (9/10)  ADELANTE");
  Serial.println("2) M_IZQ (9/10)  ATRAS");
  Serial.println("3) M_DER (21/20) ADELANTE");
  Serial.println("4) M_DER (21/20) ATRAS");
  Serial.println("Si el sentido no coincide, intercambiar los pines A/B.");
}

void loop() {
  Serial.println("1) M_IZQ ADELANTE");
  setMotors(velPrueba, 0);
  delay(1000);
  setMotors(0, 0);
  delay(500);

  Serial.println("2) M_IZQ ATRAS");
  setMotors(-velPrueba, 0);
  delay(1000);
  setMotors(0, 0);
  delay(500);

  Serial.println("3) M_DER ADELANTE");
  setMotors(0, velPrueba);
  delay(1000);
  setMotors(0, 0);
  delay(500);

  Serial.println("4) M_DER ATRAS");
  setMotors(0, -velPrueba);
  delay(1000);
  setMotors(0, 0);
  delay(500);
}
