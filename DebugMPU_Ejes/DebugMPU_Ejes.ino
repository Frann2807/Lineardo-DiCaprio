// ============================================================
// DEBUG MPU_EJES - Identificar el eje de la rampa
// ESP32-C3 Super Mini - I2C crudo (MPU6050)
//
// Muestra los datos crudos y TODOS los ángulos candidatos para
// identificar cuál es el que cambia al inclinar el robot en la
// rampa. Pasos:
//
//   1) Con el robot en LLANO, mirar la línea "Acel": el eje que
//      lee ~1.00g es el vertical del MPU.
//   2) Mirar la línea "Giro": el eje que se mueve al GIRAR el
//      robot es el eje de giro.
//   3) Inclinar el robot como en la rampa y ver qué ángulo
//      cambia:
//        AngX = atan2(az, |ay|) -> rotación sobre el eje X
//        AngZ = atan2(ax, |ay|) -> rotación sobre el eje Z
//        PitchY (plano) = atan2(-ax, raiz(ay^2+az^2))
//   Ese es el que hay que usar para la corrección.
//
// Comandos Serial:
//   'c' -> recalibrar bias del giroscopio (robot QUIETO)
// ============================================================

#include <Wire.h>
#include <math.h>

// ===== PINES I2C (ESP32-C3 Super Mini) =====
#define PIN_SDA 8
#define PIN_SCL 7

// ===== PINES DE MOTORES (se dejan en 0 en este debug) =====
#define pinMizqA 9
#define pinMizqB 10
#define pinMderA 21
#define pinMderB 20

// ===== DIRECCIÓN Y REGISTROS MPU6050 =====
#define MPU_ADDR          0x68
#define MPU_PWR_MGMT_1    0x6B
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_GYRO_XOUT_H   0x43
#define MPU_WHO_AM_I      0x75

// ===== FACTORES DE CONVERSIÓN =====
const float GYRO_LSB = 32.8f;     // ±1000 °/s -> 32.8 LSB por °/s
const float ACCEL_LSB = 8192.0f;  // ±4 g -> 8192 LSB por g
const float RAD2DEG = 57.2957795f;

// ===== BIAS DEL GIROSCOPIO =====
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ===== LECTURAS =====
float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

// ============================================================
// == FUNCIONES I2C DE BAJO NIVEL ==
// ============================================================

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

int16_t mpuRead16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

// ============================================================
// == INICIALIZACIÓN DEL MPU6050 ==
// ============================================================

bool mpuIniciar() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  delay(50);

  uint8_t who = mpuReadReg(MPU_WHO_AM_I);
  if (who == 0x00 || who == 0xFF) {
    Serial.printf("ERROR: MPU6050 no responde en 0x%02X (WHO_AM_I = 0x%02X).\n", MPU_ADDR, who);
    return false;
  }
  if (who != 0x68) {
    Serial.printf("AVISO: WHO_AM_I = 0x%02X (esperado 0x68).\n", who);
  }

  mpuWrite(MPU_PWR_MGMT_1, 0x80);
  delay(100);
  mpuWrite(MPU_PWR_MGMT_1, 0x01);
  mpuWrite(MPU_SMPLRT_DIV, 0x00);
  mpuWrite(MPU_CONFIG, 0x02);
  mpuWrite(MPU_GYRO_CONFIG, 0x10);
  mpuWrite(MPU_ACCEL_CONFIG, 0x08);
  delay(50);
  return true;
}

// ============================================================
// == CALIBRACIÓN DEL BIAS DEL GIROSCOPIO ==
// ============================================================

void mpuCalibrarBias() {
  const int N = 1000;

  Serial.println("Calibrando bias del giroscopio... ¡NO MOVER EL ROBOT!");

  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N; i++) {
    sx += mpuRead16(MPU_GYRO_XOUT_H);
    sy += mpuRead16(MPU_GYRO_XOUT_H + 2);
    sz += mpuRead16(MPU_GYRO_XOUT_H + 4);
    if (i % 200 == 199) Serial.print(".");
    delay(1);
  }

  gyroBiasX = (sx / (float)N) / GYRO_LSB;
  gyroBiasY = (sy / (float)N) / GYRO_LSB;
  gyroBiasZ = (sz / (float)N) / GYRO_LSB;

  Serial.println(" Listo!");
  Serial.printf("Bias giro (°/s): X = %.3f | Y = %.3f | Z = %.3f\n",
                gyroBiasX, gyroBiasY, gyroBiasZ);
}

// ============================================================
// == LECTURA DEL MPU6050 ==
// ============================================================

void mpuLeer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  int16_t axr = (Wire.read() << 8) | Wire.read();
  int16_t ayr = (Wire.read() << 8) | Wire.read();
  int16_t azr = (Wire.read() << 8) | Wire.read();
  int16_t tr  = (Wire.read() << 8) | Wire.read();
  int16_t gxr = (Wire.read() << 8) | Wire.read();
  int16_t gyr = (Wire.read() << 8) | Wire.read();
  int16_t gzr = (Wire.read() << 8) | Wire.read();

  ax_g = axr / ACCEL_LSB;
  ay_g = ayr / ACCEL_LSB;
  az_g = azr / ACCEL_LSB;

  gx_dps = (gxr / GYRO_LSB) - gyroBiasX;
  gy_dps = (gyr / GYRO_LSB) - gyroBiasY;
  gz_dps = (gzr / GYRO_LSB) - gyroBiasZ;
}

// ============================================================
// == SETUP ==
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motores en 0 para que no se activen
  pinMode(pinMizqA, OUTPUT);
  pinMode(pinMizqB, OUTPUT);
  pinMode(pinMderA, OUTPUT);
  pinMode(pinMderB, OUTPUT);
  digitalWrite(pinMizqA, LOW);
  digitalWrite(pinMizqB, LOW);
  digitalWrite(pinMderA, LOW);
  digitalWrite(pinMderB, LOW);

  Serial.println("===== DEBUG MPU_EJES (identificar eje de la rampa) =====");
  Serial.println("SDA = GPIO8 | SCL = GPIO7");

  if (!mpuIniciar()) {
    Serial.println("Revisar conexiones (VCC=3.3V, GND, SDA, SCL) y reiniciar.");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 inicializado OK");

  mpuCalibrarBias();

  Serial.println("------------------------------------------------------");
  Serial.println("1) En LLANO: el eje Acel con ~1.00g es el vertical.");
  Serial.println("2) GIRANDO el robot: ese eje de Giro es el eje de giro.");
  Serial.println("3) INCLINANDO como en la rampa: mirar qué ángulo cambia");
  Serial.println("   (AngX, AngZ o PitchY). Ese es el eje de la corrección.");
  Serial.println("------------------------------------------------------");
}

// ============================================================
// == LOOP ==
// ============================================================

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') mpuCalibrarBias();
  }

  mpuLeer();

  float angX = atan2(az_g, fabs(ay_g)) * RAD2DEG;
  float angZ = atan2(ax_g, fabs(ay_g)) * RAD2DEG;
  float pitchY = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD2DEG;
  float aMag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  Serial.printf("Acel: AX:%+1.3f AY:%+1.3f AZ:%+1.3f (|a|:%.2fg)\n",
                ax_g, ay_g, az_g, aMag);
  Serial.printf("Giro: GX:%+7.2f GY:%+7.2f GZ:%+7.2f °/s\n",
                gx_dps, gy_dps, gz_dps);
  Serial.printf("AngX(az):%+8.2f° AngZ(ax):%+8.2f° PitchY(plano):%+8.2f°\n",
                angX, angZ, pitchY);
  Serial.println("---");

  delay(100);
}
