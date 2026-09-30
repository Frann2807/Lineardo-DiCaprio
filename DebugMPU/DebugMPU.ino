// ============================================================
// DEBUG MPU6050 CON FILTRO DE KALMAN - ESP32-C3 Super Mini
// I2C CRUDO con Wire.h (sin librerías externas).
//
// Conexión:
//   MPU6050 SDA -> GPIO8
//   MPU6050 SCL -> GPIO7
//   VCC -> 3.3V | GND -> GND | AD0 -> GND (dirección 0x68)
//
// El Kalman (adaptado del código STM32) fusiona:
//   - Ángulo medido por el acelerómetro (inclinación de rampa)
//   - Velocidad angular del giroscopio del mismo eje
// y estima la inclinación sin drift, con el bias del giroscopio
// estimado ONLINE por el propio Kalman.
//
// Eje del Kalman:
//   MPU montado PLANO. La inclinación de la rampa es el PITCH sobre
//   el eje Y del MPU:
//     tilt = atan2(-ax, raiz(ay^2 + az^2))  -> usa giro Y
//
// Flags:
//   FlagRaw    = 1 -> muestra lecturas crudas (acel en g, giro en °/s)
//   FlagAngles = 1 -> muestra TiltK (Kalman), TiltAcc (acel), Rate y GiroY
//
// Calibración:
//   - Al arrancar se promedia el bias del giroscopio (robot QUIETO)
//     y la inclinación inicial se pone como cero.
//   - 'c' por Serial -> recalibrar bias (robot quieto)
//   - 'z' por Serial -> poner los ángulos actuales en 0
//
// NOTA: el eje de GIRO del robot (giro Y) no tiene referencia de
// gravedad, así que se muestra por integración pura (GiroY).
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
#define MPU_TEMP_OUT_H    0x41
#define MPU_GYRO_XOUT_H   0x43
#define MPU_WHO_AM_I      0x75

// ===== FACTORES DE CONVERSIÓN =====
const float GYRO_LSB = 32.8f;      // ±1000 °/s -> 32.8 LSB por °/s
const float ACCEL_LSB = 8192.0f;   // ±4 g -> 8192 LSB por g
const float RAD2DEG = 57.2957795f;

// ===== FLAGS =====
int FlagRaw = 0;      // 1 = mostrar lecturas crudas
int FlagAngles = 1;   // 1 = mostrar TiltK / TiltAcc / Rate / GiroY

// ===== BIAS DEL GIROSCOPIO (calibración inicial) =====
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ===== KALMAN (adaptado del código STM32) =====
struct Kalman_t {
  float angle;
  float bias;
  float rate;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_measure;
};

Kalman_t kalman;

void Kalman_Init(Kalman_t *k) {
  k->angle = 0.0f;
  k->bias = 0.0f;
  k->rate = 0.0f;

  k->P[0][0] = 1.0f;
  k->P[0][1] = 0.0f;
  k->P[1][0] = 0.0f;
  k->P[1][1] = 1.0f;

  k->Q_angle = 0.001f;
  k->Q_bias = 0.003f;
  k->R_measure = 0.03f;
}

float Kalman_GetAngle(Kalman_t *k, float newAngle, float newRate, float dt) {
  k->rate = newRate - k->bias;
  k->angle += dt * k->rate;

  k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += k->Q_bias * dt;

  float S = k->P[0][0] + k->R_measure;
  if (S == 0) {
    S = 0.0001f;
  }
  float K[2];
  K[0] = k->P[0][0] / S;
  K[1] = k->P[1][0] / S;

  float y = newAngle - k->angle;
  k->angle += K[0] * y;
  k->bias += K[1] * y;

  float P00_temp = k->P[0][0];
  float P01_temp = k->P[0][1];

  k->P[0][0] -= K[0] * P00_temp;
  k->P[0][1] -= K[0] * P01_temp;
  k->P[1][0] -= K[1] * P00_temp;
  k->P[1][1] -= K[1] * P01_temp;

  return k->angle;
}

// ===== ESTADO =====
float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;
float tempC;
float tiltAccel = 0;  // Ángulo medido por el acelerómetro
float rateUsada = 0;  // Giro del eje del tilt
float offTilt = 0;    // Cero de inclinación inicial
float giroY = 0;      // Ángulo de giro del robot (integración pura del giro Y)
uint32_t ultimoTiempoUs = 0;

String cmdSerial = "";

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
    Serial.printf("AVISO: WHO_AM_I = 0x%02X (esperado 0x68). Continuando de todas formas...\n", who);
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

  Serial.println("Calibrando bias del giroscopio...");
  Serial.println("¡NO MOVER EL ROBOT durante la calibracion!");

  long sx = 0, sy = 0, sz = 0;
  long sax = 0, say = 0, saz = 0;
  for (int i = 0; i < N; i++) {
    sx += mpuRead16(MPU_GYRO_XOUT_H);
    sy += mpuRead16(MPU_GYRO_XOUT_H + 2);
    sz += mpuRead16(MPU_GYRO_XOUT_H + 4);
    sax += mpuRead16(MPU_ACCEL_XOUT_H);
    say += mpuRead16(MPU_ACCEL_XOUT_H + 2);
    saz += mpuRead16(MPU_ACCEL_XOUT_H + 4);
    if (i % 200 == 199) Serial.print(".");
    delay(1);
  }

  gyroBiasX = (sx / (float)N) / GYRO_LSB;
  gyroBiasY = (sy / (float)N) / GYRO_LSB;
  gyroBiasZ = (sz / (float)N) / GYRO_LSB;

  // Cero de inclinación inicial (lo que mida el acelerómetro en reposo)
  float ax0 = (sax / (float)N) / ACCEL_LSB;
  float ay0 = (say / (float)N) / ACCEL_LSB;
  float az0 = (saz / (float)N) / ACCEL_LSB;
  offTilt = atan2(-ax0, sqrt(ay0 * ay0 + az0 * az0)) * RAD2DEG;

  Serial.println(" Listo!");
  Serial.printf("Bias giro (°/s): X = %.3f | Y = %.3f | Z = %.3f\n",
                gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.printf("Inclinacion inicial: %.2f° -> puesta como cero\n", offTilt);
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

  tempC = tr / 340.0f + 36.53f;

  gx_dps = (gxr / GYRO_LSB) - gyroBiasX;
  gy_dps = (gyr / GYRO_LSB) - gyroBiasY;
  gz_dps = (gzr / GYRO_LSB) - gyroBiasZ;

  // Inclinación del robot: Pitch sobre el eje Y (MPU montado plano).
  tiltAccel = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD2DEG;
  rateUsada = gy_dps;
}

// ============================================================
// == SETUP ==
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motores en 0 para que no se activen durante el debug
  pinMode(pinMizqA, OUTPUT);
  pinMode(pinMizqB, OUTPUT);
  pinMode(pinMderA, OUTPUT);
  pinMode(pinMderB, OUTPUT);
  digitalWrite(pinMizqA, LOW);
  digitalWrite(pinMizqB, LOW);
  digitalWrite(pinMderA, LOW);
  digitalWrite(pinMderB, LOW);

  Serial.println("===== DEBUG MPU6050 CON FILTRO DE KALMAN =====");
  Serial.println("SDA = GPIO8 | SCL = GPIO7 | AD0 -> GND (0x68)");

  if (!mpuIniciar()) {
    Serial.println("Revisar conexiones (VCC=3.3V, GND, SDA, SCL) y reiniciar.");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 inicializado OK");

  mpuCalibrarBias();

  Kalman_Init(&kalman);
  kalman.angle = offTilt; // arranca con la inclinación inicial (que es el cero)
  giroY = 0;

  Serial.print("FlagRaw: "); Serial.println(FlagRaw);
  Serial.print("FlagAngles: "); Serial.println(FlagAngles);
  Serial.println("Comandos Serial: 'c' = recalibrar bias | 'z' = cero de angulos");
}

// ============================================================
// == LOOP ==
// ============================================================

void loop() {
  // Comandos por Serial
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      procesarComando(cmdSerial);
      cmdSerial = "";
    } else {
      cmdSerial += ch;
    }
  }

  mpuLeer();

  // Cálculo del dt real
  uint32_t ahora = micros();
  float dt = (ahora - ultimoTiempoUs) / 1000000.0f;
  ultimoTiempoUs = ahora;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;

  // Filtro de Kalman para la inclinación
  float tiltK = Kalman_GetAngle(&kalman, tiltAccel, rateUsada, dt);

  // Ángulo de giro del robot (eje Y, sin referencia de gravedad)
  giroY += gy_dps * dt;

  if (FlagRaw == 1) {
    Serial.printf("AX:%+1.3fg AY:%+1.3fg AZ:%+1.3fg | GX:%+7.2f GY:%+7.2f GZ:%+7.2f °/s | T:%.1f°C\n",
                  ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, tempC);
  }

  if (FlagAngles == 1) {
    Serial.printf("TiltK:%+8.2f° TiltAcc:%+8.2f° Rate:%+7.2f°/s BiasK:%+7.3f | GiroY:%+8.2f°\n",
                  tiltK - offTilt, tiltAccel - offTilt, rateUsada, kalman.bias, giroY);
  }

  delay(10);
}

// ============================================================
// == COMANDOS SERIAL ==
// ============================================================

void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toLowerCase();
  char c = cmd[0];

  if (c == 'c') {
    mpuCalibrarBias();
    Kalman_Init(&kalman);
    kalman.angle = offTilt;
    giroY = 0;
  } else if (c == 'z') {
    // Cero manual: el ángulo actual del Kalman pasa a ser el nuevo cero
    offTilt = kalman.angle;
    giroY = 0;
    Serial.println("Angulos puestos a cero.");
  } else {
    Serial.println("Comandos: 'c' = recalibrar | 'z' = cero de angulos");
  }
}
