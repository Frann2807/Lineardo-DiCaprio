// ============================================================
// DEBUG MPU_PID - PID PROPORCIONAL AL ÁNGULO DEL ROBOT
// ESP32-C3 Super Mini - I2C crudo (MPU6050)
//
// El eje de giro del robot es el GYRO Y del MPU (ver DebugMPU).
// El ángulo del robot se obtiene integrando el giro Y con el
// bias calibrado restado, y arranca en 0.
//
// Regla de corrección:
//   correccion = -K_mpu * anguloY
//     -> anguloY NEGATIVO -> corrección POSITIVA -> aumenta velocidad
//     -> anguloY POSITIVO -> corrección NEGATIVA -> disminuye velocidad
//
// Aplicación (comando 'd' para alternar):
//   DIF (diferencial): M_IZQ = velBase + correccion
//                      M_DER = velBase - correccion
//   COM (común):       M_IZQ = velBase + correccion
//                      M_DER = velBase + correccion
//
// Flags:
//   FlagRaw     = 1 -> muestra giros crudos GX GY GZ (°/s)
//   FlagPID     = 1 -> muestra ángulo, corrección y salidas de motor
//   FlagMotores = 0 -> SOLO imprime (seguro) | 1 -> aplica a motores
//
// Comandos Serial (una línea + Enter):
//   k<valor>  -> cambiar K_mpu  (ej: k0.5, k-0.3)
//   v<valor>  -> cambiar velocidad base (ej: v40)
//   d         -> alternar aplicación DIF / COM
//   c         -> recalibrar bias del giroscopio (robot QUIETO)
//   z         -> poner el ángulo del robot en 0
//   m         -> alternar FlagMotores (imprimir/aplicar)
// ============================================================

#include <Wire.h>

// ===== PINES I2C (ESP32-C3 Super Mini) =====
#define PIN_SDA 8
#define PIN_SCL 7

// ===== PINES DE MOTORES =====
// M_IZQ = motor 1 (pines A=9, B=10)
// M_DER = motor 2 (pines A=21, B=20)
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
#define MPU_GYRO_XOUT_H   0x43
#define MPU_WHO_AM_I      0x75

// ===== FACTORES DE CONVERSIÓN =====
// Giroscopio configurado a ±1000 °/s -> 32.8 LSB por °/s
const float GYRO_LSB = 32.8f;

// ===== FLAGS =====
int FlagRaw = 0;      // 1 = mostrar giros crudos
int FlagPID = 1;      // 1 = mostrar ángulo + corrección + salidas
int FlagMotores = 0;  // 0 = solo imprimir | 1 = aplicar a motores
int modoAplicacion = 0; // 0 = DIF (diferencial) | 1 = COM (común)

// ===== BIAS DEL GIROSCOPIO (calibración) =====
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ===== VARIABLES DEL PID DE ÁNGULO =====
float K_mpu = 0;      // Ganancia (sintonizar con 'k<valor>')
float velBase = 40;   // Velocidad base (% de PWM)
float anguloY = 0;    // Ángulo del robot (integral del giro Y)
float gx_dps, gy_dps, gz_dps;
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
  Wire.begin(PIN_SDA, PIN_SCL);   // SDA = 8, SCL = 7
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

  mpuWrite(MPU_PWR_MGMT_1, 0x80);   // reset del dispositivo
  delay(100);
  mpuWrite(MPU_PWR_MGMT_1, 0x01);   // despertar, reloj = PLL del giro X
  mpuWrite(MPU_SMPLRT_DIV, 0x00);   // sample rate = 1kHz
  mpuWrite(MPU_CONFIG, 0x02);       // DLPF ~98Hz (giro) / ~94Hz (acel)
  mpuWrite(MPU_GYRO_CONFIG, 0x10);  // FS_SEL=2 -> ±1000 °/s
  mpuWrite(MPU_ACCEL_CONFIG, 0x08); // AFS_SEL=1 -> ±4 g
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
// == LECTURA DEL MPU6050 (solo giroscopio) ==
// ============================================================

void mpuLeer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

  int16_t gxr = (Wire.read() << 8) | Wire.read();
  int16_t gyr = (Wire.read() << 8) | Wire.read();
  int16_t gzr = (Wire.read() << 8) | Wire.read();

  gx_dps = (gxr / GYRO_LSB) - gyroBiasX;
  gy_dps = (gyr / GYRO_LSB) - gyroBiasY;
  gz_dps = (gzr / GYRO_LSB) - gyroBiasZ;
}

// ============================================================
// == MOTORES ==
// ============================================================

void setMotors(float mIzq, float mDer) {
  if (abs(mIzq) > 100) mIzq = 100 * (mIzq / abs(mIzq));
  if (abs(mDer) > 100) mDer = 100 * (mDer / abs(mDer));
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

// ============================================================
// == COMANDOS SERIAL ==
// ============================================================

void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toLowerCase();
  char c = cmd[0];

  if (c == 'k') {
    K_mpu = cmd.substring(1).toFloat();
    Serial.printf("K_mpu = %.3f\n", K_mpu);
  } else if (c == 'v') {
    velBase = cmd.substring(1).toFloat();
    Serial.printf("velBase = %.1f\n", velBase);
  } else if (c == 'd') {
    modoAplicacion = !modoAplicacion;
    Serial.printf("Aplicacion: %s\n",
                  modoAplicacion == 0 ? "DIF (diferencial)" : "COM (comun)");
  } else if (c == 'c') {
    mpuCalibrarBias();
    anguloY = 0;
  } else if (c == 'z') {
    anguloY = 0;
    Serial.println("Angulo puesto a cero.");
  } else if (c == 'm') {
    FlagMotores = !FlagMotores;
    Serial.printf("FlagMotores: %d\n", FlagMotores);
  } else {
    Serial.println("Comandos: k<valor> | v<valor> | d | c | z | m");
  }
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

  Serial.println("===== DEBUG MPU_PID (ANGULO DEL ROBOT) =====");

  if (!mpuIniciar()) {
    Serial.println("Revisar conexiones (VCC=3.3V, GND, SDA=8, SCL=7) y reiniciar.");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 inicializado OK");

  mpuCalibrarBias();
  anguloY = 0;

  Serial.printf("K_mpu: %.3f | velBase: %.1f | FlagMotores: %d | Aplicacion: %s\n",
                K_mpu, velBase, FlagMotores,
                modoAplicacion == 0 ? "DIF" : "COM");
  Serial.println("Comandos: k<valor> | v<valor> | d | c | z | m");
}

// ============================================================
// == LOOP ==
// ============================================================

void loop() {
  // --- Comandos Serial (línea + Enter) ---
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

  // Integración del ángulo del robot (eje de giro = giro Y)
  anguloY += gy_dps * dt;

  // Corrección proporcional al ángulo
  // anguloY negativo -> corrección positiva -> aumenta velocidad
  // anguloY positivo -> corrección negativa -> disminuye velocidad
  float correccion = -K_mpu * anguloY;

  float mIzq, mDer;
  if (modoAplicacion == 0) {
    // DIF: diferencial (gira hacia el ángulo 0)
    mIzq = velBase + correccion;
    mDer = velBase - correccion;
  } else {
    // COM: común (acelera/frena ambas ruedas)
    mIzq = velBase + correccion;
    mDer = velBase + correccion;
  }

  if (FlagRaw == 1) {
    Serial.printf("GX:%+7.2f GY:%+7.2f GZ:%+7.2f °/s\n", gx_dps, gy_dps, gz_dps);
  }
  if (FlagPID == 1) {
    Serial.printf("AngY:%+8.2f° Corr:%+8.2f | M_IZQ:%+7.2f M_DER:%+7.2f\n",
                  anguloY, correccion, mIzq, mDer);
  }

  if (FlagMotores == 1) setMotors(mIzq, mDer);
  else setMotors(0, 0);

  delay(10);
}
