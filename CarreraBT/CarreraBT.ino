// ============================================================
// SEGUIDOR DE LÍNEA - MODO CARRERA CON BLE (NUS) - SIN MPU
// Placa: ESP32-C3 Super Mini
// Arquitectura: FreeRTOS (tarea de control) + BLE NimBLE
//
// - Sensores: 8 canales por multiplexor (CD4051/74HC4051)
//   con interpolación cuadrática (bfs::polyfit).
// - PID de posición aplicado cada 4ms (Ts) directo al PWM.
//   SIN encoders: ya no hay PID de velocidad.
// - MPU6050 con FILTRO DE KALMAN (SDA=8, SCL=7): estima la
//   inclinación del robot y corrige la velocidad según el ángulo
//   (ganancia 'kmpu', corrección COMÚN a ambas ruedas).
//   Comandos: 'calibrar' (bias + cero) y 'cero' (solo cero).
// - Freno de seguridad: si la línea se pierde más de 'lost' ms
//   (500 por defecto), los motores se apagan solos.
// - Parámetros calibrables por BLE con el intérprete de
//   comandos de texto (escribir 'help' con salto de línea).
// - Inicio/freno con el botón (GPIO 4):
//     * presionar mientras corre  -> FRENO
//     * mantener presionado y soltar -> REANUDAR
// ============================================================

// ==================================================
// == BIBLIOTECAS ==
// ==================================================
#include <NimBLEDevice.h>
#include <polytools.h>
#include <array>
#include <string.h> // Para strcpy / sprintf del debug
#include <Preferences.h> // Para guardar valores en flash (comando 'guardar')
#include <Wire.h> // Para el MPU6050 (I2C crudo)
#include <math.h> // Para atan2 / fabs (inclinación con el Kalman)

// ==================================================
// == CONFIGURACIÓN DE PINES (ESP32-C3 Super Mini) ==
// ==================================================
// --- Pines del Multiplexor ---
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_SIG 3

// --- Pines de los motores ---
// M_IZQ = motor 1 (pines A=9, B=10)
// M_DER = motor 2 (pines A=21, B=20)
#define pinMizqA 9
#define pinMizqB 10
#define pinMderA 21
#define pinMderB 20

// --- Botón (GPIO 4) ---
#define pinBoton 5

// --- MPU6050 (I2C) ---
#define PIN_SDA 8
#define PIN_SCL 7

// ==================================================
// == CONFIGURACIÓN BLE (NORDIC UART SERVICE) ==
// ==================================================
#define NUS_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static NimBLEServer* pServer;
static NimBLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ==================================================
// == PERSISTENCIA EN FLASH (Preferences / NVS) ==
// ==================================================
Preferences prefs;
const char* NVS_NAMESPACE = "mecabot";

// ==================================================
// == TAREAS Y ESTADO DEL ROBOT ==
// ==================================================
TaskHandle_t h_TaskPosicion;
volatile bool isPaused = true;

// --- Botón (debounce) ---
bool lastStableButtonState = HIGH; // Último estado estable (PULLUP = HIGH)
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // 50ms
int loopButtonState = 0; // 0 = Idle, 1 = Esperando liberación para reanudar

// ==================================================
// == SENSORES Y POSICIÓN ==
// ==================================================
const int NUM_SENSORES = 8;
int ordenCanales[NUM_SENSORES] = { 3, 0, 1, 2, 4, 6, 7, 5 };
std::array<float, NUM_SENSORES> x;
std::array<float, NUM_SENSORES> y = {0};
std::array<float, 3> p = {};

volatile int linea = 0; // 0=Negra, 1=Blanca
float umbral = 2000;
volatile float posicion;
float ultimaPosicion = (NUM_SENSORES + 1) / 2.0; // 4.5
volatile int sensOn = 0;

// ==================================================
// == PID DE POSICIÓN ==
// ==================================================
float setPos = 4.5; // (NUM_SENSORES + 1) / 2.0; // 4.5
float P_pos = 0, I_pos = 0, D_pos = 0, LastP_pos = 0;
volatile float posControl = 0;
volatile float corrMpu = 0; // Corrección del MPU (para debug)

// --- Parámetros PID (sintonizar desde BLE) ---
float Kp_pos = 12;
float Ki_pos = 20;
float Kd_pos = 2;
float velBase = 30;    // Velocidad base (% de PWM)
float coefMax = 0.9;
float coefMin = -0.3;

// --- Período de muestreo (ms) ---
volatile int Ts = 10;
const float MAX_I_POS = 1000.0; // Anti-Windup

// --- Salidas de motor (para debug) ---
volatile float targetA = 0;
volatile float targetB = 0;

// Ver posición por BLE (1 = sí, 0 = no)
volatile int flagPos = 0;

// Debug MPU por Serial de la PC (1 = sí, 0 = no), independiente del modo carrera
volatile int flagMpuDebug = 1;

// --- Comando 'ruedas' (limpieza de ruedas) ---
volatile bool ruedasActivas = false;
unsigned long ruedasInicio = 0;
bool ruedasPausaPrevia = false;
const int RUEDAS_PWM = 80;
const unsigned long RUEDAS_TIEMPO_MS = 3000;

// --- Freno de seguridad por pérdida de línea ---
volatile int lostMaxMs = 500; // ms sin línea antes de frenar (comando 'lost', 0 = apagado)

// ==================================================
// == MPU6050 CON FILTRO DE KALMAN (inclinación) ==
// ==================================================
#define MPU_ADDR          0x68
#define MPU_PWR_MGMT_1    0x6B
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_GYRO_XOUT_H   0x43
#define MPU_WHO_AM_I      0x75

const float GYRO_LSB = 32.8f;      // ±1000 °/s -> 32.8 LSB por °/s
const float ACCEL_LSB = 8192.0f;   // ±4 g -> 8192 LSB por g
const float RAD2DEG = 57.2957795f;
const float CORR_MPU_MAX = 500;   // Límite de la corrección del MPU (%)
const float ALPHA_CORR = 0.1f;    // Suavizado de la corrección (evita frenadas bruscas)

volatile bool mpuOK = false;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
volatile float K_mpu = 1;          // Ganancia según el ángulo (comando 'kmpu')
volatile float velBaja = 10;       // Velocidad base al activar el freno por ángulo (comando 'velbaja')
volatile float anguloFreno = 10;   // Ángulo (°) que dispara la bajada de velocidad (comando 'angfreno')
volatile unsigned long velBajaMs = 1000; // Duración de la velocidad baja (comando 'frenoms')
volatile float offTilt = 0;        // Cero de inclinación
volatile float tiltKalman = 0;     // Salida del Kalman con cero aplicado (°)
volatile bool flagMpuCal = false;  // Pide calibración (se ejecuta en loop)
volatile bool flagMpuZero = false; // Pide cero del ángulo (se ejecuta en loop)
volatile bool mpuCalibrando = false; // La tarea no lee el MPU mientras calibra

float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float tiltAccel = 0;               // Ángulo medido por el acelerómetro
float rateUsada = 0;               // Giro del eje del tilt

// --- Filtro de Kalman (adaptado del código STM32) ---
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

// ==================================================
// == PROTOTIPOS DE FUNCIONES ==
// ==================================================
void TaskPosicion(void *pvParameters);
void freno();
void frenoSeguridad();
void setMotors(float m1, float m2);
void findPosicion();
void setMuxChannel(int channel);
bool mpuIniciar();
void mpuCalibrarBias();
void mpuLeer();
void btPrint(String s);
void btPrintln(String s);
void btNotificar(String s);
void btFlushCola();
void comandProcess(String cmd);
void mostrarAyuda();
void mostrarValores();
void leerSensores();

// ##################################################################
// ##                      CALLBACKS BLE                          ##
// ##################################################################

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  String rxBuffer; // Acumulador hasta recibir '\n'
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    rxBuffer += String(value.c_str());

    int idx;
    while ((idx = rxBuffer.indexOf('\n')) >= 0) {
      String cmd = rxBuffer.substring(0, idx);
      rxBuffer = rxBuffer.substring(idx + 1);
      cmd.trim();
      cmd.toLowerCase();
      if (cmd.length() > 0) comandProcess(cmd);
    }

    if (rxBuffer.length() > 256) rxBuffer = ""; // Protección
  }
};

/** Callbacks para el Servidor (Conexión/Desconexión) */
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("Dispositivo BLE conectado");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("Dispositivo BLE desconectado");
    NimBLEDevice::startAdvertising();
  }
};

// ##################################################################
// ##                           SETUP                             ##
// ##################################################################

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- 0. Restaurar valores guardados en flash (comando 'guardar') ---
  if (prefs.begin(NVS_NAMESPACE, true)) {
    if (prefs.isKey("velBase")) {
      velBase   = prefs.getFloat("velBase", velBase);
      Kp_pos    = prefs.getFloat("kp", Kp_pos);
      Ki_pos    = prefs.getFloat("ki", Ki_pos);
      Kd_pos    = prefs.getFloat("kd", Kd_pos);
      coefMax   = prefs.getFloat("max", coefMax);
      coefMin   = prefs.getFloat("min", coefMin);
      Ts        = prefs.getInt("ts", Ts);
      linea     = prefs.getInt("linea", linea);
      umbral    = prefs.getFloat("umbral", umbral);
      lostMaxMs = prefs.getInt("lost", lostMaxMs);
      K_mpu     = prefs.getFloat("kmpu", K_mpu);
      velBaja   = prefs.getFloat("velbaja", velBaja);
      anguloFreno = prefs.getFloat("angfreno", anguloFreno);
      velBajaMs = prefs.getInt("frenoms", velBajaMs);
      Serial.println("Valores restaurados desde flash.");
    }
    prefs.end();
  }

  // --- 1. Inicializar MPU6050 (Kalman de inclinación) ---
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (mpuIniciar()) {
    mpuCalibrarBias();
    Kalman_Init(&kalman);
    kalman.angle = offTilt;
    tiltKalman = 0;
    mpuOK = true;
    Serial.println("MPU6050 listo: Kalman activo.");
  } else {
    mpuOK = false;
    Serial.println("MPU6050 NO encontrado: se corre SIN correccion de angulo.");
  }

  // --- 2. Inicializar BLE ---
  NimBLEDevice::init("Lineardo_DiCaprio");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  NimBLEService* pNusService = pServer->createService(NUS_SERVICE_UUID);
  NimBLECharacteristic* pRxCharacteristic =
      pNusService->createCharacteristic(NUS_CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE);
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pTxCharacteristic =
      pNusService->createCharacteristic(NUS_CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);

  pNusService->start();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("Lineardo_DiCaprio");
  pAdvertising->addServiceUUID(pNusService->getUUID());
  pAdvertising->start();
  Serial.println("Esperando conexion BLE...");

  // --- 3. Configurar MUX ---
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  // --- 4. Configurar Motores ---
  pinMode(pinMizqA, OUTPUT);
  pinMode(pinMizqB, OUTPUT);
  pinMode(pinMderA, OUTPUT);
  pinMode(pinMderB, OUTPUT);
  setMotors(0, 0);

  // --- 5. Configurar Botón ---
  pinMode(pinBoton, INPUT_PULLUP);
  lastStableButtonState = digitalRead(pinBoton);
  lastDebounceTime = millis(); // Sincronizamos el timer

  // --- 6. Inicializar array 'x' (Rango 1.0 a 8.0) ---
  for (int i = 0; i < NUM_SENSORES; i++) {
    x[i] = i + 1.0;
  }

  // --- 7. ESPERA DEL BOTÓN DE ARRANQUE (Lógica "ARRANCAR AL SOLTAR") ---
  Serial.println("\nRobot PAUSADO. MANTEN presionado el boton (GPIO 4)...");
  isPaused = true;

  int buttonWaitState = 0; // 0 = espera press, 1 = espera release
  bool lastReadingState_Setup = HIGH;

  while (isPaused) {
    bool currentReading = digitalRead(pinBoton);

    // 1. Si la lectura cambia, resetear timer
    if (currentReading != lastReadingState_Setup) {
      lastDebounceTime = millis();
    }

    // 2. Si la señal es estable (ha pasado el tiempo de debounce)
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // 3. Y si este estado estable es NUEVO
      if (currentReading != lastStableButtonState) {
        lastStableButtonState = currentReading; // Guardamos el nuevo estado global

        if (buttonWaitState == 0) {
          // Esperando PRESIÓN
          if (lastStableButtonState == LOW) {
            Serial.println("Boton presionado... ¡SUELTA para comenzar!");
            buttonWaitState = 1; // Pasamos a esperar la liberación
          }
        } else if (buttonWaitState == 1) {
          // Esperando LIBERACIÓN
          if (lastStableButtonState == HIGH) {
            Serial.println("¡Boton soltado! ARRANCANDO MODO CARRERA...");
            isPaused = false; // ¡Arrancamos!
          }
        }
      }
    }

    // 4. Guardar lectura para la próxima iteración del setup-loop
    lastReadingState_Setup = currentReading;

    // Este delay(10) es VITAL para que BLE siga funcionando
    delay(10);
  }

  // ¡IMPORTANTE! Reseteamos el timer de debounce OTRA VEZ,
  // para que el loop() no se confunda con la liberación del setup.
  lastDebounceTime = millis();

  // --- 8. Crear Tarea de Control ---
  xTaskCreatePinnedToCore(
      TaskPosicion, "PID Posicion", 4096, NULL, 2, &h_TaskPosicion, 0);

  Serial.println("¡Setup completo! Control activo (Ts = 4ms).");
  Serial.println("Comandos: escribir 'help' por BLE (con salto de linea).");
}

// ##################################################################
// ##               TAREA DE POSICIÓN (LOOP Ts ms)                ##
// ##################################################################
void TaskPosicion(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Seguimiento de pérdida de línea
  bool estabaPerdido = false;
  unsigned long lostDesdeMs = 0;

  // Freno por ángulo grande (velocidad baja temporal)
  static bool velBajaActiva = false;
  static unsigned long velBajaInicio = 0;

  for (;;) {
    TickType_t xFrequency = pdMS_TO_TICKS(Ts);
    if (xFrequency < 1) xFrequency = 1;

    // --- Modo limpieza de ruedas (comando 'ruedas') ---
    if (ruedasActivas) {
      if (millis() - ruedasInicio < RUEDAS_TIEMPO_MS) {
        setMotors(RUEDAS_PWM, RUEDAS_PWM);
      } else {
        setMotors(0, 0);
        ruedasActivas = false;
        isPaused = ruedasPausaPrevia; // Volver al estado anterior
        P_pos = 0; I_pos = 0; D_pos = 0; LastP_pos = 0;
        posControl = 0;
        corrMpu = 0;
        targetA = 0; targetB = 0;
        btPrintln("Ruedas listas.");
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }

    // --- Leer MPU y actualizar el Kalman (siempre, también pausado) ---
    if (mpuOK && !mpuCalibrando) {
      float dt = Ts / 1000.0f;
      if (dt < 0.001f) dt = 0.001f;
      mpuLeer();
      tiltKalman = Kalman_GetAngle(&kalman, tiltAccel, rateUsada, dt) - offTilt;
    }

    if (isPaused) {
      P_pos = 0; I_pos = 0; D_pos = 0; LastP_pos = 0;
      posControl = 0;
      corrMpu = 0;
      velBajaActiva = false;
      targetA = 0;
      targetB = 0;
      estabaPerdido = false;
      lostDesdeMs = 0;
      setMotors(0, 0);
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }

    findPosicion();

    float dt = Ts / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;

    // --- Freno por ángulo grande: si el ángulo supera 'angfreno',
    // la velocidad base baja a 'velbaja' durante 1 segundo ---
    float velActual = velBase;
    if (!velBajaActiva && tiltKalman > anguloFreno) {
      velBajaActiva = true;
      velBajaInicio = millis();
    }
    if (velBajaActiva) {
      velActual = velBaja;
      if (millis() - velBajaInicio >= velBajaMs) {
        velBajaActiva = false;
      }
    }

    if (sensOn == 0) {
      // --- Línea perdida ---
      if (!estabaPerdido) {
        lostDesdeMs = millis();
        estabaPerdido = true;
      }

      // Freno de seguridad: demasiado tiempo sin ver la línea
      if (lostMaxMs > 0 && (millis() - lostDesdeMs) > (unsigned long)lostMaxMs) {
        frenoSeguridad();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }

      // Girar fuerte hacia el último lado conocido
      if (posicion > setPos) {
        targetA = velActual * coefMax;
        targetB = velActual * coefMin;
      } else {
        targetA = velActual * coefMin;
        targetB = velActual * coefMax;
      }
      posControl = 0;
      corrMpu = 0;
      I_pos = 0;
    } else {
      // --- Línea detectada ---
      estabaPerdido = false;
      lostDesdeMs = 0;

      P_pos = posicion - setPos;

      // Flip sign: si el error cambió de signo, la integral se resetea
      if ((P_pos > 0 && LastP_pos < 0) || (P_pos < 0 && LastP_pos > 0)) {
        I_pos = 0;
      }

      I_pos += P_pos * dt;
      I_pos = constrain(I_pos, -MAX_I_POS, MAX_I_POS);
      D_pos = (P_pos - LastP_pos) / dt;
      LastP_pos = P_pos;
      posControl = Kp_pos * P_pos + Ki_pos * I_pos + Kd_pos * D_pos;

      // Corrección por ángulo (Kalman): COMÚN a ambas ruedas,
      // activa TODO EL TIEMPO según el ángulo (con suavizado).
      float corrDeseada = constrain(-K_mpu * tiltKalman, -CORR_MPU_MAX, CORR_MPU_MAX);
      corrMpu += ALPHA_CORR * (corrDeseada - corrMpu);
      if (fabs(corrMpu) < 0.5f) corrMpu = 0;

      targetA = velActual + posControl + corrMpu;
      targetB = velActual - posControl + corrMpu;
    }

    setMotors(targetA, targetB);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ##################################################################
// ##                    LOOP (TAREA 50ms)                        ##
// ##################################################################

void loop() {
  // --- A. Manejo de Conexión BLE ---
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = false;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }

  // --- B. Imprimir Debug (Tarea lenta) ---
  if (flagPos == 1) {
    char debugString[80];
    char posStrBuffer[10];

    if (sensOn == 0) {
      strcpy(posStrBuffer, "LOST");
    } else {
      sprintf(posStrBuffer, "%.2f", posicion);
    }

    sprintf(debugString, "P:%s C:%.0f Ang:%.1f MIZQ:%.0f MDER:%.0f",
            posStrBuffer,   // P:4.50 o P:LOST
            posControl,     // C:-150
            tiltKalman,     // Ang: inclinación del Kalman (°)
            targetA,        // MIZQ: salida motor izquierdo (%)
            targetB);       // MDER: salida motor derecho (%)

    Serial.println(debugString);
    btNotificar(String(debugString));
  }

  // --- B2. Debug MPU por Serial de la PC (independiente del modo carrera) ---
  if (flagMpuDebug == 1) {
    //corrMpu = constrain(-K_mpu * tiltKalman, -CORR_MPU_MAX, CORR_MPU_MAX);
    char mpuDbg[64];
    sprintf(mpuDbg, "MPU: Ang:%+7.2f° Corr:%+7.2f MIZQ:%+7.2f MDER:%+7.2f",
            tiltKalman, corrMpu, targetA, targetB);
    Serial.println(mpuDbg);
  }

  // --- C. Manejo del Botón de Pausa/Reanudación ---
  static bool lastReadingState_Loop = HIGH;
  bool currentReading = digitalRead(pinBoton);

  // 1. Si la lectura cambia (sea rebote o no), reseteamos el timer
  if (currentReading != lastReadingState_Loop) {
    lastDebounceTime = millis();
  }

  // 2. Si el tiempo desde el último cambio es mayor al delay (la señal es estable)
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // 3. Y si este estado estable *nuevo* es diferente del *último estado estable guardado*
    if (currentReading != lastStableButtonState) {
      // 4. Guardamos el NUEVO estado estable
      lastStableButtonState = currentReading;

      // --- 5. MÁQUINA DE ESTADOS ---

      // --- CASO A: El botón fue PRESIONADO (LOW) ---
      if (lastStableButtonState == LOW) {
        if (!isPaused) {
          // A.1: Estaba corriendo -> PAUSAR
          freno(); // Esto pone isPaused = true
          loopButtonState = 0;
        } else {
          // A.2: Estaba pausado -> Iniciar espera para reanudar
          loopButtonState = 1; // "Esperando Liberación"
        }
      }
      // --- CASO B: El botón fue SOLTADO (HIGH) ---
      else {
        // B.1: Si estaba pausado Y estábamos esperando la liberación
        if (isPaused && loopButtonState == 1) {
          isPaused = false;
          btPrintln("Reanudando.");
        }
        // B.2: Siempre reseteamos el estado al soltar
        loopButtonState = 0;
      }
    } // Fin del if (currentReading != lastStableButtonState)
  } // Fin del if (millis() - lastDebounceTime)

  // 6. Guardamos la lectura actual para la próxima iteración
  lastReadingState_Loop = currentReading;

  // --- D. Acciones del MPU pedidas por comandos ---
  if (flagMpuCal) {
    mpuCalibrando = true;
    mpuCalibrarBias();
    Kalman_Init(&kalman);
    kalman.angle = offTilt;
    tiltKalman = 0;
    mpuCalibrando = false;
    flagMpuCal = false;
    btPrintln("MPU calibrado: angulo en 0.");
  }
  if (flagMpuZero) {
    offTilt = kalman.angle; // El ángulo actual pasa a ser el nuevo cero
    tiltKalman = 0;
    flagMpuZero = false;
    btPrintln("Angulo del MPU en 0.");
  }

  // --- E. Vaciar cola de envío BLE a ritmo seguro ---
  btFlushCola();

  // --- F. Dormir el loop ---
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ##################################################################
// ##              FUNCIONES DE LÓGICA (SENSORES)                 ##
// ##################################################################

void findPosicion() {
  sensOn = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    int canal = ordenCanales[i];
    setMuxChannel(canal);
    delayMicroseconds(50);
    y[i] = analogRead(MUX_SIG);
  }
  int xaux = 0;
  if (linea == 0) { // Linea NEGRA
    float yaux = 4096;
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (y[i] < yaux && y[i] < umbral) { yaux = y[i]; xaux = i; }
      if (y[i] < umbral) { sensOn++; }
    }
  } else { // Linea BLANCA
    float yaux = 0;
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (y[i] > yaux && y[i] > umbral) { yaux = y[i]; xaux = i; }
      if (y[i] > umbral) { sensOn++; }
    }
  }

  if (xaux > 0 && xaux < (NUM_SENSORES - 1) && sensOn > 0) { // Caso central
    std::array<float, 3> xII = { x[xaux - 1], x[xaux], x[xaux + 1] };
    std::array<float, 3> yII = { y[xaux - 1], y[xaux], y[xaux + 1] };
    p = bfs::polyfit<2>(xII, yII);
    posicion = -p[1] / (2 * p[0]);
  } else if (xaux == 0 && sensOn > 0) { // Borde izquierdo
    std::array<float, 3> xII = { x[xaux], x[xaux + 1], x[xaux + 2] };
    std::array<float, 3> yII = { y[xaux], y[xaux + 1], y[xaux + 2] };
    p = bfs::polyfit<2>(xII, yII);
    posicion = (p[0] > 0) ? (-p[1] / (2 * p[0])) : 1.0;
  } else if (xaux == (NUM_SENSORES - 1) && sensOn > 0) { // Borde derecho
    std::array<float, 3> xII = { x[xaux - 2], x[xaux - 1], x[xaux] };
    std::array<float, 3> yII = { y[xaux - 2], y[xaux - 1], y[xaux] };
    p = bfs::polyfit<2>(xII, yII);
    posicion = (p[0] > 0) ? (-p[1] / (2 * p[0])) : (float)NUM_SENSORES;
  } else if (sensOn == 0) { // Si no ve nada
    posicion = ultimaPosicion;
  }

  if (posicion > (float)NUM_SENSORES) posicion = (float)NUM_SENSORES; // 8.0
  if (posicion < 1.0) posicion = 1.0; // 1.0
  if (sensOn > 0) {
    ultimaPosicion = posicion;
  }
}

void setMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
}

// ##################################################################
// ##              FUNCIONES DE LÓGICA (MOTORES)                  ##
// ##################################################################

void freno() {
  setMotors(0, 0);
  targetA = 0;
  targetB = 0;
  P_pos = 0; I_pos = 0; D_pos = 0; LastP_pos = 0;
  posControl = 0;
  isPaused = true; // --- Activa la pausa ---

  btPrintln("--- FRENO ---");
  btPrintln("Datos finales:");
  btPrintln("Vel Base: " + String(velBase));
  btPrintln("Kp: " + String(Kp_pos));
  btPrintln("Kd: " + String(Kd_pos));
  btPrintln("Ki: " + String(Ki_pos));
  btPrintln("Coef max: " + String(coefMax));
  btPrintln("Coef min: " + String(coefMin));
  btPrintln("Ts: " + String(Ts));
  btPrintln("Linea: " + String(linea));
  btPrintln("Umbral: " + String(umbral));
  btPrintln("Mantener presionado el boton para reanudar (o comando 'arranque').");
}

void frenoSeguridad() {
  setMotors(0, 0);
  targetA = 0;
  targetB = 0;
  P_pos = 0; I_pos = 0; D_pos = 0; LastP_pos = 0;
  posControl = 0;
  isPaused = true; // --- Activa la pausa ---

  btPrintln("¡FRENO DE SEGURIDAD! Linea perdida demasiado tiempo.");
  btPrintln("Reanudar con el boton o con el comando 'arranque'.");
}

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
// == FUNCIONES MPU6050 (I2C CRUDO - KALMAN DE INCLINACIÓN)  ==
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

bool mpuIniciar() {
  uint8_t who = mpuReadReg(MPU_WHO_AM_I);
  if (who == 0x00 || who == 0xFF) return false;
  if (who != 0x68) {
    Serial.printf("AVISO: WHO_AM_I = 0x%02X (esperado 0x68).\n", who);
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

  // Inclinación del robot: Pitch sobre el eje Y (MPU montado plano).
  // Ángulo con la gravedad usando ax contra el plano ay-az,
  // giro correspondiente = GY.
  tiltAccel = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD2DEG;
  rateUsada = gy_dps;
}

// ============================================================
// == SALIDA BLE / SERIAL (COLA DE TRANSMISIÓN)               ==
// ============================================================

// Cola de TX: los mensajes se ENCOLAN y se envían desde loop()
// a ritmo seguro. Si se mandan muchas notificaciones seguidas
// desde el callback de escritura, el teléfono pierde la mayoría.
String txCola = "";
portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;

void btNotificar(String s) {
  if (!deviceConnected) return;
  portENTER_CRITICAL(&txMux);
  txCola += s;
  if (txCola.length() > 8192) txCola = ""; // Protección (no debería pasar)
  portEXIT_CRITICAL(&txMux);
}

void btPrint(String s) {
  Serial.print(s);
  btNotificar(s);
}

void btPrintln(String s) {
  btPrint(s + "\n");
}

// Se llama desde loop(): envía hasta MAX_ENVIOS chunks por vez,
// dejando tiempo entre tandas para que el teléfono procese todo.
void btFlushCola() {
  if (txCola.length() == 0) return;
  if (!deviceConnected || pTxCharacteristic == nullptr) {
    portENTER_CRITICAL(&txMux);
    txCola = "";
    portEXIT_CRITICAL(&txMux);
    return;
  }

  const int CHUNK = 20;      // Notificaciones de hasta 20 bytes
  const int MAX_ENVIOS = 3;  // Chunks por llamada (loop = 50ms)

  for (int i = 0; i < MAX_ENVIOS; i++) {
    String chunk;
    portENTER_CRITICAL(&txMux);
    if (txCola.length() == 0) {
      portEXIT_CRITICAL(&txMux);
      break;
    }
    chunk = txCola.substring(0, CHUNK);
    txCola = txCola.substring(CHUNK);
    portEXIT_CRITICAL(&txMux);

    pTxCharacteristic->setValue(chunk.c_str());
    pTxCharacteristic->notify();
  }
}

// ##################################################################
// ##              INTÉRPRETE DE COMANDOS (BLE)                   ##
// ##################################################################

void comandProcess(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() < 1) {
    btPrintln("Comando invalido");
    return;
  }

  // Separar el nombre (letras) del valor numérico (admite negativos)
  String nombre = "";
  int i = 0;
  while (i < cmd.length() && !isDigit(cmd[i]) && cmd[i] != '-' && cmd[i] != '.' && cmd[i] != '+') {
    nombre += cmd[i];
    i++;
  }
  String valorStr = cmd.substring(i);
  float valor = (valorStr.length() > 0) ? valorStr.toFloat() : 0;

  // Comandos sin valor
  if (nombre == "help") {
    mostrarAyuda();
    return;
  }
  if (nombre == "leer") {
    leerSensores();
    return;
  }
  if (nombre == "mostrar") {
    mostrarValores();
    return;
  }
  if (nombre == "freno") {
    freno();
    return;
  }
  if (nombre == "arranque") {
    if (isPaused) {
      isPaused = false;
      loopButtonState = 0;
      btPrintln("Arrancando modo carrera.");
    } else {
      btPrintln("Ya esta en modo carrera.");
    }
    return;
  }
  if (nombre == "ruedas") {
    ruedasPausaPrevia = isPaused;
    isPaused = true; // El control no interviene durante la limpieza
    ruedasInicio = millis();
    ruedasActivas = true;
    btPrintln("Ruedas al 80% durante 3 segundos...");
    return;
  }
  if (nombre == "calibrar") {
    if (mpuOK) {
      flagMpuCal = true;
      btPrintln("Calibrando MPU... ¡NO MOVER EL ROBOT!");
    } else {
      btPrintln("MPU no disponible.");
    }
    return;
  }
  if (nombre == "cero") {
    if (mpuOK) {
      flagMpuZero = true;
      btPrintln("Poniendo el angulo del MPU en 0...");
    } else {
      btPrintln("MPU no disponible.");
    }
    return;
  }
  if (nombre == "tilt") {
    btPrintln("Angulo: " + String(tiltKalman, 1) + "° (biasK=" + String(kalman.bias, 3) + ")");
    return;
  }
  if (nombre == "guardar") {
    if (!prefs.begin(NVS_NAMESPACE, false)) {
      btPrintln("Error: no se pudo abrir Preferences.");
      return;
    }
    prefs.putFloat("velBase", velBase);
    prefs.putFloat("kp", Kp_pos);
    prefs.putFloat("ki", Ki_pos);
    prefs.putFloat("kd", Kd_pos);
    prefs.putFloat("max", coefMax);
    prefs.putFloat("min", coefMin);
    prefs.putInt("ts", Ts);
    prefs.putInt("linea", linea);
    prefs.putFloat("umbral", umbral);
    prefs.putInt("lost", lostMaxMs);
    prefs.putFloat("kmpu", K_mpu);
    prefs.putFloat("velbaja", velBaja);
    prefs.putFloat("angfreno", anguloFreno);
    prefs.putInt("frenoms", (int)velBajaMs);
    prefs.end();
    btPrintln("Valores guardados en flash (se restauran al encender).");
    return;
  }

  // Comandos con valor
  if (valorStr.length() == 0) {
    btPrintln("Comando invalido: falta el valor. Escribir 'help' para ver los comandos.");
    return;
  }

  if (nombre == "kp") {
    Kp_pos = valor;
  } else if (nombre == "ki") {
    Ki_pos = valor;
  } else if (nombre == "kd") {
    Kd_pos = valor;
  } else if (nombre == "vel") {
    velBase = valor;
  } else if (nombre == "min") {
    coefMin = valor;
  } else if (nombre == "max") {
    coefMax = valor;
  } else if (nombre == "linea") {
    linea = (int)valor;
  } else if (nombre == "umbral") {
    umbral = (int)valor;
  } else if (nombre == "pos") {
    flagPos = (int)valor;
  } else if (nombre == "ts") {
    Ts = constrain((int)valor, 1, 1000);
  } else if (nombre == "kmpu") {
    K_mpu = valor;
  } else if (nombre == "velbaja") {
    velBaja = valor;
  } else if (nombre == "angfreno") {
    anguloFreno = constrain(valor, 0, 60);
  } else if (nombre == "frenoms") {
    velBajaMs = constrain((int)valor, 0, 60000);
  } else if (nombre == "mpudbg") {
    flagMpuDebug = (int)valor;
  } else if (nombre == "lost") {
    lostMaxMs = constrain((int)valor, 0, 60000);
  } else {
    btPrintln("Comando no reconocido. Escribir 'help' para ver los comandos.");
    return;
  }

  btPrintln(nombre + " cambiado a: " + valorStr);
}

void leerSensores() {
  String out = "";
  for (int i = 0; i < NUM_SENSORES; i++) {
    setMuxChannel(ordenCanales[i]);
    delayMicroseconds(50);
    out += String(analogRead(MUX_SIG));
    if (i < NUM_SENSORES - 1) out += " | ";
  }
  btPrintln(out);
}

void mostrarAyuda() {
  btPrintln("Comandos: kp ki kd vel min max linea umbral pos ts kmpu velbaja angfreno frenoms mpudbg lost");
  btPrintln("Acciones: leer mostrar tilt freno arranque ruedas calibrar cero guardar help");
  btPrintln("kp=" + String(Kp_pos) + " ki=" + String(Ki_pos) + " kd=" + String(Kd_pos) +
            " vel=" + String(velBase) + " min=" + String(coefMin) + " max=" + String(coefMax));
  btPrintln("linea=" + String(linea) + " umbral=" + String(umbral) + " ts=" + String(Ts) +
            " pos=" + String(flagPos) + " kmpu=" + String(K_mpu) +
            " velbaja=" + String(velBaja) + " angfreno=" + String(anguloFreno) +
            " frenoms=" + String((int)velBajaMs) +
            " lost=" + String(lostMaxMs));
}

void mostrarValores() {
  btPrintln("===== VALORES ACTUALES =====");
  btPrintln("Vel Base: " + String(velBase));
  btPrintln("Kp: " + String(Kp_pos));
  btPrintln("Kd: " + String(Kd_pos));
  btPrintln("Ki: " + String(Ki_pos));
  btPrintln("Coef max: " + String(coefMax));
  btPrintln("Coef min: " + String(coefMin));
  btPrintln("Ts: " + String(Ts));
  btPrintln("Linea: " + String(linea));
  btPrintln("Umbral: " + String(umbral));
  btPrintln("Kmpu: " + String(K_mpu));
  btPrintln("Vel baja: " + String(velBaja) + " | Ang freno: " + String(anguloFreno) + "° | " + String((int)velBajaMs) + "ms");
  btPrintln("FlagPos: " + String(flagPos));
  btPrintln("Lost ms: " + String(lostMaxMs));
  btPrintln("============================");
}
