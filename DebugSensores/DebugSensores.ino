// ============================================================
// DEBUG SENSORES - Seguidor de Línea ESP32-C3 Super Mini
// Lee los 8 sensores a través del multiplexor (CD4051/74HC4051)
// y calcula la posición por centro de masa ponderado.
//
// Pines del MUX:
//   S0 = GPIO0 | S1 = GPIO1 | S2 = GPIO2 | SIG = GPIO3
//
// Flags:
//   FlagAnalogRead     = 1 -> muestra lecturas crudas
//                             en orden físico (canal del MUX)
//   FlagLinea          = 1 -> línea blanca | 0 -> línea negra
//   FlagDebugPosition  = 1 -> muestra posición y sensores activos
//
// NOTA: en carreraBT la posición se calcula con interpolación
// cuadrática (polyfit); acá se usa centro de masa para depurar
// lecturas y umbral más fácilmente.
// ============================================================

// ===== PINES DEL MULTIPLEXOR =====
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_SIG 3

// ===== PINES DE MOTORES (se dejan en 0 en este debug) =====
#define pinMizqA 9
#define pinMizqB 10
#define pinMderA 21
#define pinMderB 20

// ===== SENSORES =====
const int NUM_SENSORES = 8;

// Orden físico de los sensores (canales del MUX leídos de izq a der)
int ordenCanales[NUM_SENSORES] = { 3, 0, 1, 2, 4, 6, 7, 5 };

// ===== FLAGS =====
int FlagAnalogRead = 0;     // 1 = mostrar lecturas crudas
int FlagLinea = 0;          // 1 = línea blanca, 0 = línea negra
int FlagDebugPosition = 1;  // 1 = mostrar posición

// Umbral de detección (0 - 4095)
int umbral = 2000;

float VS[NUM_SENSORES] = {0};
float PosS[NUM_SENSORES] = {1, 2, 3, 4, 5, 6, 7, 8};

float posicion;
int cantSens = 0;

void setMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
}

void setup() {
  Serial.begin(115200);

  // Motores en 0 para que no se activen durante el debug
  pinMode(pinMizqA, OUTPUT);
  pinMode(pinMizqB, OUTPUT);
  pinMode(pinMderA, OUTPUT);
  pinMode(pinMderB, OUTPUT);
  digitalWrite(pinMizqA, LOW);
  digitalWrite(pinMizqB, LOW);
  digitalWrite(pinMderA, LOW);
  digitalWrite(pinMderB, LOW);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  Serial.println("===== DEBUG SENSORES =====");
  Serial.print("FlagAnalogRead: "); Serial.println(FlagAnalogRead);
  Serial.print("FlagLinea: "); Serial.println(FlagLinea);
  Serial.print("FlagDebugPosition: "); Serial.println(FlagDebugPosition);
  Serial.print("Umbral: "); Serial.println(umbral);

  Serial.print("Orden de lectura (canales): ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("C");
    Serial.print(ordenCanales[i]);
    if (i < NUM_SENSORES - 1) Serial.print(" | ");
  }
  Serial.println();
}

void loop() {
  // 1) Lectura cruda de los sensores (a través del MUX)
  for (int i = 0; i < NUM_SENSORES; i++) {
    setMuxChannel(ordenCanales[i]);
    delayMicroseconds(50);  // tiempo de asentamiento del MUX
    VS[i] = analogRead(MUX_SIG);
  }

  if (FlagAnalogRead == 1) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print((int)VS[i]);
      if (i < NUM_SENSORES - 1) Serial.print(" | ");
    }
    Serial.println();
  }

  // 2) Inversión según el color de línea
  float v[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (FlagLinea == 0) v[i] = 4095 - VS[i];  // línea negra: invertir
    else v[i] = VS[i];                        // línea blanca: sin cambio
  }

  // 3) Posición por centro de masa ponderado
  // El peso de cada sensor es lo que supera el umbral, para que la
  // posición se desplace de forma continua (sin saltos al cruzar el umbral).
  float sumaPonderada = 0;
  float sumaValores = 0;
  cantSens = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (v[i] > umbral) {
      float w = v[i] - umbral;
      sumaPonderada += w * PosS[i];
      sumaValores += w;
      cantSens++;
    }
  }
  if (cantSens > 0) posicion = sumaPonderada / sumaValores;
  else posicion = -1;  // sin sensores sobre la línea

  if (FlagDebugPosition == 1) {
    Serial.print("Posicion: ");
    Serial.print(posicion, 4);
    Serial.print(" | Sensores detectando: ");
    Serial.println(cantSens);
  }

  delay(100);
}
