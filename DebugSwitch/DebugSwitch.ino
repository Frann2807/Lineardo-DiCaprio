// ============================================================
// DEBUG SWITCH - Seguidor de Línea ESP32-C3 Super Mini
// Lee el botón (GPIO 4) con un while bloqueante.
// Se usa INPUT_PULLUP: presionado = LOW, suelto = HIGH.
// Si el comportamiento está invertido, cambiar los HIGH/LOW de
// los while o usar INPUT_PULLDOWN si el botón va a VCC.
// ============================================================

#define pinSwitch 5

// ===== PINES DE MOTORES (se dejan en 0 en este debug) =====
#define pinMizqA 9
#define pinMizqB 10
#define pinMderA 21
#define pinMderB 20

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

  pinMode(pinSwitch, INPUT_PULLUP);

  Serial.println("===== DEBUG SWITCH =====");
  Serial.println("Presionar el boton (GPIO 4) para probar...");
  Serial.println("NOTA: si la logica esta invertida, cambiar HIGH/LOW en los while.");
}

void loop() {
  while (digitalRead(pinSwitch) == HIGH) { delay(10); }  // esperar a que se presione
  delay(50);                                             // antirrebote
  Serial.println("Switch: ACTIVO (presionado)");

  while (digitalRead(pinSwitch) == LOW) { delay(10); }   // esperar a que se suelte
  delay(50);
  Serial.println("Switch: DESACTIVO (suelto)");
}
