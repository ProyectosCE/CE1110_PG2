// MOTOR_CONTROL.ino
// Plantilla de control de motor DC para Arduino (ej. UNO/Nano).
// Controla dirección y velocidad con un puente H (L298N, TB6612FNG, etc.).
// Comandos por Serial:
//   F <0-255>    -> Adelante a velocidad (PWM)
//   R <0-255>    -> Reversa a velocidad (PWM)
//   S            -> Stop
//   ?            -> Ayuda
// Cableado típico L298N:
//   IN1 -> DIR_A_PIN, IN2 -> DIR_B_PIN, ENA -> EN_PIN (PWM)
// Ajuste los pines según su hardware.

// --- Configuración de pines (ajuste según su placa) ---
const uint8_t EN_PIN     = 9;   // Pin PWM para velocidad (UNO: 3,5,6,9,10,11)
const uint8_t DIR_A_PIN  = 7;   // Dirección A
const uint8_t DIR_B_PIN  = 8;   // Dirección B

// --- Parámetros Serial ---
const unsigned long BAUDRATE = 9600;

// --- Estado ---
uint8_t currentSpeed = 0; // 0-255

void setup() {
  // Inicializar Serial
  Serial.begin(BAUDRATE);
  while (!Serial) { /* Espera Serial (en placas con USB nativo) */ }

  // Inicializar pines
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_A_PIN, OUTPUT);
  pinMode(DIR_B_PIN, OUTPUT);

  // Estado inicial: detenido
  stopMotor();

  Serial.println("Motor Control listo.");
  Serial.println("Comandos: F <0-255>, R <0-255>, S, ?");
}

void loop() {
  handleSerialCommands();
}

// --- Funciones de control ---
void forward(uint8_t speed) {
  digitalWrite(DIR_A_PIN, HIGH);
  digitalWrite(DIR_B_PIN, LOW);
  analogWrite(EN_PIN, speed);
  currentSpeed = speed;
  Serial.print("Adelante a "); Serial.println(speed);
}

void reverse(uint8_t speed) {
  digitalWrite(DIR_A_PIN, LOW);
  digitalWrite(DIR_B_PIN, HIGH);
  analogWrite(EN_PIN, speed);
  currentSpeed = speed;
  Serial.print("Reversa a "); Serial.println(speed);
}

void stopMotor() {
  analogWrite(EN_PIN, 0);
  digitalWrite(DIR_A_PIN, LOW);
  digitalWrite(DIR_B_PIN, LOW);
  currentSpeed = 0;
  Serial.println("Motor detenido");
}

// --- Parsing simple por Serial ---
void handleSerialCommands() {
  if (Serial.available() == 0) return;

  char cmd = Serial.read();
  if (cmd == '\n' || cmd == '\r' || cmd == ' ') return; // Ignorar separadores

  switch (cmd) {
    case 'F':
    case 'f': {
      int val = readIntFromSerial();
      uint8_t spd = clampPWM(val);
      forward(spd);
      break;
    }
    case 'R':
    case 'r': {
      int val = readIntFromSerial();
      uint8_t spd = clampPWM(val);
      reverse(spd);
      break;
    }
    case 'S':
    case 's': {
      stopMotor();
      break;
    }
    case '?': {
      printHelp();
      break;
    }
    default:
      Serial.print("Comando desconocido: "); Serial.println(cmd);
      printHelp();
      break;
  }
}

int readIntFromSerial() {
  // Leer números hasta encontrar no dígito
  String token = "";
  unsigned long start = millis();
  while (millis() - start < 300) { // Timeout simple 300 ms
    while (Serial.available()) {
      char c = Serial.peek();
      if (c == ' ' || c == '\r' || c == '\n') { Serial.read(); if (token.length() > 0) return token.toInt(); else continue; }
      if (isDigit(c)) { token += (char)Serial.read(); }
      else { Serial.read(); if (token.length() > 0) return token.toInt(); }
    }
  }
  return token.length() ? token.toInt() : 0;
}

uint8_t clampPWM(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

void printHelp() {
  Serial.println("\nAyuda:");
  Serial.println(" F <0-255>  -> Adelante a velocidad PWM");
  Serial.println(" R <0-255>  -> Reversa a velocidad PWM");
  Serial.println(" S          -> Stop");
  Serial.println(" ?          -> Mostrar esta ayuda\n");
  Serial.print("Pines: EN="); Serial.print(EN_PIN);
  Serial.print(", DIR_A="); Serial.print(DIR_A_PIN);
  Serial.print(", DIR_B="); Serial.println(DIR_B_PIN);
}
