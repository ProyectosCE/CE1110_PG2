#include <Arduino.h>
#include <ctype.h>

// ============================================================================
// CONTROL PID DE MOTOR DC GM25-370 CON ENCODER Y TB6612FNG
// ============================================================================
// ESPECIFICACIONES DEL MOTOR (Reducción 1:10):
//   - Voltaje nominal: 6 VDC
//   - Velocidad sin carga: 280 RPM
//   - Velocidad nominal (con carga): ~230 RPM
//   - Resolución encoder: 224.4 PPR
//   - Torque nominal: 0.12 kg·cm
//
// IMPORTANTE: 
//   El motor tiene un LÍMITE FÍSICO de ~280 RPM sin carga.
//   Con carga (fricción del reductor, inercia, etc.) el máximo es ~240 RPM.
//   Si se pide VEL 280, el motor NO puede alcanzarla - saturará en ~220 RPM.
//   Usar comandos realistas: VEL 20 a VEL 240
// ============================================================================

// Pins para Arduino UNO + TB6612FNG
#define ENCA 2  // Pin de interrupción (2 o 3 en UNO)
#define ENCB 3  // Pin digital cualquiera
#define PWM 5   // Pin PWM (3,5,6,9,10,11 en UNO)
#define IN1 6   // Pin digital
#define IN2 7   // Pin digital
#define STBY 8  // Pin digital

// ========================= PINES PARA SENSORES Y CONTROLES =========================
// Disponibles: 3, 9, 10, 11, 12, 13 y A0-A5
// Asignación:
// NOTA: La velocidad ya NO se toma del potenciómetro; se define por Serial con "VEL X"
#define PIN_BTN_ABRIR A4    // Botón ABRIR (activo en alto)
#define PIN_BTN_CERRAR A5   // Botón CERRAR (activo en alto)
#define PIN_SWITCH_PID 11   // Switch PID (HIGH=PID ON, LOW=PID OFF)

// HC-SR04 sensores de distancia
#define PIN_TRIG_OPEN 9     // HC-SR04 #1 (detener al abrir)
#define PIN_ECHO_OPEN 10
#define PIN_TRIG_CLOSE 12   // HC-SR04 #2 (detener al cerrar)  
#define PIN_ECHO_CLOSE 13

// Especificaciones del encoder del GM25-370
// Hall resolution: 224.4 PPR
const float PPR = 224.4f;        // Pulses Per Revolution
const float pulsesPerRev = PPR;  // alias

// Muestreo y tiempo
// Ventana de muestreo ampliada para mejorar lecturas a bajas RPM
const unsigned long samplePeriodMicros = 30000UL; // 30 ms
unsigned long prevMicros = 0;
unsigned long nowMicros = 0;
float deltaT = 0.01f; // 10 ms en segundos

// Encoder y velocidad (patrón ejemplo_youtube)
volatile int pos_i = 0;           // posición de ticks
volatile float velocity_i = 0.0f; // cuentas/seg (método 2)
volatile unsigned long prevT_i = 0;

int posPrev = 0;                  // para método 1 en loop
float v1Filt = 0.0f, v1Prev = 0.0f;
float v2Filt = 0.0f, v2Prev = 0.0f;

// Filtro pasa-bajas discreto (25 Hz aprox)
const float a1 = 0.854f;
const float b0 = 0.0728f;
const float b1 = 0.0728f;

// Setpoint (RPM). Se puede ajustar por Serial o potenciómetro si deseas.
float setpointRPM = 100.0f; // objetivo inicial

// PID (basado en ejemplo_medium.ino, adaptado a UNO)
// Ganancias ajustadas para el motor GM25-370 (reducción 1:10)
// El motor tiene límite físico de 280 RPM sin carga, ~240 RPM con carga
float kp = 6.0f;   // Reducido para evitar saturación en altas velocidades
float ki = 12.0f;  // Reducido para que no sature cuando llega al límite físico
float kd = 0.2f;   // Aumentado ligeramente para mejor amortiguamiento

float ek = 0.0f, ek1 = 0.0f; // error actual y previo
float integral = 0.0f;
float derivada = 0.0f;
float uk = 0.0f; // salida de control (PWM con signo)

// Actuación y ayudas
const int PWM_MAX = 255;
const int PWM_MIN_EFFECTIVE = 30; // mínimo para vencer fricción
const float RPM_MIN_EFFECTIVE = 35.0f; // mínimo soportado en modo PID
const float deadbandRPM = 1.5f;   // banda muerta reducida para mejor precisión

// Feedforward con mapeo no lineal (compensación de fricción)
// MOTOR GM25-370 con reducción 1:10
// Velocidad sin carga: 280 RPM @ 6V (especificación nominal)
// Velocidad con carga máxima realista: ~240 RPM @ PWM=255
// 
// CALIBRACIÓN DEL FEEDFORWARD basada en la física del motor:
// - El motor tiene límite físico de ~280 RPM sin carga
// - Con carga (fricción del reductor + inercia), máximo realista: 240 RPM
// - Velocidades bajas (0-80 RPM): kff = 1.8 (compensa fricción estática alta)
// - Velocidades medias (80-180 RPM): kff = 1.3 (zona lineal)
// - Velocidades altas (180-240 RPM): kff = 1.5 (compensa pérdidas del reductor)
// - Offset base: 25 PWM para vencer fricción de Coulomb
//
// Ajuste fino: Si persiste error, revisar voltaje de alimentación (debe ser 6V estable)
const float kff_low = 1.8f;   // Ganancia para velocidades bajas
const float kff_mid = 1.3f;   // Ganancia para velocidades medias
const float kff_high = 1.5f;  // Ganancia para velocidades altas (cerca del límite)
const float rpm_transition_low = 80.0f;  // Transición baja->media
const float rpm_transition_high = 180.0f; // Transición media->alta
const int pwmOffset = 25;     // offset base para vencer fricción estática

// Dirección actual (true = adelante)
bool forward = true;

// Buffer y estado para parser de Serial (no bloqueante)
static char cmdBuf[32];
static uint8_t cmdIdx = 0;

// Modo de control
bool manualMode = false;     // true = controla por PWM puro (sin PID)
bool invertDirPins = false;  // invierte IN1/IN2 por software si el cableado quedó invertido

// ========================= VARIABLES PARA SENSORES =========================
// Debounce y detección de flancos
uint8_t lastBtnAbrir = 0;
uint8_t lastBtnCerrar = 0;
uint8_t lastSwitchPid = 0;

// Debounce individual para cada botón
unsigned long lastBtnAbrirTime = 0;
unsigned long lastBtnCerrarTime = 0;
unsigned long lastSwitchTime = 0;
const unsigned long debounceIntervalMs = 50;

// Estado de movimiento para gestión de parada por distancia
enum MoveState { IDLE, OPENING, CLOSING };
MoveState moveState = IDLE;

// Movimiento controlado solo por sensores de distancia (sin temporizador)
bool motorMoving = false;

// Desaceleración suave al detectar obstáculo
bool decelerating = false;
const float STOP_EPS_RPM = 3.0f;      // umbral para considerar velocidad ~0
const int PWM_DEC_STEP = 10;          // paso de reducción de PWM por ciclo de control en modo manual

// Calibración de distancia de vacío (sin objeto) para cada sensor
float baselineOpenCm = -1.0f;
float baselineCloseCm = -1.0f;
const int BASELINE_SAMPLES = 10;      // muestras para promediar
const float DETECT_MARGIN_CM = 3.0f;  // margen: si se mide < (baseline - margen) => objeto

// Umbrales de distancia (cm) para parar - AJUSTAR según tu aplicación
float stopOpenCm = 15.0f;   // si al ABRIR la distancia <= 15 cm, detener
float stopCloseCm = 15.0f;  // si al CERRAR la distancia <= 15 cm, detener

// Rango de RPM permitido (controlado por comandos Serial VEL X)
// Referencia: usar entre 35 y 240 RPM (límite físico con carga)

// Magnitud de velocidad objetivo para comandos PID
// LÍMITE SUPERIOR basado en especificaciones del motor:
// - Velocidad sin carga: 280 RPM (especificación)
// - Velocidad con carga máxima realista: ~240 RPM
float targetSpeedMagRPM = 100.0f; // restringida a [35, 240]

int manualDir = 1;           // +1 adelante, -1 reversa (solo en modo manual)
int manualPWM = 128;         // Valor de PWM (0-255) para el modo manual

// Seguimiento del último PWM aplicado (para transferencias sin golpes)
int lastAppliedPWM = 0;      // magnitud 0..255
int lastAppliedSign = 1;     // +1 adelante, -1 reversa

// ISR del encoder (patrón ejemplo_youtube): lee ENCB cuando ENCA sube
void onEncoderA() {
  // Leer canal B para decidir sentido
  int b = digitalRead(ENCB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;

  // Método 2: velocidad por tiempo entre flancos (cuentas/seg)
  unsigned long currT = micros();
  float dt = (float)(currT - prevT_i) / 1.0e6f;
  if (dt > 0) {
	velocity_i = (float)increment / dt;
  }
  prevT_i = currT;
}

// Control de motor usando TB6612FNG
// ukSigned: -255..255 (negativo = reversa)
void setMotor(int ukSigned) {
	// standby habilitado
	digitalWrite(STBY, HIGH);

	// dirección
	if (ukSigned >= 0) {
		forward = true;
		if (!invertDirPins) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
		else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
	} else {
		forward = false;
		if (!invertDirPins) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
		else { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
	}

	int duty = abs(ukSigned);
	// Solo aplicar mínimo efectivo en modo PID, no en modo manual
	if (!manualMode && duty > 0 && duty < PWM_MIN_EFFECTIVE) {
		duty = PWM_MIN_EFFECTIVE; // evita zona de no respuesta en PID
	}
	duty = constrain(duty, 0, PWM_MAX);
	analogWrite(PWM, duty);
}

// ========================= FUNCIONES PARA SENSORES =========================

// Mide distancia (cm) con un HC-SR04
float measureDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout de 30ms (~5 metros máx)
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1.0f; // Error de lectura

  // Conversión: distancia = (tiempo_us / 2) * 0.0343
  return (duration * 0.5f) * 0.0343f;
}

// Promedia múltiples lecturas válidas de un sensor (ignora lecturas inválidas)
float averageDistanceCm(uint8_t trigPin, uint8_t echoPin, int samples) {
	float acc = 0.0f;
	int count = 0;
	for (int i = 0; i < samples; ++i) {
		float d = measureDistanceCm(trigPin, echoPin);
		if (d > 0) { acc += d; count++; }
		delay(10);
	}
	if (count == 0) return -1.0f;
	return acc / count;
}

// Calibra las distancias base de "vacío" (sin objeto) para ambos sensores
void calibrateBaselines() {
	Serial.println("Calibrando distancias base (sin objeto)...");
	baselineOpenCm = averageDistanceCm(PIN_TRIG_OPEN, PIN_ECHO_OPEN, BASELINE_SAMPLES);
	baselineCloseCm = averageDistanceCm(PIN_TRIG_CLOSE, PIN_ECHO_CLOSE, BASELINE_SAMPLES);
	if (baselineOpenCm > 0) {
		Serial.print("Baseline OPEN: "); Serial.print(baselineOpenCm); Serial.println(" cm");
	} else {
		Serial.println("Baseline OPEN: no válida, usando umbral fijo");
	}
	if (baselineCloseCm > 0) {
		Serial.print("Baseline CLOSE: "); Serial.print(baselineCloseCm); Serial.println(" cm");
	} else {
		Serial.println("Baseline CLOSE: no válida, usando umbral fijo");
	}
}

// Procesa botones y switch (sin potenciómetro)
void processSensors() {
  unsigned long now = millis();
  
  // Debounce individual para botón ABRIR
  uint8_t bAbrir = digitalRead(PIN_BTN_ABRIR);
  if (now - lastBtnAbrirTime >= debounceIntervalMs) {
    if (bAbrir == HIGH && lastBtnAbrir == LOW) {
      // Presión válida del botón ABRIR
      startMovement(OPENING);
      lastBtnAbrirTime = now;
    }
    lastBtnAbrir = bAbrir;
  }

  // Debounce individual para botón CERRAR  
  uint8_t bCerrar = digitalRead(PIN_BTN_CERRAR);
  if (now - lastBtnCerrarTime >= debounceIntervalMs) {
    if (bCerrar == HIGH && lastBtnCerrar == LOW) {
      // Presión válida del botón CERRAR
      startMovement(CLOSING);
      lastBtnCerrarTime = now;
    }
    lastBtnCerrar = bCerrar;
  }

	// Debounce para switch PID
  uint8_t sPid = digitalRead(PIN_SWITCH_PID);
  if (now - lastSwitchTime >= debounceIntervalMs) {
    if (sPid != lastSwitchPid) {
      if (sPid == HIGH) activar();
      else desactivar();
      lastSwitchPid = sPid;
      lastSwitchTime = now;
    }
  }
}

// Inicia movimiento (detención controlada por sensores de distancia)
void startMovement(MoveState direction) {
  moveState = direction;
  motorMoving = true;
	decelerating = false; // reiniciar cualquier desaceleración previa
  
  if (direction == OPENING) {
    abrir();
		Serial.println("Iniciando APERTURA");
  } else if (direction == CLOSING) {
    cerrar();
		Serial.println("Iniciando CIERRE");
  }
}

// Detiene el movimiento
void stopMovement() {
  motorMoving = false;
  moveState = IDLE;
	// Parada: setpoint 0 y PWM manual 0; el lazo pondrá uk=0 cuando motorMoving=false
	setpointRPM = 0.0f;
	manualPWM = 0;
	decelerating = false;
  Serial.println("Movimiento detenido");
}

// Controla la duración del movimiento
void processMovementTimer() {
	if (!motorMoving) return;

	// Verificar límites de distancia solo si el motor está en movimiento
	processDistanceStop();

	// Procesar desaceleración suave (si fue activada por los sensores)
	// En PID: setpoint a 0 y esperar a que la velocidad caiga por debajo del umbral
	// En Manual: reducir PWM gradualmente hasta 0
	if (decelerating) {
		if (manualMode) {
			if (manualPWM > 0) {
				manualPWM = max(0, manualPWM - PWM_DEC_STEP);
			}
			// Considerar detenido cuando PWM es 0 y la velocidad medida es baja
			if (manualPWM == 0 && fabs(v1Filt) <= STOP_EPS_RPM) {
				stopMovement();
			}
		} else {
			// PID: ya se ha fijado setpointRPM = 0. Esperar a que la velocidad caiga
			if (fabs(v1Filt) <= STOP_EPS_RPM) {
				stopMovement();
			}
		}
	}
}

// Gestiona parada por distancia según el estado actual
void processDistanceStop() {
  if (!motorMoving) return; // Solo verificar si el motor se está moviendo
  
  if (moveState == OPENING) {
    float dist = measureDistanceCm(PIN_TRIG_OPEN, PIN_ECHO_OPEN);
		// Umbral dinámico: si hay baseline válida, usar baseline - margen; si no, usar stopOpenCm
		float threshold = (baselineOpenCm > 0) ? (baselineOpenCm - DETECT_MARGIN_CM) : stopOpenCm;
		if (dist > 0 && dist <= threshold) {
			// Iniciar desaceleración a 0
			decelerating = true;
			if (!manualMode) {
				setpointRPM = 0.0f; // PID decelera con control activo
			}
			Serial.println("Desacelerando: Límite de apertura alcanzado");
		}
  } 
  else if (moveState == CLOSING) {
    float dist = measureDistanceCm(PIN_TRIG_CLOSE, PIN_ECHO_CLOSE);
		float threshold = (baselineCloseCm > 0) ? (baselineCloseCm - DETECT_MARGIN_CM) : stopCloseCm;
		if (dist > 0 && dist <= threshold) {
			// Iniciar desaceleración a 0
			decelerating = true;
			if (!manualMode) {
				setpointRPM = 0.0f; // PID decelera con control activo
			}
			Serial.println("Desacelerando: Límite de cierre alcanzado");
		}
  }
}

// --- Comandos de alto nivel ---
// La variable targetSpeedMagRPM ya está declarada arriba en las variables globales

// ------------------ Utilidades de mapeo RPM <-> PWM ------------------
// Cálculo de kff en función de la magnitud de RPM objetivo
float kffForRPM(float absRPM){
	float kff_actual;
	if (absRPM <= rpm_transition_low){
		kff_actual = kff_low;
	} else if (absRPM <= rpm_transition_high){
		float ratio = (absRPM - rpm_transition_low) / (rpm_transition_high - rpm_transition_low);
		ratio = constrain(ratio, 0.0f, 1.0f);
		kff_actual = kff_low + (kff_mid - kff_low) * ratio;
	} else {
		float ratio = (absRPM - rpm_transition_high) / (240.0f - rpm_transition_high);
		ratio = constrain(ratio, 0.0f, 1.0f);
		kff_actual = kff_mid + (kff_high - kff_mid) * ratio;
	}
	return kff_actual;
}

// Mapea RPM (con signo) a un PWM (con signo) usando el feedforward no lineal
int pwmFromRPMSigned(float rpmSigned){
	float absSetpoint = fabs(rpmSigned);
	int sign = (rpmSigned >= 0.0f) ? +1 : -1;
	float kff_actual = kffForRPM(absSetpoint);
	float u_ff = kff_actual * rpmSigned; // incluye signo
	if (absSetpoint > 0.1f){
		if (u_ff >= 0) u_ff += pwmOffset; else u_ff -= pwmOffset;
	}
	int pwmCmd = (int)round(constrain(u_ff, -PWM_MAX, PWM_MAX));
	// Asegurar mínimo efectivo cuando no es cero
	if (pwmCmd != 0 && abs(pwmCmd) < PWM_MIN_EFFECTIVE) pwmCmd = sign * PWM_MIN_EFFECTIVE;
	return pwmCmd;
}

// Estima RPM a partir de PWM (magnitud) usando un pequeño refinamiento iterativo
float estimateRPMFromPWM(int pwmMag){
	if (pwmMag <= 0) return 0.0f;
	int eff = max(0, pwmMag - pwmOffset);
	// Valor inicial con kff medio
	float rpm = (float)eff / max(0.001f, kff_mid);
	// 2 iteraciones de corrección usando kff calculado a la nueva RPM
	for (int i=0; i<2; ++i){
		float k = kffForRPM(fabs(rpm));
		rpm = (float)eff / max(0.001f, k);
	}
	// Limitar al rango plausible
	rpm = constrain(rpm, 0.0f, 240.0f);
	return rpm;
}

// abre (gira sentido positivo) - respeta el modo actual (PID o manual)
void abrir() {
	if (!motorMoving) return; // Solo permitir si el motor debe moverse
	
	// NO cambiar manualMode - respetar modo actual según switch
	manualDir = +1; // Dirección para modo manual
	
	// Configurar setpoint/PWM para ambos modos
	setpointRPM = targetSpeedMagRPM; // sentido positivo para PID
	manualPWM = abs(pwmFromRPMSigned(setpointRPM)); // PWM para manual
}

// cierra (gira sentido negativo) - respeta el modo actual (PID o manual)
void cerrar() {
	if (!motorMoving) return; // Solo permitir si el motor debe moverse
	
	// NO cambiar manualMode - respetar modo actual según switch
	manualDir = -1; // Dirección para modo manual
	
	// Configurar setpoint/PWM para ambos modos  
	setpointRPM = -targetSpeedMagRPM; // sentido negativo para PID
	manualPWM = abs(pwmFromRPMSigned(setpointRPM)); // PWM para manual
}

// configura la velocidad (magnitud) entre 20 y 240 RPM para el modo PID
void velocidad(float rpm) {
	if (rpm < RPM_MIN_EFFECTIVE) rpm = RPM_MIN_EFFECTIVE;   // Límite inferior
	if (rpm > 240.0f) rpm = 240.0f; // Límite superior ajustado a capacidad real bajo carga
	targetSpeedMagRPM = rpm;
	// Actualizar SIEMPRE ambas representaciones para transferencias sin golpes
	// Mantener el sentido actual (en PID según setpoint actual, en manual según manualDir)
	int sign = 1;
	if (manualMode) sign = (manualDir >= 0) ? +1 : -1; else sign = (setpointRPM >= 0.0f) ? +1 : -1;
	setpointRPM = sign * targetSpeedMagRPM;
	manualPWM = abs(pwmFromRPMSigned(setpointRPM));
}

// activar/desactivar PID
void activar(){
	// Activa control PID con realimentación
	manualMode = false;
	// Evitar golpe al activar PID: el setpoint se iguala a la velocidad actual
		// Respetar mínimo efectivo si la velocidad es muy baja
		if (fabs(v1Filt) < RPM_MIN_EFFECTIVE && fabs(v1Filt) > 0.0f){
			setpointRPM = (v1Filt >= 0.0f) ? RPM_MIN_EFFECTIVE : -RPM_MIN_EFFECTIVE;
		} else {
			setpointRPM = v1Filt;
		}
	integral = 0.0f;
	ek1 = 0.0f;
	// También sincronizar la representación manual a lo aplicado hasta ahora
	manualDir = (lastAppliedSign >= 0) ? +1 : -1;
	manualPWM = lastAppliedPWM;
}

void desactivar(){
	// Desactiva PID: control por PWM puro, sin ajustes en tiempo real
	manualMode = true;
	// Transferencia sin golpes: aplicar el PWM equivalente al último valor aplicado
	manualDir = (uk >= 0.0f) ? +1 : -1;
	manualPWM = abs((int)round(constrain(uk, -PWM_MAX, PWM_MAX)));
}

// utilidades de parser
void toUpperInPlace(char* s){ for(char* p=s; *p; ++p){ *p = (char)toupper(*p);} }

void parseCommand(char* line){
	char up[32];
	strncpy(up, line, sizeof(up)-1);
	up[sizeof(up)-1] = '\0';
	toUpperInPlace(up);

	// S<number> -> Fija setpoint para modo PID
	if (up[0] == 'S'){
		float val = atof(line+1);
				// Aplicar mínimo efectivo si no es cero
				if (fabs(val) > 0.0f && fabs(val) < RPM_MIN_EFFECTIVE){
					setpointRPM = (val >= 0.0f) ? RPM_MIN_EFFECTIVE : -RPM_MIN_EFFECTIVE;
				} else {
					setpointRPM = val;
				}
		targetSpeedMagRPM = fabs(val);
		// Actualizar también la representación manual
		manualDir = (setpointRPM >= 0.0f) ? +1 : -1;
		manualPWM = abs(pwmFromRPMSigned(setpointRPM));
		manualMode = false; // Cambia a modo PID
		return;
	}
	// PWM<number> -> Fija PWM para modo Manual
	if (strncmp(up, "PWM", 3) == 0){
		int val = atoi(line+3);
	manualPWM = constrain(val, 0, 255); // siempre positivo; dirección se define por manualDir
		// Sincronizar también el setpoint estimando la RPM equivalente
		{
			float rpmEst = estimateRPMFromPWM(manualPWM);
			setpointRPM = ((manualDir >= 0) ? +1.0f : -1.0f) * rpmEst;
			targetSpeedMagRPM = fabs(setpointRPM);
		}
		manualMode = true; // Cambia a modo Manual
		return;
	}
	// ABRIR / CERRAR -> Modo Manual con dirección
		if (strncmp(up, "ABRIR", 5) == 0){ abrir(); return; }
		if (strncmp(up, "CERRAR", 6) == 0){ cerrar(); return; }
	// VEL <num> / VELOCIDAD <num> -> Fija RPM para modo PID
		if (strncmp(up, "VEL ", 4) == 0){
		float v = atof(line+4);
			velocidad(v);
		return;
	}
		if (strncmp(up, "VELOCIDAD ", 10) == 0){
		float v = atof(line+10);
			velocidad(v);
		return;
	}

		// PID -> vuelve a control PID (automático)
		if (strncmp(up, "PID", 3) == 0){ activar(); return; }

					// ACTIVAR / DESACTIVAR PID
					if (strncmp(up, "ACTIVAR", 7) == 0){ activar(); return; }
					if (strncmp(up, "DESACTIVAR", 10) == 0){ desactivar(); return; }

		// STOP -> frena (coast) y pone todo a 0
		if (strncmp(up, "STOP", 4) == 0){
			stopMovement(); // Usar la nueva función para detener
			return;
		}

		// DIRINV -> alterna inversión de pines IN1/IN2
		if (strncmp(up, "DIRINV", 6) == 0){ invertDirPins = !invertDirPins; return; }

		// CALIBRAR -> recalibra distancias base (sin objeto)
		if (strncmp(up, "CALIBRAR", 9) == 0){ calibrateBaselines(); return; }
}

void setup() {
	Serial.begin(115200);

	// Pines motor y encoder
	pinMode(ENCA, INPUT_PULLUP);
	pinMode(ENCB, INPUT_PULLUP);
	pinMode(PWM, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(STBY, OUTPUT);

	// ========================= CONFIGURAR PINES SENSORES =========================
	// Botones (activos en alto - usar resistencias pull-down externas)
	pinMode(PIN_BTN_ABRIR, INPUT);
	pinMode(PIN_BTN_CERRAR, INPUT);
    
	// Switch PID
	pinMode(PIN_SWITCH_PID, INPUT);
	
	// HC-SR04 sensores
	pinMode(PIN_TRIG_OPEN, OUTPUT);
	pinMode(PIN_ECHO_OPEN, INPUT);
	pinMode(PIN_TRIG_CLOSE, OUTPUT);
	pinMode(PIN_ECHO_CLOSE, INPUT);
	
	// Inicializar HC-SR04
	digitalWrite(PIN_TRIG_OPEN, LOW);
	digitalWrite(PIN_TRIG_CLOSE, LOW);

	// Calibrar baselines de distancia al inicio (sin objeto presente)
	calibrateBaselines();
	
	// Lecturas iniciales para evitar flancos falsos
	lastBtnAbrir = digitalRead(PIN_BTN_ABRIR);
	lastBtnCerrar = digitalRead(PIN_BTN_CERRAR);
	lastSwitchPid = digitalRead(PIN_SWITCH_PID);
	
	// Inicializar tiempos de debounce
	lastBtnAbrirTime = millis();
	lastBtnCerrarTime = millis();
	lastSwitchTime = millis();
	
	// Configurar PID según posición inicial del switch
	if (lastSwitchPid == HIGH) {
		activar();
	} else {
		desactivar();
	}

	// Motor inicialmente detenido
	motorMoving = false;
	moveState = IDLE;

	digitalWrite(STBY, HIGH); // habilitar driver
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	analogWrite(PWM, 0);

		// Interrupción en flanco ascendente de ENCA
		attachInterrupt(digitalPinToInterrupt(ENCA), onEncoderA, RISING);

	prevMicros = micros();
}

void loop() {
	nowMicros = micros();
	unsigned long elapsed = nowMicros - prevMicros;

	if (elapsed >= samplePeriodMicros) {
		// tiempo real (s)
		deltaT = (float)elapsed / 1e6f;

		// Copiar posicion y velocidad (método 2) de forma atómica
		int pos;
		float velocity2;
		noInterrupts();
		pos = pos_i;
		velocity2 = velocity_i;
		interrupts();

		// Método 1: velocidad por diferencia de posición
		float velocity1;
		velocity1 = (pos - posPrev) / deltaT; // cuentas/seg
		posPrev = pos;

		// Convertir cuentas/seg a RPM: cuentas por revolución = PPR
		float v1 = (velocity1 / pulsesPerRev) * 60.0f;
		float v2 = (velocity2 / pulsesPerRev) * 60.0f;

		// Filtrado (25 Hz)
		v1Filt = a1 * v1Filt + b0 * v1 + b1 * v1Prev; v1Prev = v1;
		v2Filt = a1 * v2Filt + b0 * v2 + b1 * v2Prev; v2Prev = v2;

		// Control
		if (manualMode) {
			// Modo Manual: PWM fijo SIN RETROALIMENTACIÓN
			// El motor gira al PWM indicado, sin correcciones
			// NUEVO: Solo aplicar PWM si el motor debe moverse
			if (motorMoving) {
				uk = (manualDir >= 0) ? manualPWM : -manualPWM;
			} else {
				uk = 0; // Motor detenido cuando no hay movimiento activo
			}
			// Resetear variables PID para evitar acumulación
			integral = 0.0f;
			ek1 = 0.0f;
		} else {
			// Modo PID: lazo cerrado CON RETROALIMENTACIÓN
			// El sistema ajusta automáticamente el PWM para alcanzar el setpoint
			// NUEVO: Solo operar si el motor debe moverse
			if (motorMoving) {
				ek = setpointRPM - v1Filt;
				
				// Banda muerta solo cuando setpoint y velocidad están cerca de cero
				if (fabs(v1Filt) < deadbandRPM && fabs(setpointRPM) < deadbandRPM) {
					ek = 0.0f;
				}
				
				derivada = (ek - ek1) / deltaT;
				integral += ((ek + ek1) * 0.5f) * deltaT;
				
				// Anti-windup mejorado
				integral = constrain(integral, -200.0f, 200.0f);
				ek1 = ek;

				// Feedforward NO LINEAL para compensar fricción en todo el rango
				// Usa interpolación en 3 zonas para modelar mejor el comportamiento del motor
				float absSetpoint = fabs(setpointRPM);
				float kff_actual;
				
				if (absSetpoint <= rpm_transition_low) {
					// Zona baja (0-80 RPM): fricción estática dominante
					kff_actual = kff_low;
				} else if (absSetpoint <= rpm_transition_high) {
					// Zona media (80-180 RPM): comportamiento lineal
					float ratio = (absSetpoint - rpm_transition_low) / (rpm_transition_high - rpm_transition_low);
					ratio = constrain(ratio, 0.0f, 1.0f);
					kff_actual = kff_low + (kff_mid - kff_low) * ratio;
				} else {
					// Zona alta (180-240 RPM): pérdidas del reductor aumentan
					float ratio = (absSetpoint - rpm_transition_high) / (240.0f - rpm_transition_high);
					ratio = constrain(ratio, 0.0f, 1.0f);
					kff_actual = kff_mid + (kff_high - kff_mid) * ratio;
				}
				
				float u_ff = kff_actual * setpointRPM;
				
				// Offset adicional para vencer fricción estática
				if (fabs(setpointRPM) > 0.1f) {
					if (u_ff >= 0) u_ff += pwmOffset; 
					else u_ff -= pwmOffset;
				}
				
							// Salida PID con feedforward no lineal
							uk = kp * ek + ki * integral + kd * derivada + u_ff;

							// Política de "sin frenado en reversa" para grandes reducciones de velocidad
							// Si queremos 40 RPM y estamos en ~220 RPM en el mismo sentido, NO aplicar uk opuesto (reversa),
							// en su lugar hacer coast (uk=0) hasta acercarnos al setpoint.
							int desiredSign = (setpointRPM > 0.0f) ? +1 : (setpointRPM < 0.0f ? -1 : 0);
							float margin = 5.0f; // histéresis pequeña
							if (desiredSign != 0) {
								// misma dirección actual?
								int currentSign = (v1Filt > 0.0f) ? +1 : (v1Filt < 0.0f ? -1 : 0);
								if (currentSign == desiredSign) {
									// Si la magnitud actual es mucho mayor que el objetivo, prohibir frenado inverso
									if (desiredSign > 0 && v1Filt > setpointRPM + margin && uk < 0.0f) {
										// coast: salida a 0, detener integración para evitar windup
										uk = 0.0f;
										// anti-windup adicional cuando coasteamos
										integral = constrain(integral, -200.0f, 200.0f);
									}
									if (desiredSign < 0 && v1Filt < setpointRPM - margin && uk > 0.0f) {
										uk = 0.0f;
										integral = constrain(integral, -200.0f, 200.0f);
									}
								}
							}

							// En bajas velocidades, forzar sentido coherente con el setpoint
							// y evitar reversa que "jode" el sistema a bajas RPM.
							if (fabs(setpointRPM) > 0.0f && fabs(setpointRPM) <= RPM_MIN_EFFECTIVE){
								int desiredSign = (setpointRPM >= 0.0f) ? +1 : -1;
								// Evitar invertir el sentido: si uk va en sentido opuesto, amortiguar a cero
								if ((uk >= 0.0f ? +1 : -1) != desiredSign){
									uk = 0.0f; // sin reversa; el feedforward ya aportará el mínimo necesario
								}
								// Asegurar mínimo efectivo de PWM si la salida es muy pequeña
								float ukAbs = fabs(uk);
								if (ukAbs > 0.0f && ukAbs < PWM_MIN_EFFECTIVE){
									uk = desiredSign * PWM_MIN_EFFECTIVE;
								}
							}

							// Limitar salida y advertir si está saturado
							uk = constrain(uk, -PWM_MAX, PWM_MAX);
			} else {
				// Motor detenido cuando no hay movimiento activo
				uk = 0;
				// Resetear variables PID cuando está parado
				integral = 0.0f;
				ek1 = 0.0f;
			}
		}

		// aplicar al motor
		int ukCmd = (int)round(uk);
		setMotor(ukCmd);

				// Guardar lo que realmente aplicamos para facilitar transferencia sin saltos
				lastAppliedSign = (ukCmd >= 0) ? +1 : -1;
				lastAppliedPWM = abs(ukCmd);
				// Si estamos en PID, mantener sincronizada la ruta manual
				if (!manualMode){
					manualDir = lastAppliedSign;
					manualPWM = lastAppliedPWM;
				}

		// Telemetría para Serial Plotter: setpoint y rpm medida filtrada
		// Plotter con etiquetas significativas
		// Formato: label:value pares separados por espacio
	Serial.print("setpointRPM:"); Serial.print(setpointRPM); Serial.print(' ');
	Serial.print("rpmSensor:"); Serial.print(v1Filt); Serial.print(' ');
	Serial.print("pwmAplicado:"); Serial.print(manualMode ? manualPWM * manualDir : (int)uk); Serial.print(' ');
	Serial.print("error:"); Serial.print(ek); Serial.print(' ');
	
	// Indicador de saturación (1 = saturado, 0 = normal)
	int saturado = (!manualMode && (fabs(uk) >= PWM_MAX - 5)) ? 1 : 0;
	Serial.print("saturado:"); Serial.print(saturado * 50); Serial.print(' '); // x50 para visualizar
	Serial.print("modo:"); Serial.println(manualMode ? 0 : 1); // 0=Manual, 1=PID

		prevMicros = nowMicros;
	}

	// ========================= PROCESAR SENSORES =========================
	// Procesar botones, switch y potenciómetro
	processSensors();
	
	// Gestionar temporizador de movimiento y parada por distancia
	processMovementTimer();

	// Parser no bloqueante de comandos por Serial
	while (Serial.available() > 0) {
		char c = (char)Serial.read();
		if (c == '\r') continue; // ignorar CR
		if (c == '\n') {
			cmdBuf[cmdIdx] = '\0';
			if (cmdIdx > 0) parseCommand(cmdBuf);
			cmdIdx = 0;
		} else {
			if (cmdIdx < sizeof(cmdBuf)-1) cmdBuf[cmdIdx++] = c; else cmdIdx = 0; // evitar overflow
		}
	}
}