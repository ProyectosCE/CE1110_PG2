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
#define ENCB 4  // Pin digital cualquiera
#define PWM 5   // Pin PWM (3,5,6,9,10,11 en UNO)
#define IN1 6   // Pin PWM (3,5,6,9,10,11 en UNO)
#define IN2 7   // Pin digital
#define STBY 8  // Pin digital

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
int manualDir = 1;           // +1 adelante, -1 reversa (solo en modo manual)
int manualPWM = 128;         // Valor de PWM (0-255) para el modo manual

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

// --- Comandos de alto nivel ---
// Magnitud de velocidad objetivo para comandos PID
// LÍMITE SUPERIOR basado en especificaciones del motor:
// - Velocidad sin carga: 280 RPM (especificación)
// - Velocidad con carga máxima realista: ~240 RPM
float targetSpeedMagRPM = 100.0f; // restringida a [20, 240]

// abre (gira sentido positivo) con PWM fijo
void abrir() {
	manualMode = true; manualDir = +1;
}

// cierra (gira sentido negativo) con PWM fijo
void cerrar() {
	manualMode = true; manualDir = -1;
}

// configura la velocidad (magnitud) entre 20 y 240 RPM para el modo PID
void velocidad(float rpm) {
	if (rpm < 20.0f) rpm = 20.0f;   // Límite inferior
	if (rpm > 240.0f) rpm = 240.0f; // Límite superior ajustado a capacidad real bajo carga
	targetSpeedMagRPM = rpm;
	// En PID, actualizar setpoint en misma dirección actual
	if (!manualMode) {
		if (setpointRPM < 0) setpointRPM = -targetSpeedMagRPM; else setpointRPM = targetSpeedMagRPM;
	}
}

// activar/desactivar PID
void activar(){
	// Activa control PID con realimentación
	manualMode = false;
	// Evitar golpe al activar PID: el setpoint se iguala a la velocidad actual
	setpointRPM = v1Filt;
	integral = 0.0f;
	ek1 = 0.0f;
}

void desactivar(){
	// Desactiva PID: control por PWM puro, sin ajustes en tiempo real
	manualMode = true;
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
		setpointRPM = val;
		targetSpeedMagRPM = fabs(val);
		manualMode = false; // Cambia a modo PID
		return;
	}
	// PWM<number> -> Fija PWM para modo Manual
	if (strncmp(up, "PWM", 3) == 0){
		int val = atoi(line+3);
		manualPWM = constrain(val, 0, 255);
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
			manualMode = true; // usar trayectoria directa a 0
			setpointRPM = 0;
			manualPWM = 0;
			analogWrite(PWM, 0);
			digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
			return;
		}

		// DIRINV -> alterna inversión de pines IN1/IN2
		if (strncmp(up, "DIRINV", 6) == 0){ invertDirPins = !invertDirPins; return; }
}

void setup() {
	Serial.begin(115200);

	// Pines
		// Muchos encoders de efecto Hall/open-collector requieren pull-ups
		pinMode(ENCA, INPUT_PULLUP);
		pinMode(ENCB, INPUT_PULLUP);
	pinMode(PWM, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(STBY, OUTPUT);

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
			uk = (manualDir >= 0) ? manualPWM : -manualPWM;
			// Resetear variables PID para evitar acumulación
			integral = 0.0f;
			ek1 = 0.0f;
		} else {
			// Modo PID: lazo cerrado CON RETROALIMENTACIÓN
			// El sistema ajusta automáticamente el PWM para alcanzar el setpoint
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
			
			// Limitar salida y advertir si está saturado
			uk = constrain(uk, -PWM_MAX, PWM_MAX);
		}

		// aplicar al motor
		int ukCmd = (int)round(uk);
		setMotor(ukCmd);

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

