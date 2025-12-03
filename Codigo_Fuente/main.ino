// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

// Pins para Arduino UNO + TB6612FNG
#define ENCA 2  // Pin de interrupción (2 o 3 en UNO)
#define ENCB 4  // Pin digital cualquiera
#define PWM 5   // Pin PWM (3,5,6,9,10,11 en UNO)
#define IN1 6   // Pin PWM (3,5,6,9,10,11 en UNO) 
#define IN2 7   // Pin digital
#define STBY 8  // Pin digital

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;
// Variables para PID completo (agregando término derivativo)
float ek = 0, ek1 = 0;  // Error actual y anterior para derivada
float deltaT_pid = 0.01; // Tiempo fijo para PID discreto (como medium)

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(STBY,OUTPUT);

  digitalWrite(STBY,HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM usando PPR correcto del motor GM25-370
  float v1 = velocity1/224.4*60.0; // RPM medida (método 1)
  float v2 = velocity2/224.4*60.0; // RPM medida (método 2)

  // Low-pass filter (25 Hz cutoff) para estabilizar la señal al PID
  v1Filt = 0.854f*v1Filt + 0.0728f*v1 + 0.0728f*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target RPM apropiado para GM25-370 (max 280 RPM sin carga)
  float vt = 100*(sin(currT/1e6)>0); // oscila entre 0 y 100 RPM

  // Compute the control signal u con PID completo (P + I + D)
  float kp = 5;
  float ki = 10;
  float kd = 0.05;  // Derivada más suave para evitar ruido
  
  float e = vt - v1Filt;
  
  // Término integral (acumulativo)
  // Integración con anti-windup (solo cuando salida no saturada fuerte)
  eintegral += e * deltaT;
  eintegral = constrain(eintegral, -100.0f, 100.0f);
  
  // Término derivativo (cambio del error)
  // Derivada usando deltaT real para mejor estabilidad
  float derivada = (e - ek1) / max(deltaT, 1e-3f);
  
  // Anti-windup: saturar integral como en medium
  // (constrain ya aplicado arriba)
  
  // PID completo: P + I + D
  float u = kp * e + ki * eintegral + kd * derivada;
  
  // Actualizar error anterior para próxima iteración
  ek1 = e;

  // Set the motor speed and direction con zona muerta para evitar chatter
  int dir = 1;
  const float deadband = 2.0f; // RPM de zona muerta en torno a 0
  if (u < -deadband) {
    dir = -1;
  } else {
    dir = 1; // evita invertir rápidamente cerca de 0
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  // Vencer fricción estática: mínimo PWM cuando se requiere movimiento
  if (pwr > 0 && pwr < 40) pwr = 40;
  setMotor(dir,pwr,PWM,IN1,IN2);

  // Serial Plotter: referencia y RPM medida filtrada
  Serial.print(vt); Serial.print(' ');
  Serial.println(v1Filt);
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}