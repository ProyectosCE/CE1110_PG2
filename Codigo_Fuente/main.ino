#include <Arduino.h>
#include "esp32-hal-cpu.h"

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average


//Pines de esp32
int pin_pwm = 16;
int pin_der = 22;
int pin_izq = 23;
const int encoderPinA = 2;  // Pin de la señal A del encoder
const int encoderPinB = 4;  // Pin de la señal B del encoder
int buttonGiro = 14;
const int portPin = 35;

// Almacenamiento del valor de puerto (Rango de 0-4095)
int potValor = 0;


//Parametros de configuracion de pin PWM
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int dutyCycle = 255;


//Variables de tiempo
float tiempo = 0;
unsigned long currentTime = 0;
unsigned long elapsedTime = 0;
unsigned long prevTime = 0;
const unsigned long samplePeriod = 10000;  // Período de muestreo en microsegundos (10000 = 10milisegundos)


//Pulsos y RMPs
volatile int pulseCount = 0;
float rpm = 0;
float setPoint = 85;
float ek = 0, ek1 = 0;
float ek_1 = 0;
float deltaT = 0.01;

float derivada = 0;
float integral = 0;
float uk = 0;

float kp = 5;
float ki = 10;//2.85;
float kd = 0;

bool giro = true;


//relation = 56;



//Interrupcion para la lectura de pulsos
void IRAM_ATTR countPulse() {
  pulseCount++;

  /*if (digitalRead(encoderPinB) == HIGH) {
    pulseCount--;  // Cambia a decremento
  } else {
    pulseCount++;  // Cambia a incremento
  }*/
}



void setup() {

  Serial.begin(115200);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse, CHANGE);
  setCpuFrequencyMhz(240);

  pinMode(buttonGiro, INPUT_PULLUP);
  pinMode(pin_der, OUTPUT);
  pinMode(pin_izq, OUTPUT);
  digitalWrite(pin_der, LOW);
  digitalWrite(pin_izq, LOW);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pin_pwm, ledChannel);

for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  prevTime = micros();
}



void loop() {

  currentTime = micros();
  elapsedTime = currentTime - prevTime;


  // Si ha pasado el período de muestreo
  if (elapsedTime >= samplePeriod) {
  
    if (digitalRead(buttonGiro) == HIGH) {
      setPoint = abs(setPoint);
    }
    if (digitalRead(buttonGiro) == LOW) {
      setPoint = setPoint * -1;
    }

    if (false == giro) {
      pulseCount = pulseCount * -1;
    }

    // Calcula las RPM  (se multiplica por 100 ya que el tiempo de muestreo es 10)
    rpm = 100 * (pulseCount * (60.0 / 4200.0));
    pulseCount = 0;
    prevTime = micros();

      // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = rpm;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
  
    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }
  
    // calculate the average:
    average = total / numReadings;
  
    Serial.print("Setpoint: ");
    Serial.print(setPoint);
    Serial.print(", RPMs: ");
    Serial.println(average);

    
    // Serial.print("Setpoint: ");
    // Serial.print(setPoint);
    // Serial.print("\t\tRPMs: ");
    // Serial.println(average);



    //Control PID

    //Error
    ek = setPoint - average;

    //Calculo de la derivada del error
    derivada = (ek - ek1) / deltaT;

    //Calculo de la integral del error
    integral += ((ek + ek1) / 2) * deltaT;
    integral = constrain(integral, -100, 100);  // Saturar la integral entre -100 y 100
    ek1 = ek;

    //Calculo del PID
    uk = kp * ek + ki * integral + kd * derivada;

    //Sentido de giro en base a uk
    if (0 < uk) {
      digitalWrite(pin_izq, LOW);
      digitalWrite(pin_der, HIGH);
      giro = true;
    } else {
      digitalWrite(pin_der, LOW);
      digitalWrite(pin_izq, HIGH);
      giro = false;
    }

    //Salida del PWM del motor
    uk = abs(uk);
    uk = constrain(uk, 0, 255);  // Saturar la uk entre 0 y 255
    ledcWrite(ledChannel, uk);

  }

  potValor = analogRead(portPin);
  setPoint = int(potValor * (48.0 / 4095.0));

}