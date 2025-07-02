#include <Arduino.h>
#include "definitions.h"
#include <math.h>

// Variables Motor
const float deadZone = 0.36; // valor estimado de la zona muerta
const float h = 0.01;  // tiempo de muestreo en segundos

// Variables PID
float Kp = 0.47;  // Ganancia proporcional
float Ki = 1;   // Ganancia integral
float Kd = 0;   // Ganancia derivativa
float umin = -3;
float umax = 3;

float r = radians(90); // referencia
float e; // error
float y; // entrada
float y_a; // entrada anterior
float u; // salida
float usat; // salida al motor
float p; // acción proporcional
float i = 0.0; // acción integral
float d; // acción derivativa
int idx;

void calcularPID(void) {
  //calculamos el error       
  e = r - y; 
 
  p = Kp*e;
 
  d = Kd*(y-y_a)/h;

  // Anti wind-up clamping
  float u_bc = p + d;
  float i_bc = i + e * h;
  float u_e = u_bc + Ki * i_bc;

  // Evaluamos si el controlador está saturado
  if (u_e > umax && e > 0) {
    // No integramos si el error aumenta la saturación
  } else if (u_e < umin && e < 0) {
    // No integramos si el error aumenta la saturación negativa
  } else {
    i = i_bc;  // Solo integramos si no hay riesgo de windup
  }

  // Acción de control completa
  u = u_bc + Ki * i;
 
  // realizamos la compensacion de zona muerta del motor u = u + deadzone * sign(u), si |u|>0
  u = compDeadZone(u, deadZone);

  // saturamos la señal de control para los limites energéticos disponibles entre umin=-5V y umax=5V
  usat = constrain(u, umin, umax);

  //enviamos la señal de control saturada al motor 
  y_a = y; // update output state
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Cris_V");
  Serial.println("Dispositivo iniciado, puedes conectarlo a Bluetooth");
  //setup_WiFi();
  configPins();
  configEncoder();
  //configMPU();
  startTime = millis();

  /*Serial.println("Inicio de calibración");
  delay(1000);
  
  for (int i = 0; i < 5000; i++) {
    uint16_t angle = encoder.getAngle();
    y = fmod(((angle - 3827) * 360.0 / 4096.0 + 360.0), 360.0);
    unsigned long t = millis() - startTime;
    Serial.print("Angle: ");
    Serial.print(y);
    Serial.print(" Tiempo: ");
    Serial.println(t);
  }
  Serial.println("Captura completada.");*/
}

void loop() {
  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  uint16_t angle = encoder.getAngle();

  if(SerialBT.available()){
    S = GetLine();
    strcpy(buffer,S.c_str());
    buffer[strcspn(buffer, "\r\n")] = 0;
    setTime = millis();
  }

  unsigned long now = millis();

  y = fmod((angle-1197)*360.0/4096.0, 360.0);
  if (y > 180.0) y -= 360.0;
  if (y < -180.0) y += 360.0;
  y = radians(y);
  //Serial.print(" Velocity: ");
  //Serial.print(g.gyro.x);
  if(strcmp(buffer,"g") == 0){
    unsigned long t = now - setTime;
    calcularPID();
    voltsToMotor(-usat);
    if(now - lastChange >= 50 && t <= 20000){
      Serial.print("t: "); Serial.print(t);
      Serial.print(" Angulo: "); Serial.print(y);
      Serial.print(" Error: "); Serial.print(e);
      Serial.print(" Señal: "); Serial.println(usat);
      lastChange = now;
    }
  }
  else{
    i = 0.0;
    voltsToMotor(0);
    if(now - lastChange >= 100){
      Serial.print("Raw: "); Serial.print(angle);
      Serial.print(" Angulo: "); Serial.println(y);
      lastChange = now;
    }
  }

  //Validación de modelo
  /*float Vo[] = {1.00, 2.00, 2.5, 2.00, 1.00, 0, 2.00, 0};
  if (now - lastChange >= 3000 && idx < 8) {
    voltsToMotor(Vo[idx]);  
    lastChange = now;
    idx++;
  }

  if (idx<8){
    uint16_t angle = encoder.getAngle();
    float y = fmod(((angle - 3827) * 360.0 / 4096.0 + 360.0), 360.0);
    unsigned long t = now - startTime;
    Serial.print("t: "); Serial.print(t);
    Serial.print(" ms, ángulo: "); Serial.println(y);
  }*/
  

  //Caracterización del modelo
  /*float V = 0.0;

  if(strcmp(buffer,"u") == 0){
    buffer[0] = 'o';
    buffer[1] = '\0';
    while(y < 193) {
      voltsToMotor(V);
      uint16_t angle = encoder.getAngle();
      y = fmod(((angle - 3827) * 360.0 / 4096.0 + 360.0), 360.0);
      V += 0.002;
      delay(100);
    }
    Serial.print(" Voltaje: ");
    Serial.println(V);
  }
  if(strcmp(buffer,"s") == 0){
    voltsToMotor(0);
    uint16_t angle = encoder.getAngle();
    y = fmod(((angle - 3827) * 360.0 / 4096.0 + 360.0), 360.0);
  }*/
}