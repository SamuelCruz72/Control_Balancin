#include <Arduino.h>
#include "definitions.h"
#include "controller.h"

// Conectividad
WiFiClient wifiClient;

// Variables Motor
const float deadZone = 0.3; // valor estimado de la zona muerta
const float h = 0.01;  // tiempo de muestreo en segundos
const float alpha = 0.96; // Filtro complementario
const float beta = 1; // valor estimado de beta (variamos Beta de 0.1 a 500)
const float Taw = 0.9*h; // valor estimado de la constante anti-windup
const float N = 20; // valor estimado de ancho de banda del derivador, suele estar entre 5 y 20

// Variables PID
float Kp = 1200;  // Ganancia proporcional
float Ki = 100;   // Ganancia integral
float Kd = 3000;   // Ganancia derivativa
float umin = -9;
float umax = 9;

float bi = Ki*h;
float ad = Kd/ (Kd + N*h);
float bd = ad*N;
float br = h/Taw; 

float r = 0; // referencia
float e; // error
float y; // entrada
float y_a; // entrada anterior
float u; // salida
float usat; // salida al motor
float p; // acción proporcional
float i = 0.0; // acción integral
float d = 0.0; // acción derivativa
float d_a = 0.0; // acción derivativa anterior

void calcularPID(void) {
  //calculamos el error       
 e = r - y; 
 
 p = Kp*(beta*r-y);
 
 d = ad*d_a*alpha-bd*(1-alpha)*(y-y_a);
 
 // calculamos la accion de control PID
 u = p+i+d; 
 
  // realizamos la compensacion de zona muerta del motor u = u + deadzone * sign(u), si |u|>0
 u = -compDeadZone(u, deadZone);
 
 // saturamos la señal de control para los limites energéticos disponibles entre umin=-5V y umax=5V
 usat = constrain(u, -9, 9);
 
 //enviamos la señal de control saturada al motor 
 i = i+bi*e+br*(usat-u); // update integral state
 y_a = y; // update output state
 d_a = d; // update derivative state
}


void setup() {
  Serial.begin(115200);
  setup_wifi();
  configPins();
  //configMPU();
  configEncoder();
}

void loop() {
  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp); 
  
  uint16_t rawAngle = encoder.getRawAngle();
  uint16_t angle = encoder.getAngle();

  float y = angle;
  Serial.print("Angle: ");
  Serial.print(y);
  Serial.println(" degrees"); 
  calcularPID();
  voltsToMotor(usat);
}