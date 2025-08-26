#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Arduino.h>   

// Variables de tiempo
extern unsigned long startTime;
extern unsigned long lastChange;
extern unsigned long lastChange2;
extern unsigned long setTime;

// Conversión
extern float voltsToPwm;

// Variables generales
extern float r_list[];
extern float r;
extern float e;
extern float y;
extern float y_a;        
extern float u;
extern float usat;
extern int idx;
extern int idx_ref;
extern float v;
extern float umin;
extern float umax;

// Variables Motor
extern const float deadZone;
extern const float h;

// Funciones
void voltsToMotor(float volts);
float compDeadZone(float var, float dz);
void configPins();

#endif