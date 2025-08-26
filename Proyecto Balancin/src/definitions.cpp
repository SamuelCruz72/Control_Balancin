#include "definitions.h"
#include <math.h>

// ESP32 Settings
#define  FREQUENCY_PWM     100 // frecuencia del pwm
#define  RESOLUTION_PWM    8  // bits del pwm
#define  CH_PWM_AIN1       0 // canales de pwm bidireccionales
#define  CH_PWM_AIN2       1
#define  AIN1      26
#define  AIN2      25

// Variables de tiempo
unsigned long startTime;
unsigned long lastChange = 0;
unsigned long lastChange2 = 0;
unsigned long setTime;

// Conversión
float voltsToPwm = (pow(2, RESOLUTION_PWM) - 1) / 5.0; 

// Variables generales
float r_list[] = {radians(90), 0};
float r;
float e;
float y;
float y_a;        
float u;
float usat;
int idx;
int idx_ref = 1;
float v;
float umin = -3.5;
float umax = 3.5;

// Variables Motor
const float deadZone = 0.36;
const float h = 0.01;

void configPins(void){
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  ledcSetup(CH_PWM_AIN1, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcSetup(CH_PWM_AIN2, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttachPin(AIN1,CH_PWM_AIN1);
  ledcAttachPin(AIN2,CH_PWM_AIN2);
}

void voltsToMotor(float volts) {
    uint16_t pwm = abs(volts) * voltsToPwm;

    if (volts < 0){
        ledcWrite(CH_PWM_AIN1, 0);
        ledcWrite(CH_PWM_AIN2, pwm);
    }
    else{
        ledcWrite(CH_PWM_AIN1, pwm);
        ledcWrite(CH_PWM_AIN2, 0);
    }
}

float compDeadZone(float var, float dz) {
    if (var == 0){
        return 0;
    } else {
        float sgnvar = abs(var) / var;
        return var + sgnvar * dz;
    }
}
