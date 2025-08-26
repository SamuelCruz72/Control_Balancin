#include "definitions.h"

float a = 0.38; // Compensador de Error (0.6)
float beta = 3.6; // Compensador de Superficie (1.5)
float cv = 1; // Compensador de Velocidad (0.3)
float slide;

void calcularSMC(void) {      
  e = r - y; 
  slide = a*e+v*cv;
  // Acción de control completa
  if(slide>=0){
    u = beta*slide;
  }
  else{
    u = 0;
  }
  u = compDeadZone(u, deadZone);
  usat = constrain(u, umin, umax);
}