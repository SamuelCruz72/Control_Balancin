#include "definitions.h"

float Kp = 0.64;  // Ganancia proporcional (0.76)
float Ki = 0.2;   // Ganancia integral (0.4)
float Kd = 0;   // Ganancia derivativa
float p; // acción proporcional
float i = 0.0; // acción integral
float d; // acción derivativa

void calcularPID(void) {
    e = r - y; 
 
    float p = Kp*e;
 
    float d = Kd*(y-y_a)/h;

    static float i = 0.9;
    float u_bc = p + d;
    float i_bc = i + e * h;
    float u_e = u_bc + Ki * i_bc;

    if (u_e > umax && e > 0) {
        // No integramos si el error aumenta la saturación
    } else if (u_e < umin && e < 0) {
        // No integramos si el error aumenta la saturación negativa
    } else {
        i = i_bc;
    }

    u = u_bc + Ki * i;
    u = compDeadZone(u, deadZone);
    usat = constrain(u, umin, umax);
    y_a = y;
}