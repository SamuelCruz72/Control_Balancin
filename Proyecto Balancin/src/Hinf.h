#include "definitions.h"

const int HINF_ORDER = 4;  // Orden del controlador

// Matrices del controlador H-infinito discreto 
const float A_hinf[HINF_ORDER][HINF_ORDER] = {
    // Rellenar con los valores de A_ctrl del código MATLAB
    {0.998996527627817,	-2.86756129153764e-16,	0,	-1.80856119868300e-20},
{0.793193949096115,	-0.182667888139390,	0.0174244110357383,	5.74460062861514e-06},
{8.79754387712410,	-3.43473276684702,	-0.806740809363665,	6.37150297785144e-05},
{1759.50877542482,	-686.946553369403,	-361.348161872733,	-0.987256994044297}
};


const float B_hinf[HINF_ORDER] = {
    // Rellenar con los valores de B_ctrl del código MATLAB
    0.0199899652762782,
0.00793193949096115,
0.0879754387712410, 
17.5950877542482,
};

const float C_hinf[HINF_ORDER] = {
    // Rellenar con los valores de C_ctrl del código MATLAB
    -0.0805, -0.3034, -0.1526, -0.0378
};

const float D_hinf = -0.0031;  // Rellenar con D_ctrl del código MATLAB

// Estados del controlador H-infinito
float x_hinf[HINF_ORDER];
float x_hinf_next[HINF_ORDER];

// Variables de tiempo para el controlador discreto
unsigned long prev_time_hinf = 0;
const unsigned long HINF_SAMPLE_TIME = 10;  // 10ms = 0.01s

// Filtro anti-aliasing para la señal de error
float error_hinf_filtered = 0;
const float HINF_FILTER_ALPHA = 0.9;

// ========================= FUNCIONES H-INFINITO =========================

void initHinf() {
    // Inicializar estados del controlador a cero
    for (int i = 0; i < HINF_ORDER; i++) {
        x_hinf[i] = 0.0;
        x_hinf_next[i] = 0.0;
    }
    
    error_hinf_filtered = 0;
    prev_time_hinf = millis();
    
    Serial.println("Controlador H-infinito inicializado");
}

void calcularHinf() {
    unsigned long current_time = millis();
    
    // Verificar si ha pasado el tiempo de muestreo
    if (current_time - prev_time_hinf < HINF_SAMPLE_TIME) {
        return;  // No actualizar hasta completar el periodo de muestreo
    }
    
    // Calcular error de seguimiento
    float error_raw = y - r;
    
    // Filtro anti-aliasing en el error
    error_hinf_filtered = HINF_FILTER_ALPHA * error_hinf_filtered + 
                         (1 - HINF_FILTER_ALPHA) * error_raw;
    
    // Calcular salida del controlador: u[k] = C*x[k] + D*e[k]
    u = D_hinf * error_hinf_filtered;
    for (int i = 0; i < HINF_ORDER; i++) {
        u += C_hinf[i] * x_hinf[i];
    }
    
    // Calcular próximo estado: x[k+1] = A*x[k] + B*e[k]
    for (int i = 0; i < HINF_ORDER; i++) {
        x_hinf_next[i] = B_hinf[i] * error_hinf_filtered;
        for (int j = 0; j < HINF_ORDER; j++) {
            x_hinf_next[i] += A_hinf[i][j] * x_hinf[j];
        }
    }
    
    // Actualizar estados para la siguiente iteración
    for (int i = 0; i < HINF_ORDER; i++) {
        x_hinf[i] = x_hinf_next[i];
    }
    
    // Compensación de zona muerta
    u = compDeadZone(u, deadZone);
    
    // Saturación
    usat = constrain(u, umin, umax);
    
    prev_time_hinf = current_time;
}