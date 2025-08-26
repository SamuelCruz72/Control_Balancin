#include "definitions.h"

// Parámetros del sistema
const float W_L = 0.04;         // Peso izquierdo [N]
const float W_R = 0.02;         // Peso derecho [N]
const float b = 0.06253;        // Distancia del fulcro al peso izquierdo [m]
const float c = 0.02605;        // Distancia del fulcro al peso derecho [m]
const float a_plant = 0.120;    // Factor de acoplamiento del actuador [N·m/N]
const float omega_hz = 8.3819;  // Frecuencia natural [Hz]
const float zeta = 0.0487;      // Factor de amortiguamiento

// Parámetros calculados
const float omega_n = omega_hz;
const float K_g = (W_L * b - W_R * c);
const float J = K_g / (omega_n * omega_n);
const float B = 2 * zeta * sqrt(J * K_g);

// Matrices del sistema planta
const float A_plant[2][2] = {
    {0, 1},
    {-K_g/J, -B/J}
};
const float B_plant[2] = {0, a_plant/J};

// Modelo de referencia
const float wn_ref = 2 * omega_n;  // 25% de la frecuencia natural
const float zeta_ref = 0.8;           // Amortiguamiento moderado

const float A_ref[2][2] = {
    {0, 1},
    {-wn_ref * wn_ref, -2 * zeta_ref * wn_ref}
};
const float B_ref[2] = {0, wn_ref * wn_ref};

// Parámetros del controlador MRAC
const float gamma_x = 1.0;     // Ganancia de adaptación para estados
const float gamma_ref = 2.0;    // Ganancia de adaptación para referencia (renombrado)
const float gamma_u = 0.25;     // Ganancia de adaptación auxiliar

// Límites de saturación
const float Kx_max[2] = {10.0, 2.0};
const float Kr_max = 4.0;
const float Ku_max = 1.5;

// Parámetros de robustez
const float sigma_modification = 0.1;
const float deadzone_threshold = 0.002;
const float max_control = 3.5;
const float filter_alpha = 0.95;

// Variables de estado del controlador MRAC
float x_plant[2] = {0, 0};      // Estados de la planta [posición, velocidad]
float x_ref[2] = {0, 0};        // Estados del modelo de referencia
float Kx[2] = {1.0, 0.5};       // Ganancias adaptativas para estados (inicialización LQR reducida)
//float Kr = wn_ref * wn_ref;     // Ganancia feedforward
float Kr = 1.5;     // Ganancia feedforward
float Ku = 0.05;                // Ganancia auxiliar

// Variables de filtro
float u_filtered = 0;
float error_filtered = 0;
float e1_raw = 0;               // Declarar e1_raw como variable global

// Tiempo anterior para integración
unsigned long prev_time_mrac = 0;

void initMRAC() {
    // Inicialización de variables MRAC
    x_plant[0] = 0;
    x_plant[1] = 0;
    x_ref[0] = 0;
    x_ref[1] = 0;
    
    // Inicialización con valores conservadores basados en LQR
    Kx[0] = 1.0;   // Ganancia de posición
    Kx[1] = 0.5;   // Ganancia de velocidad
    Kr = 1.5;
    Ku = 0.5;
    
    u_filtered = 0;
    error_filtered = 0;
    e1_raw = 0;
    prev_time_mrac = millis();
}

float saturateSmooth(float value, float limit) {
    // Saturación suave con tanh
    return limit * tanh(value / limit);
}

void integrateRK4(float state[2], const float A[2][2], const float B[2], float input, float dt) {
    // Integración Runge-Kutta 4 simplificada para sistema 2x2
    float k1[2], k2[2], k3[2], k4[2];
    float temp[2];
    
    // k1 = f(x, u)
    k1[0] = A[0][0] * state[0] + A[0][1] * state[1] + B[0] * input;
    k1[1] = A[1][0] * state[0] + A[1][1] * state[1] + B[1] * input;
    
    // k2 = f(x + dt/2 * k1, u)
    temp[0] = state[0] + 0.5 * dt * k1[0];
    temp[1] = state[1] + 0.5 * dt * k1[1];
    k2[0] = A[0][0] * temp[0] + A[0][1] * temp[1] + B[0] * input;
    k2[1] = A[1][0] * temp[0] + A[1][1] * temp[1] + B[1] * input;
    
    // k3 = f(x + dt/2 * k2, u)
    temp[0] = state[0] + 0.5 * dt * k2[0];
    temp[1] = state[1] + 0.5 * dt * k2[1];
    k3[0] = A[0][0] * temp[0] + A[0][1] * temp[1] + B[0] * input;
    k3[1] = A[1][0] * temp[0] + A[1][1] * temp[1] + B[1] * input;
    
    // k4 = f(x + dt * k3, u)
    temp[0] = state[0] + dt * k3[0];
    temp[1] = state[1] + dt * k3[1];
    k4[0] = A[0][0] * temp[0] + A[0][1] * temp[1] + B[0] * input;
    k4[1] = A[1][0] * temp[0] + A[1][1] * temp[1] + B[1] * input;
    
    // Actualizar estado
    state[0] += (dt / 6.0) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
    state[1] += (dt / 6.0) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
}

void calcularMRAC() {
    unsigned long current_time = millis();
    float dt = (current_time - prev_time_mrac) / 1000.0;  // Convertir a segundos
    
    // Limitar dt para evitar problemas numéricos
    if (dt > 0.05) dt = 0.01;  // Usar dt fijo si es muy grande
    if (dt <= 0) return;       // No calcular si no ha pasado tiempo
    
    // Actualizar estados de la planta
    x_plant[0] = y;  // Posición actual del encoder
    x_plant[1] = v;  // Velocidad angular del giroscopio
    
    // Señal de referencia (escalón suave)
    float r_val = r;
    
    // Error de seguimiento con filtro
    e1_raw = x_plant[0] - x_ref[0];
    float e2 = x_plant[1] - x_ref[1];
    
    // Filtro pasa-bajas en error
    error_filtered = filter_alpha * error_filtered + (1 - filter_alpha) * e1_raw;
    float e1 = error_filtered;
    
    // Zona muerta adaptativa
    float deadzone = deadzone_threshold * (1 + 0.1 * (abs(x_plant[0]) + abs(x_plant[1])));
    float e1_adapt = (abs(e1) < deadzone) ? 0 : e1;
    
    // Ley de control MRAC
    float u_fb = -Kx[0] * x_plant[0] - Kx[1] * x_plant[1];  // Feedback
    float u_ff = Kr * r_val;                                 // Feedforward
    float u_aux = Ku * e1;                                   // Auxiliar
    float u_raw = u_fb + u_ff + u_aux;
    
    // Filtro en control
    u_filtered = filter_alpha * u_filtered + (1 - filter_alpha) * u_raw;
    
    // Saturación suave
    u = saturateSmooth(u_filtered, max_control);
    
    // Adaptación de ganancias (solo si el error es significativo)
    if (abs(e1_adapt) > deadzone) {
        // Factor de normalización
        float norm_factor = 1.0 / (1.0 + abs(x_plant[0]) + abs(x_plant[1]) + abs(r_val));
        
        // Gradientes con modificación sigma (leakage)
        float grad_Kx0 = gamma_x * e1_adapt * x_plant[0] * norm_factor - sigma_modification * Kx[0];
        float grad_Kx1 = gamma_x * e1_adapt * x_plant[1] * norm_factor - sigma_modification * Kx[1];
        float grad_Kr = gamma_ref * e1_adapt * r_val * norm_factor - sigma_modification * (Kr - wn_ref * wn_ref);
        float grad_Ku = gamma_u * e1_adapt * e1 * norm_factor - sigma_modification * Ku;
        
        // Actualización de ganancias
        Kx[0] += grad_Kx0 * dt;
        Kx[1] += grad_Kx1 * dt;
        Kr += grad_Kr * dt;
        Ku += grad_Ku * dt;
        
        // Proyección suave con tanh
        Kx[0] = saturateSmooth(Kx[0], Kx_max[0]);
        Kx[1] = saturateSmooth(Kx[1], Kx_max[1]);
        Kr = saturateSmooth(Kr, Kr_max);
        Ku = saturateSmooth(Ku, Ku_max);
    }
    
    // Integrar modelo de referencia usando RK4
    integrateRK4(x_ref, A_ref, B_ref, r_val, dt);
    
    // Compensación de zona muerta
    u = compDeadZone(u, deadZone);
    
    // Saturación final
    usat = constrain(u, umin, umax);
    
    prev_time_mrac = current_time;
}