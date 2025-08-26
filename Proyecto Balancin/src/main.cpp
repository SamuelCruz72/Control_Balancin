#include "definitions.h"
#include "sensors.h"
#include "com.h"
#include "PID.h"
#include "SMC.h"
#include "MRAC.h"
#include "Hinf.h"

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_Cris_V");
    Serial.println("Dispositivo iniciado, puedes conectarlo a Bluetooth");
    
    configPins();
    configEncoder();
    configMPU();
    
    // Inicializar controladores
    initMRAC();
    initHinf();  
    
    startTime = millis();
}

void loop() {
    unsigned long now = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    uint16_t angle = encoder.getAngle();
    
    y = fmod((angle-offset)*360.0/4096.0, 360.0);
    if (y > 180.0) y -= 360.0;
    if (y < -180.0) y += 360.0;
    y = radians(y);
    v = g.gyro.z;

    if(SerialBT.available()){
        S = GetLine();
        strcpy(buffer,S.c_str());
        buffer[strcspn(buffer, "\r\n")] = 0;
        setTime = millis();
        i = 0.0;  // Reset para PID
        initHinf();  // Reset para H-infinito
    }

    unsigned long t = now - setTime; 
    
    // =============== IMPLEMENTACIÓN H-INFINITO ===============
    if(strcmp(buffer,"h") == 0){  
        r = radians(90);  // Referencia de 90 grados
        calcularHinf();
        voltsToMotor(usat);
        
        if(now - lastChange >= 100 && t <= 20000){
            Serial.print("H∞ - t: "); Serial.print(t);
            Serial.print(" Ang: "); Serial.print(degrees(y), 2);
            Serial.print(" Ref: "); Serial.print(degrees(r), 2);
            Serial.print(" Err: "); Serial.print(degrees(r-y), 4);
            Serial.print(" U: "); Serial.print(usat, 3);
            Serial.print(" Estados: [");
            for (int i = 0; i < HINF_ORDER; i++) {
                  Serial.print(x_hinf[i], 4);  
                  if (i < HINF_ORDER - 1) Serial.print(",");
              }
              Serial.println("]");
            lastChange = now;
        }
    }
    
    // =============== IMPLEMENTACIÓN MRAC ===============
    if(strcmp(buffer,"M") == 0){
        r = radians(90);
        calcularMRAC();
        voltsToMotor(usat);
        
        if(now - lastChange >= 100 && t <= 20000){
            Serial.print("MRAC - t: "); Serial.print(t);
            Serial.print(" Ang: "); Serial.print(degrees(y), 2);
            Serial.print(" Ref: "); Serial.print(degrees(r), 2);
            Serial.print(" Err: "); Serial.print(degrees(e1_raw), 4);
            Serial.print(" U: "); Serial.print(usat, 3);
            Serial.print(" Kx:["); Serial.print(Kx[0], 3); 
            Serial.print(","); Serial.print(Kx[1], 3);
            Serial.print("] Kr:"); Serial.print(Kr, 3);
            Serial.print(" Ku:"); Serial.println(Ku, 4);
            lastChange = now;
        }
    }
    
    // =============== IMPLEMENTACION PID ===============
    if(strcmp(buffer,"p") == 0){
        r = radians(90);
        calcularPID();
        voltsToMotor(usat);
        if(now - lastChange2 >= 5000){
            if(idx_ref == 1){
                idx_ref = 2;
            }
            else{
                idx_ref = 1;
            }
            lastChange2 = now;
        }
        if(now - lastChange >= 100 && t <= 20000){
            Serial.print("PID - t: "); Serial.print(t);
            Serial.print(" Angulo: "); Serial.print(y);
            Serial.print(" Referencia: "); Serial.print(r);
            Serial.print(" Error: "); Serial.print(e);
            Serial.print(" Señal: "); Serial.println(usat);
            lastChange = now;
        }
    }

    // =============== IMPLEMENTACION SMC ===============
    if(strcmp(buffer,"s") == 0){
        r = radians(90);
        calcularSMC();
        voltsToMotor(usat);
        if(now - lastChange >= 100 && t <= 20000){
            Serial.print("SMC - t: "); Serial.print(t);
            Serial.print(" Angulo: "); Serial.print(y);
            Serial.print(" Error: "); Serial.print(e);
            Serial.print(" Velocidad: "); Serial.print(v);
            Serial.print(" Referencia: "); Serial.print(r);
            Serial.print(" Señal: "); Serial.println(usat);
            lastChange = now;
        }
    }
    
    // =============== COMPARACIÓN DE CONTROLADORES ===============
    if(strcmp(buffer,"comp") == 0){  // Modo comparación
        r = radians(90);
        
        // Alternar entre controladores cada 5 segundos
        if(now - lastChange2 >= 5000){
            if(idx_ref == 1){
                idx_ref = 2;  // H-infinito
            }
            else if(idx_ref == 2){
                idx_ref = 3;  // MRAC
            }
            else if(idx_ref == 3){
                idx_ref = 4;  // PID
            }
            else{
                idx_ref = 1;  // SMC
            }
            lastChange2 = now;
            
            // Reiniciar controladores al cambiar
            initHinf();
            i = 0.0;  // Reset PID
        }
        
        // Aplicar controlador correspondiente
        if(idx_ref == 1){
            calcularSMC();
            Serial.print("SMC - ");
        }
        else if(idx_ref == 2){
            calcularHinf();
            Serial.print("H∞ - ");
        }
        else if(idx_ref == 3){
            calcularMRAC();
            Serial.print("MRAC - ");
        }
        else{
            calcularPID();
            Serial.print("PID - ");
        }
        
        voltsToMotor(usat);
        
        if(now - lastChange >= 100 && t <= 60000){  // 60 segundos para ver todos
            Serial.print("t: "); Serial.print(t);
            Serial.print(" Ang: "); Serial.print(degrees(y), 2);
            Serial.print(" Ref: "); Serial.print(degrees(r), 2);
            Serial.print(" U: "); Serial.println(usat, 3);
            lastChange = now;
        }
    }
    
    // =============== CORRECCIÓN DE ANGULOS =================
    if(strcmp(buffer,"a") == 0){
        voltsToMotor(0);
        if(now - lastChange >= 100){
            Serial.print("Raw: "); Serial.print(angle);
            Serial.print(" Angulo: "); Serial.print(degrees(y));
            Serial.print(" Velocidad: "); Serial.println(g.gyro.z);
            lastChange = now;
        }
    }

     // =============== VALIDACIÓN DE MODELO ===================
    if(strcmp(buffer,"m") == 0){
      float Vo[] = {1.00, 1.50, 2.00, 1.50, 1.00, 0, 1.50, 0};
      if (now - lastChange >= 3000 && idx < 8) {
        voltsToMotor(Vo[idx]);  
        lastChange = now;
        idx++;
      }

      if (idx<9){
        Serial.print("t: "); Serial.print(t);
        Serial.print(" ms, ángulo: "); Serial.println(y);
      }
    }

    // ============== CARACTERIZACIÓN DE LA FRICCIÓN ================
    if(strcmp(buffer,"f") == 0){
      if (now - lastChange >= 100) {
        Serial.print("Angle: "); Serial.print(y);
        Serial.print(" Tiempo: "); Serial.println(t);
        lastChange = now;
      }
    }

    // ============== CARACTERIZACIÓN DEL MOTOR ================
    float V = 0.0;

    if(strcmp(buffer,"u") == 0){
      buffer[0] = 'o';
      buffer[1] = '\0';

      while(y < 90) {
        uint16_t angle = encoder.getAngle();
        y = fmod((angle-offset)*360.0/4096.0, 360.0);
        if (y > 180.0) y -= 360.0;
        if (y < -180.0) y += 360.0;
        voltsToMotor(V);
        V += 0.01;
        if (V > 5.0){
          break;
        }
        delay(100);
      }
      Serial.print(" Voltaje: ");
      Serial.println(V);
    }

    // ============== DETENER MOTOR ================
    if(strcmp(buffer,"x") == 0){
        voltsToMotor(0);
    }
}