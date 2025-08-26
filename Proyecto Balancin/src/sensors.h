#include <Adafruit_MPU6050.h>
#include <Adafruit_AS5600.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_AS5600 encoder;

// VARIABLES ENCODER 
int offset = 3452;

void configEncoder(void){
  Wire.begin();
  if (!encoder.begin()) {
    Serial.println("AS5600 no detectado. Revisa Conexión.");
    while (1);
  }
  Serial.println("AS5600 inicializada.");
  encoder.enableWatchdog(false);
  // Normal (high) power mode
  encoder.setPowerMode(AS5600_POWER_MODE_NOM);
  // No Hysteresis
  encoder.setHysteresis(AS5600_HYSTERESIS_OFF);

  // analog output
  encoder.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);

  // setup filters
  encoder.setSlowFilter(AS5600_SLOW_FILTER_16X);
  encoder.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);

  // Reset position settings to defaults
  encoder.setZPosition(0);
  encoder.setMPosition(4095);
  encoder.setMaxAngle(4095);

  Serial.println("Esperando mediciones...");
}

void configMPU(void){
    if (!mpu.begin()) {
        Serial.println("Fallo en el MPU6050 chip");
        while (1) {
          delay(10);
        }
      }
      Serial.println("MPU6050 Encontrado!");
    
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      Serial.print("Rango de Acelerometro: ");
      switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
      }
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      Serial.print("Rango del giróscopo: ");
      switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
      }
    
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
      Serial.print("Filtro pasbanda: ");
      switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
      }
    
      Serial.println("");
      delay(100);
}