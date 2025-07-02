#include <Adafruit_MPU6050.h>
#include <Adafruit_AS5600.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <BluetoothSerial.h>

const char WIFI_SSID[] = "HUAWEI Y9 Prime 2019";
const char WIFI_PASSWORD[] = "Samuel_Cruz";
unsigned long startTime;
unsigned long lastChange = 0;
unsigned long setTime;

Adafruit_MPU6050 mpu;
Adafruit_AS5600 encoder;
BluetoothSerial SerialBT;
WiFiClient wifiClient;

// ESP32 Settings
#define  FREQUENCY_PWM     100 // frecuencia del pwm
#define  RESOLUTION_PWM    8  // bits del pwm
#define  CH_PWM_AIN1       0 // canales de pwm bidireccionales
#define  CH_PWM_AIN2       1
#define  AIN1      26
#define  AIN2      25

// Bluethooth Constants
String S;
char buffer[1];

// factor de conversion de bits a voltios:  2^ #bits -1 / voltios
float  voltsToPwm =   (pow(2, RESOLUTION_PWM) - 1) / 5; 

void configPins(void){
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  ledcSetup(CH_PWM_AIN1, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcSetup(CH_PWM_AIN2, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttachPin(AIN1,CH_PWM_AIN1);
  ledcAttachPin(AIN2,CH_PWM_AIN2);
}

String GetLine(void){   
  String S = "" ;
  if(SerialBT.available()){    
    char c = SerialBT.read(); 
    while ( c != '\n'){             //Hasta que el caracter sea intro    
      S = S + c;
      delay(25);
      c = SerialBT.read();
    }
  }
  return( S + '\n') ;
}

void setup_WiFi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Conectado al WiFi");
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

void voltsToMotor(float volts){
    // This function convert a voltage value given in the variable volts
    // to a bipolar pwm signal for controlling the DC motor
    
    uint16_t pwm = abs(volts) * voltsToPwm;


    if (volts < 0){
        // if var volts is negative use CH_PWM_AIN2 to output a pwm signal
        // proportional to the input voltage
        ledcWrite(CH_PWM_AIN1, 0);
        ledcWrite(CH_PWM_AIN2, pwm);
    }
    else{
        // if var volts is negative use CH_PWM_AIN1 to output a pwm signal
        // proportional to the input voltage
        ledcWrite(CH_PWM_AIN1, pwm);
        ledcWrite(CH_PWM_AIN2, 0);
    }
}

float compDeadZone(float var, float dz){
    // This function compensates the dead zone of DC motor
    if (var == 0){
        return 0;
    }
    else {
        float sgnvar = abs(var)/var;
        return var + sgnvar * dz ;
    }
}




