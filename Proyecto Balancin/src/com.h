#include <WiFi.h>
#include <BluetoothSerial.h>

const char WIFI_SSID[] = "HUAWEI Y9 Prime 2019";
const char WIFI_PASSWORD[] = "Samuel_Cruz";

BluetoothSerial SerialBT;
WiFiClient wifiClient;

// Bluethooth Constants
String S;
char buffer[1];

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