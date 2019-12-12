#include "WiFi.h"
#include <WiFiUdp.h>


const char* ssid = "CheeseBox";
const char* password =  "pifiatfi";

WiFiUDP udp;
boolean connectedToWifi = false;
const char * udpAddress = "192.168.8.246";    //IP address to send UDP data to:
const int udpPort = 5005;

int analogPin=35;
int bar=34;

int val=0;
int foo=0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  connectedToWifi= true;


}

void loop() {
  foo = analogRead(analogPin);
  val = analogRead(bar);  


  
 
   Serial.println("val: " + String(val)+ " val2: "+ String(foo));


   if(connectedToWifi){
    String toSend="{\"commands\":{\"servoMotor\":{\"leftDrive\":100}}}";
    uint8_t buf[255];
    toSend.getBytes(buf, toSend.length()+1);
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buf, toSend.length());
    udp.endPacket();
    memset(buf, 0, 255);
  }

}

void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connectedToWifi = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connectedToWifi = false;
          break;
    }
}
