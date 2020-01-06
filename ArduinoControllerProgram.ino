#include "WiFi.h"
#include <WiFiUdp.h>


const char* ssid = "CheeseBox";
const char* password =  "pifiatfi";

WiFiUDP udp;
boolean connectedToWifi = false;
const char * udpAddress = "192.168.8.246";    //IP address to send UDP data to:
const int udpPort = 5005;

int analogPin=35;
int analogPin2=34;

int val=0;
int val2=0;

String motors(int xVal, int yVal){
  int motor1, motor2;
  x=xVal-2000;
  y=yVal-2000;
  Speed=(sqrt((x*x)+(y*y))*.045;
  if(y>x and y >= -x){
    //top
    
  } else if(y<x and y >= -x) {
    //right
  
  } else if(y<x and y <= -x) {
    //down
  
  } else {
    //left
  }


  // do math here

  
  String command="{\"commands\":{\"servoMotor\":{\"leftDrive\":" + String(motor1) + ", \"rightDrive\":" + String(motor2) +"}}}";

  return command;
  
  }

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
  val = analogRead(analogPin);
  val2 = analogRead(analogPin2);  


  
 
   Serial.println("val: " + String(val)+ " val2: "+ String(val2));


   if(connectedToWifi){
    
//    String toSend="{\"commands\":{\"servoMotor\":{\"leftDrive\":100}}}";
    String toSend=motors(val1, val2);
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
