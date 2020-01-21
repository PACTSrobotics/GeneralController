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
  int x=xVal-2000;
  int y=yVal-2000;
  //int Speed=(sqrt((x*x)+(y*y)))*.047;
  //int SpeedBack=90-(Speed-90);
  if(y>=-250 && x>=-250 && y<=250 && x<=250){
    //check operational signs. x and y cannot be two different values when using a conditional  statement 
    //stop
    motor1=90;
    motor2=90;
//    Serial.println("stop");
  } else if(y>x && y >= -x){
    //top
    motor1=min(int(y*0.045+90),180);
    motor2=min(int(y*0.045+90),180);
    
//    Serial.println("moving forward");
  } else if(y<x && y >= -x){
    //right
    motor1=min(int(x*0.045+90),180);
    motor2=max(int(90-x*0.045),0);
//    Serial.println("moving right");
  } else if(y<x && y <= -x){
    //down
    motor1=max(int(90-(0-(y*0.045))),0);
    motor2=max(int(90-(0-(y*0.045))),0);
//    Serial.println("moving back");

  } else {
    //left
    motor1=max(int(90-(0-(x*0.045))),0);
    motor2=min(int((0-(x*0.045))+90),180);
//    Serial.println("moving left");

  } 
//  Serial.println(motor1);
//  Serial.println(motor2);


  
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
    String toSend=motors(val, val2);
    Serial.println(toSend);
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
