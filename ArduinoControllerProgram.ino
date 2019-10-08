#include "WiFi.h"
#include <WiFiUdp.h>

int analogPin = 36; //potentiometer wiper (middle terminal) connected to analog pin 5
int analogPin2 = 39; //These two pins are for the left stick when the arduino USB is on the right side
                    // outside leads to ground and +5V
int analogPin3 = 34;
int analogPin4 = 23; //These two pins are for the right stick when the arduino USB is on the right side


int val = 0;  // variable to store the value read
int incomingByte = 0; // for incoming serial data

const char* ssid = "CheeseBox";
const char* password =  "pifiatfi";


const char * udpAddress = "192.168.8.246";    //IP address to send UDP data to:
const int udpPort = 5005;

void setup() { 
  
  Serial.begin(115200);           //  setup serial
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
}
  Serial.println("Connected to the WiFi network");


}

String gg = String(val);
WiFiUDP udp;  //The udp library class
byte command[27] = {0x20, 0x00, 0x00, 0x00, 0x16, 0x02, 0x62, 0x3A, 0xD5, 0xED, 0xA3, 0x01, 0xAE, 0x08, 0x2D, 0x46, 0x61, 0x41, 0xA7, 0xF6, 0xDC, 0xAF, 0xD3, 0xE6, 0x00, 0x00, 0x1E};
boolean connected = false;  //Are we currently connected?
//  udp.beginPacket(udpAddress,udpPort);   //Send a packet
//  udp.sendTo("{\"commands\":{\"servoMotor\":{\"leftDrive\":90}, \"playsound\":1}}", IPAddress(192,168,8,246), 5005);
//  udp.write(command, 27);
//  udp.endPacket();

void loop() {
  //if(connected){
    //udp.beginPacket(udpAddress,udpPort);   //Send a packet
    //udp.sendTo("{"commands":{"servoMotor":{"leftDrive":90}, "playsound":1}}", IPAddress(192,168,8,246), 5005);
    //udp.write(command, 27);
   // udp.endPacket();
  //}
     //data will be sent to server
  uint8_t buffer[255] = "{}";
  //send hello world to server
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, 255);
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 255) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
    
  delay(1000);  //Wait for 1 second

  
  val = analogRead(analogPin);  // read the input pin
  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");       // debug value
  val = analogRead(analogPin2);
  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
  val = analogRead(analogPin3);
  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
  val = analogRead(analogPin4);
  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
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
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}
