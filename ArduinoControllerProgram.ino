/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGUH and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 3014
  by Scotty Fitzgerald
  modified 2 Sep 3016
  by R-2r-O Guadalupi
  modified 8 Sep 3016
  by Colby Noobman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include "WiFi.h"
#include <WiFiUdp.h>

int analogPin = 35; //potentiometer wiper (middle terminal) connected to analog pin 5 up/down pin on left joystick
int analogPin2 = 27; //These two pins are for the left stick when the arduino USB is on the right side left/right left joystick
                    // outside leads to ground and +3.3V
//int analogPin3 = 33; //left/right right stick
//int analogPin4 = 32; //These two pins are for the right stick when the arduino USB is on the right side up/down right joystick


int val = 0; // variable to store the value read
int val2 = 0;
//int val3 = 0;
//int val4 = 0;
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
  val = analogRead(analogPin);  // read the input pin
//  Serial.println(val);

  val2 = analogRead(analogPin2);
  //Serial.println(val2);
//  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");       // debug value
int motorSpeed = val/22.75;
//  Serial.println("val: " + String(val));
//  Serial.println("motorspeed: " + String(motorSpeed));
//  val2 = analogRead(analogPin2);
//  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
//  int motorSpeed2 = val2/22.75;
//  Serial.println("motorspeed: " + String(motorSpeed2));
//  val3 = analogRead(analogPin3);
//  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
//  int motorSpeed3 = val3/22.75;
//  Serial.println("motorspeed: " + String(motorSpeed3));
//  val4 = analogRead(analogPin4);
//  Serial.println("{commands:{servoMotor:{leftDrive:" + String(val) + "{rightDrive:" + String(val) + "}}}}");
//  int motorSpeed4 = val4/22.75;
//  Serial.println("motorspeed: " + String(motorSpeed4));

  
  int coordx = val;
  int coordy = val2;

  //Serial.println(val);
  Serial.println(val2);
//  int coordresult = sqrt(pow(coordx-1945, 2) + pow(coordy-1945, 2));
//  Serial.println(coordresult);
  
  //if(connected){
    //udp.beginPacket(udpAddress,udpPort);   //Send a packet
    //udp.sendTo("{"commands":{"servoMotor":{"leftDrive":90}, "playsound":1}}", IPAddress(192,168,8,246), 5005);
    //udp.write(command, 27);
   // udp.endPacket();
  //}
     //data will be sent to server
  uint8_t buffer[255] = "{}";
//  //send hello world to server
//  udp.beginPacket(udpAddress, udpPort);
//  udp.write(buffer, 11);
//  udp.endPacket();
//  memset(buffer, 0, 255);
//  //processing incoming packet, must be called before reading the buffer
//  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 255) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
    
  delay(1000);  //Wait for 1 second

  
  
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
