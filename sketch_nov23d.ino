#include "WiFi.h"
#include <WiFiUdp.h>

int analogPin = 35; //potentiometer wiper (middle terminal) connected to analog pin 5 up/down pin on left joystick
int analogPin2 = 27; //These two pins are for the left stick when the arduino USB is on the right side left/right left joystick
                    // outside leads to ground and +3.3V
int analogPin3 = 33; //left/right right stick
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
  // put your setup code here, to run once:
  Serial.begin(9600);           //  setup serial
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
}
  Serial.println("Connected to the WiFi network");




String gg = String(val);
WiFiUDP udp;  //The udp library class
byte command[27] = {0x20, 0x00, 0x00, 0x00, 0x16, 0x02, 0x62, 0x3A, 0xD5, 0xED, 0xA3, 0x01, 0xAE, 0x08, 0x2D, 0x46, 0x61, 0x41, 0xA7, 0xF6, 0xDC, 0xAF, 0xD3, 0xE6, 0x00, 0x00, 0x1E};
boolean connected = false;  //Are we currently connected?
//  udp.beginPacket(udpAddress,udpPort);   //Send a packet
//  udp.sendTo("{\"commands\":{\"servoMotor\":{\"leftDrive\":90}, \"playsound\":1}}", IPAddress(192,168,8,246), 5005);
//  udp.write(command, 27);
//  udp.endPacket();

}

void loop() {
Serial.print(map(analogRead(analogPin3,0,4000,0,180)));
// int right = constrain(((map(analogRead(A0), 0, 1023,180,0) - 90) + (map(analogRead(A1), 0, 1023,0,180) - 90)/2)+90,0,180);
//  int left = constrain(((map(analogRead(A0), 0, 1023,180,0) - 90) - (map(analogRead(A1), 0, 1023,0,180) - 90)/2)+90,0,180);
  
}
