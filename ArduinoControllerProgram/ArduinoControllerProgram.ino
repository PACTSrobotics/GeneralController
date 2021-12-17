#include "WiFi.h"
#include <WiFiUdp.h>

const char* ssid = "CheeseBox";
const char* password =  "pifiatfi";

WiFiUDP udp;
boolean connectedToWifi = false;
const char * udpAddress = "192.168.8.246";    //IP address to send UDP data to:
const int udpPort = 5005;

//Analog Pins
int xPin=35;
int yPin=34;
int headPin=33;

//Sent Values (global to appear in serial plotter)
int leftMotor, rightMotor, headMotor;

//Motor constants
const float headSpeed = 0.5;
const float motorSpeed = 0.5;
const int deadzone = 300;
//0 to 4000 -> -2000 to 2000 -> -90 to 90 -> 0 to 180
const float joystick2Motor = 2000 / 90;


void rawHeadToMotor(int rawHead, int* headMotor) {
  int hx = rawHead - 2000;
  
  if (hx <= deadzone && hx >= -deadzone){
    //normal stop or unplugged wire
    *headMotor = 90;
  } else if(hx > 0){
    //right
    *headMotor = int(min(hx * joystick2Motor * headSpeed, 90 * headSpeed) + 90);
  } else {
    //left
    *headMotor = int(max(hx * joystick2Motor * headSpeed, -90 * headSpeed) + 90);
  }
}


void rawXYToMotor(int rawX, int rawY, int* leftMotor, int* rightMotor) {
  int x = rawX - 2000;
  int y = rawY - 2000;
  
  if (y >= -deadzone && x >= -deadzone && y <= deadzone && x <= deadzone){
    //check operational signs. x and y cannot be two different values when using a conditional statement 
    //stop
    *leftMotor = 90;
    *rightMotor = 90;
  } else if (y > x && y >= -x){
    //top
    *leftMotor = int(min(y * joystick2Motor * motorSpeed, 90 * motorSpeed) + 90);
    *rightMotor = *leftMotor;
  } else if (y < x && y >= -x){
    //right
    *leftMotor = int(min(x * joystick2Motor * motorSpeed, 90 * motorSpeed) + 90);
    *rightMotor = 180 - *leftMotor;
  } else if (y < x && y <= -x){
    //down
    *leftMotor = int(max(y * joystick2Motor * motorSpeed, -90 * motorSpeed) + 90);
    *rightMotor = *leftMotor;
  } else {
    //left
    *leftMotor = int(max(x * joystick2Motor * motorSpeed, -90 * motorSpeed) + 90);
    *rightMotor = 180 - *leftMotor;
  }
}


//Default Command: {"commands":{"servoMotor":{"leftDrive":90, "rightDrive":90}, "forward":{"head": {"commands":{"servoMotor":{"mainDrive":90}}}}}}
String formatMotorsCommand(int left, int right, int head) {
  return "{\"commands\":{\"servoMotor\":{\"leftDrive\":" + String(left) + ", \"rightDrive\":" + String(right) + "}, \"forward\":{\"head\":{\"commands\":{\"servoMotor\":{\"mainDrive\":" + String(head) + "}}}}}}";
}


void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  connectedToWifi = true;
}


void loop() {
  if (!connectedToWifi) return;
  
  int rawX = analogRead(xPin);
  int rawY = analogRead(yPin);  
  int rawHead = analogRead(headPin);
  rawHeadToMotor(rawHead, &headMotor);
  rawXYToMotor(rawX, rawY, &leftMotor, &rightMotor);
  String command = formatMotorsCommand(leftMotor, rightMotor, headMotor);
  Serial.println(command);
  uint8_t buf[255];
  command.getBytes(buf, command.length()+1);
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buf, command.length());
  udp.endPacket();
  memset(buf, 0, 255);
}


//Mystery function
void WiFiEvent(WiFiEvent_t event){
  connectedToWifi = true;
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
