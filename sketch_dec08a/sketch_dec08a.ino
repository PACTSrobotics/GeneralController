
int LeftRightBottom = 35; 
int UpDownBottom = 34;
int LRBval, UDBval;
int LeftRightTop = 33;
int UpDownTop = 32; 
int test = 5;
int LRTval, UDTval;
String str0= " Left/RightA ", str1= " Up/DownA ", str2= " Left/RightB ", str3= " Up/DownB "; 

  // val = value
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
// pinMode(10, INPUT); 

}

void loop() {
  // put your main code here, to run repeatedly:
  LRBval = analogRead(LeftRightBottom);
 
  UDBval = analogRead(UpDownBottom);

  LRTval = analogRead(LeftRightTop); 

  UDTval = analogRead(UpDownTop); 
  
//  val = digitalRead(10);     // read the input pin




// 
// Serial.print (" Left/RightA ") ; 
// Serial.print (LRBval) ;
// Serial.print (" Up/DownA ") ; 
// Serial.print (UDBval) ;
// Serial.print (" Left/RightB ") ; 
// Serial.print (LRTval) ; 
// Serial.print (" Up/DownB ") ; 
// Serial.print (UDTval) ; 


// Serial.println (str0+LRBval+str1+UDBval+str2+LRTval+str3+UDTval) ; 
Serial.println ("{\"input\":{\"A\":{\"x\": "+String(LRBval)+", \"y\": "+String(UDBval)+"},\"B\": {\"x\": "+String(LRTval)+", \"y\": "+String(UDTval)+"}}}"); 
//Serial.println(LRBval);
 // Serial.println(LRBval); 
 // Serial.println(UDBval);
  
}
