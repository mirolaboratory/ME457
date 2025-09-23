#include<Wire.h>

void setup(){
  Wire.begin();
  Wire.beginTransmission();
  Wire.write();
  Wire.write();  
  Wire.endTransmission();
  Serial.begin();
  while(!Serial){;}
}
void loop(){
  Wire.beginTransmission();
  Wire.write();
  Wire.endTransmission();
  Wire.requestFrom();  
  AcX=Wire.read()<<8|Wire.read();
}
