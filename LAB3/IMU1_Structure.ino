#include<Wire.h>

// Scale factors for different ranges
#define ACCEL_SCALE_2G 16384.0    // LSB/g for ±2g range
#define ACCEL_SCALE_4G 8192.0     // LSB/g for ±4g range  
#define ACCEL_SCALE_8G 4096.0     // LSB/g for ±8g range
#define ACCEL_SCALE_16G 2048.0    // LSB/g for ±16g range

#define GYRO_SCALE_250 131.0      // LSB/(°/s) for ±250°/s range
#define GYRO_SCALE_500 65.5       // LSB/(°/s) for ±500°/s range
#define GYRO_SCALE_1000 32.8      // LSB/(°/s) for ±1000°/s range
#define GYRO_SCALE_2000 16.4      // LSB/(°/s) for ±2000°/s range

// Choose your scales (change these based on your range settings)
#define ACCEL_SCALE ACCEL_SCALE_2G  // Default ±2g
#define GYRO_SCALE GYRO_SCALE_250   // Default ±250°/s

void setup(){
  Wire.begin();
  Wire.beginTransmission();
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
