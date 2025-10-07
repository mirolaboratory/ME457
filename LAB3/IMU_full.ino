#include<Wire.h>
#define MPU 0x68
#define ACCLX 0x3B
#define ACCLY 0x3D
#define ACCLZ 0x3F
#define TEMP 0x41
#define GYROX 0x43
#define GYROY 0x45
#define GYROZ 0x47

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

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
  Serial.begin(115200); 
  Wire.begin();// Initialize I2C bus
  Wire.beginTransmission(MPU);
  dataWrite(0x6B, 0);
}
void loop(){
AcX = dataRead(ACCLX);
float AcXg = AcX / ACCEL_SCALE;

AcY = dataRead(ACCLY);
float AcYg = AcY / ACCEL_SCALE;

AcZ = dataRead(ACCLZ);
float AcZg = AcZ / ACCEL_SCALE;

//Tmp = dataRead(TEMP);  
//Tmp_C = (Tmp / 340.0) + 36.53;  // Temperature formula from datasheet

Serial.print(AcXg); Serial.print(",");Serial.print(AcYg); Serial.print(",");Serial.println(AcZg); 
delay(50);
}
int16_t dataRead(uint8_t REG){
  Wire.beginTransmission(MPU);
  Wire.write(REG);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,2,true);
  int16_t tempValue = Wire.read()<<8|Wire.read(); //OUT_H & OUT_L
  return tempValue;
}

void dataWrite(uint8_t REG, uint8_t val){
  Wire.beginTransmission(MPU);
  Wire.write(REG);
  Wire.write(val);
Wire.endTransmission(true);
}
