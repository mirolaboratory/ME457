#include<Wire.h>
#define MPU 0x68
#define ACCLX 0x3B
#define ACCLY 0x3D
#define ACCLZ 0x3F
#define TEMP 0x41
#define GYROX 0x43
#define GYROY 0x45
#define GYROZ 0x47
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV_REG  0x19
#define CONFIG_REG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define INT_STATUS      0x3A
#define ACCEL_XOUT_H    0x3B
#define USER_CTRL       0x6A      // FIFO and I2C Master control
#define FIFO_EN         0x23      // FIFO Enable register
#define FIFO_COUNTH     0x72      // FIFO Count High byte
#define FIFO_COUNTL     0x73      // FIFO Count Low byte
#define FIFO_R_W        0x74      // FIFO Read/Write register

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

#define NUM_BATCHES 4           // Number of batches to collect
#define BYTES_PER_BATCH 6       // 3 axes × 2 bytes each
#define TOTAL_BYTES 24          // 4 batches × 6 bytes

// Structure to hold raw accelerometer data for one sample
struct AccelData {
  int16_t x;
  int16_t y;
  int16_t z;
};

// Array to store 4 batches of accelerometer data using structures
AccelData accelData[NUM_BATCHES];

unsigned long startTime;
bool firstLoop = true;
unsigned long currentTime; 

void setup(){
  Serial.begin(115200); 
  Wire.begin();  // Initialize I2C bus
  
  // Wake up MPU6050
  dataWrite(PWR_MGMT_1, 0);
  delay(100);  // Give sensor time to stabilize
  
  // Configure for 200 Hz sampling rate
  
  // Enable FIFO
  
  // Enable accelerometer data to go into FIFO
}

void loop(){
  if (firstLoop) {
    startTime = millis();
    firstLoop = false;
  }

  // Get FIFO count
  uint16_t fifoCount = getFifoCount();
  
  // Process all available complete sets of 4 batches
  while (fifoCount >= TOTAL_BYTES) {
    currentTime = millis() - startTime;
    
    // Read 24 bytes from FIFO (4 complete samples)
    
    // Parse 4 batches into structure array
    for (int i = 0; i < NUM_BATCHES; i++) {
      accelData[i].x = (Wire.read() << 8) | Wire.read();  // AcX
    }
    
    // Variables to store averaged values
    float AcX_avg, AcY_avg, AcZ_avg;
    
    // Calculate averages using the function
    calAverage(AcX_avg, AcY_avg, AcZ_avg);
    
    // Print averaged data
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(AcX_avg, 4);
    Serial.print(",");
    Serial.print(AcY_avg, 4);
    Serial.print(",");
    Serial.println(AcZ_avg, 4);

    // Update FIFO count for next iteration
    fifoCount -= 24;
  }
}

// Function to calculate average for each axis from accelData array
void calAverage(float &avgX, float &avgY, float &avgZ) {
  float sumX = 0, sumY = 0, sumZ = 0;
  
  // Sum all values for each axis
  for (int i = 0; i < NUM_BATCHES; i++) {
    sumX += accelData[i].x;
    sumY += accelData[i].y;
    sumZ += accelData[i].z;
  }
  
  // Calculate average (still in raw int16_t values)
  float rawAvgX = sumX / NUM_BATCHES;
  float rawAvgY = sumY / NUM_BATCHES;
  float rawAvgZ = sumZ / NUM_BATCHES;
  
  // Convert to g and return via reference parameters
  avgX = rawAvgX / ACCEL_SCALE;
  avgY = rawAvgY / ACCEL_SCALE;
  avgZ = rawAvgZ / ACCEL_SCALE;
}

// Function to get FIFO count
uint16_t getFifoCount() {

}

void dataWrite(uint8_t REG, uint8_t val){
  Wire.beginTransmission(MPU);
  Wire.write(REG);
  Wire.write(val);
  Wire.endTransmission(true);
}
