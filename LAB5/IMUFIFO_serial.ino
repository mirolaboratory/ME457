#include <Wire.h>

// MPU6050 Register Addresses
#define MPU 0x68
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV_REG  0x19
#define CONFIG_REG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define USER_CTRL       0x6A
#define FIFO_EN         0x23
#define FIFO_COUNTH     0x72
#define FIFO_COUNTL     0x73
#define FIFO_R_W        0x74

// Buffer Configuration
#define NUM_TIMESTAMPS 5        // Number of timestamps (5 timestamps for 10 samples)
#define BYTES_PER_READ 24       // Read 24 bytes at once (2 samples × 12 bytes)
#define BYTES_PER_PACKET 28     // Timestamp(4) + 2 Samples(24) = 28 bytes
#define BUFFER_SIZE 141         // Start marker(1) + 5 × 28 bytes = 141

// Global Variables
uint8_t dataBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
unsigned long startTime = 0;
bool firstLoop = true;

// Function Prototypes
uint16_t getFifoCount();
void dataWrite(uint8_t reg, uint8_t val);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication
  
  // Wake up MPU6050
  dataWrite(PWR_MGMT_1, 0x00);
  delay(100);
  
  // Configure accelerometer (±2g range)
  dataWrite(ACCEL_CONFIG, 0x00);
  
  // Configure gyroscope (±250°/s range)
  dataWrite(GYRO_CONFIG, 0x00);
  
  // Configure sample rate: 100 Hz
  // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
  // 1000 Hz / (1 + 9) = 100 Hz
  dataWrite(CONFIG_REG, 0x03);      // DLPF_CFG = 3 (Gyro: 1kHz, Accel: 1kHz)
  dataWrite(SMPLRT_DIV_REG, 0x09);  // Sample rate divider = 9
  
  // Reset FIFO
  dataWrite(USER_CTRL, 0x04);
  delay(10);
  
  // Enable FIFO
  dataWrite(USER_CTRL, 0x40);
  
  // Enable accelerometer and gyroscope data to FIFO
  // Bit 3: ACCEL_FIFO_EN, Bits 6-4: Gyro XYZ
  dataWrite(FIFO_EN, 0x78);
  
  delay(100);
}

void loop() {
  // Initialize start time on first loop
  if (firstLoop) {
    startTime = millis();
    firstLoop = false;
  }

  // Get FIFO count
  uint16_t fifoCount = getFifoCount();

  // Get current timestamp
  unsigned long currentTime = millis() - startTime;
  
  // Process FIFO data when we have at least 24 bytes (2 complete samples)
  while (fifoCount >= BYTES_PER_READ) {
    // Add start marker at the beginning of a new buffer
    if (bufferIndex == 0) {
      dataBuffer[bufferIndex++] = 0xFF;
    }
    
    // Store timestamp (4 bytes, big-endian format)
    dataBuffer[bufferIndex++] = (currentTime >> 24) & 0xFF;
    dataBuffer[bufferIndex++] = (currentTime >> 16) & 0xFF;
    dataBuffer[bufferIndex++] = (currentTime >> 8) & 0xFF;
    dataBuffer[bufferIndex++] = currentTime & 0xFF;
    
    // Read 24 bytes from FIFO (2 complete samples)
    Wire.beginTransmission(MPU);
    Wire.write(FIFO_R_W);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, BYTES_PER_READ, true);
    
    // Store 24 bytes of sensor data
    for (uint8_t i = 0; i < BYTES_PER_READ; i++) {
      if (bufferIndex < BUFFER_SIZE && Wire.available()) {
        dataBuffer[bufferIndex++] = Wire.read();
      }
    }
    
    // Send buffer when full (5 timestamps + 10 samples = 141 bytes)
    if (bufferIndex >= BUFFER_SIZE) {
      Serial.write(dataBuffer, BUFFER_SIZE);
      bufferIndex = 0;
    }
    
    // Update FIFO count
    fifoCount -= BYTES_PER_READ;
  }
}

// Get FIFO count (number of bytes currently in FIFO)
uint16_t getFifoCount() {
  Wire.beginTransmission(MPU);
  Wire.write(FIFO_COUNTH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)2, true);
  
  uint16_t count = ((uint16_t)Wire.read() << 8) | Wire.read();
  return count;
}

// Write data to MPU6050 register
void dataWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}
