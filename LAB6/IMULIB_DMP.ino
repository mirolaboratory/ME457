#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Kalman.h"

// Scaling factors from MPU6050 datasheet
#define ACCEL_SCALE_MODIFIER_2G 16384.0  // LSB/g for ±2g range
#define GYRO_SCALE_MODIFIER_250 131.0    // LSB/(°/s) for ±250°/s range

#define INTERRUPT_PIN 2

// Complementary filter coefficient (0.98 = 98% gyro, 2% accel)
#define COMPLEMENTARY_FILTER_ALPHA 0.98

// MPU6050 object with I2C address 0x68
MPU6050 mpu(0x68);

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;                            // [w, x, y, z] Quaternion container
VectorInt16 aa;                          // [x, y, z] Accel sensor measurements
VectorInt16 gy;                          // [x, y, z] Gyro sensor measurements
VectorFloat gravity;                     // [x, y, z] Gravity vector
float ypr[3];                            // [yaw, pitch, roll] container

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Complementary filter variables
float roll = 0.0;                        // Filtered roll angle
float pitch = 0.0;                       // Filtered pitch angle
float gyroRoll = 0.0;                    // Integrated gyro roll (for comparison)
float gyroPitch = 0.0;                   // Integrated gyro pitch (for comparison)

// Timing variable
static unsigned long lastTime = 0;

void setup() {
    // Initialize I2C bus
    Wire.begin();
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial);  // Wait for serial port to connect 
    
    // Initialize MPU6050
    mpu.initialize();
// Reset all offsets
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

    // Apply calibration offsets (from IMU_ZERO.ino)
    mpu.setXAccelOffset(1462);
    mpu.setYAccelOffset(20075);
    mpu.setZAccelOffset(3372);
    mpu.setXGyroOffset(93);
    mpu.setYGyroOffset(-42);
    mpu.setZGyroOffset(44);
    
    // Initialize and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();
    
    // Make sure DMP initialization was successful (returns 0 if so)
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        DMPReady = true;
        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("DMP ready!"));
        // Initialize timestamp
        lastTime = millis();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    delay(500);
}

void loop() {
    if (!DMPReady) return; // Stop the program if DMP programming fails.
    
    // Check if DMP data is available
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        
        // ===== DMP ANGLES (from quaternion) =====
        // Get quaternion from DMP
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        // Calculate gravity vector
        mpu.dmpGetGravity(&gravity, &q);
        // Get Yaw/Pitch/Roll angles
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Convert from radians to degrees
        float dmpRoll = ypr[2] * 180 / M_PI;
        float dmpPitch = ypr[1] * 180 / M_PI;
        
        Serial.print("DMP angles\t");
        Serial.print(dmpRoll);
        Serial.print("\t");
        Serial.print(dmpPitch);
        
        // ===== COMPLEMENTARY FILTER ANGLES =====
        // Get raw sensor data
        mpu.dmpGetAccel(&aa, FIFOBuffer);
        mpu.dmpGetGyro(&gy, FIFOBuffer);
        // Calculate time elapsed (in seconds)
        float dt = (millis() - lastTime) / 1000.0;
        lastTime = millis();
        
        // Convert raw accelerometer values to physical units (g's)
        float accelX = aa.x / ACCEL_SCALE_MODIFIER_2G;
        float accelY = aa.y / ACCEL_SCALE_MODIFIER_2G;
        float accelZ = aa.z / ACCEL_SCALE_MODIFIER_2G;
        
        // Convert raw gyroscope values to physical units (°/s)
        float gyroX = gy.x / GYRO_SCALE_MODIFIER_250;
        float gyroY = gy.y / GYRO_SCALE_MODIFIER_250;
        float gyroZ = gy.z / GYRO_SCALE_MODIFIER_250;
        
        // Calculate roll angle from accelerometer
        float accelRoll = atan2(accelY, accelZ) * 180 / PI;
        
        // Integrate gyroscope for roll
        gyroRoll += gyroX * dt;
        
        // Apply complementary filter for roll
        roll = COMPLEMENTARY_FILTER_ALPHA * (roll + gyroX * dt) + 
               (1 - COMPLEMENTARY_FILTER_ALPHA) * accelRoll;
        
        // Calculate pitch angle from accelerometer
        float accelPitch = atan2(accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
        
        // Integrate gyroscope for pitch
        gyroPitch += gyroY * dt;
        
        // Apply complementary filter for pitch
        pitch = COMPLEMENTARY_FILTER_ALPHA * (pitch + gyroY * dt) + 
                (1 - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;
        
        Serial.print("\tCF angles\t");
        Serial.print(roll);
        Serial.print("\t");
        Serial.println(pitch);
        
        delay(10);
    } 
}