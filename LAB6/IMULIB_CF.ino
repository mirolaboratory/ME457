#include "I2Cdev.h"
#include "MPU6050.h"

// Scaling factors from MPU6050 datasheet
#define ACCEL_SCALE_MODIFIER_2G 16384.0  // LSB/g for ±2g range
#define GYRO_SCALE_MODIFIER_250 131.0    // LSB/(°/s) for ±250°/s range

// Complementary filter coefficient (0.98 = 98% gyro, 2% accel)
// Higher value = more gyro influence (smooth but drifts)
// Lower value = more accel influence (corrects drift but noisy)
#define COMPLEMENTARY_FILTER_ALPHA 0.98

// Define angle modes as constants
#define ROLL 0
#define PITCH 1
#define ANGLE_MODE ROLL  // Change to PITCH to measure pitch instead

// MPU6050 object with I2C address 0x68
MPU6050 mpu(0x68);

// Raw sensor readings from MPU6050
int16_t ax, ay, az;  // Accelerometer (X, Y, Z)
int16_t gx, gy, gz;  // Gyroscope (X, Y, Z)

// Angle variables - only compile the ones needed based on ANGLE_MODE
#if ANGLE_MODE == ROLL
    float roll = 0.0;      // Filtered roll angle (complementary filter output)
    float gyroRoll = 0.0;  // Integrated gyroscope roll angle (for comparison)
#elif ANGLE_MODE == PITCH
    float pitch = 0.0;       // Filtered pitch angle (complementary filter output)
    float gyroPitch = 0.0;   // Integrated gyroscope pitch angle (for comparison)
#endif

// Timestamp for calculating time delta between samples
static unsigned long lastTime = 0;

void setup() {
    // Initialize I2C communication
    Wire.begin();
    
    // Initialize serial communication for data output
    Serial.begin(115200);
    
    // Initialize MPU6050 sensor
    mpu.initialize();
    
    // Configure Digital Low Pass Filter (42 Hz bandwidth)
    // Helps reduce high-frequency noise from accelerometer and gyroscope
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    
    // Set sample rate divider (Rate = 1kHz / (1 + 4) = 200 Hz)
    mpu.setRate(4);
    
    // Apply calibration offsets (obtained from IMU_ZERO.ino calibration)
    // These compensate for manufacturing variations and sensor bias
    mpu.setXAccelOffset(1462);   // X-axis accelerometer offset
    mpu.setYAccelOffset(20075);  // Y-axis accelerometer offset
    mpu.setZAccelOffset(3372);   // Z-axis accelerometer offset
    mpu.setXGyroOffset(93);      // X-axis gyroscope offset
    mpu.setYGyroOffset(-42);     // Y-axis gyroscope offset
    mpu.setZGyroOffset(44);      // Z-axis gyroscope offset
    
    // Wait for sensor to stabilize after configuration
    delay(500);
    
    // Initialize angles from accelerometer reading
    // This gives the filter a correct starting point
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert raw accelerometer values to g's
    float accelX = ax / ACCEL_SCALE_MODIFIER_2G;
    float accelY = ay / ACCEL_SCALE_MODIFIER_2G;
    float accelZ = az / ACCEL_SCALE_MODIFIER_2G;
    
    #if ANGLE_MODE == ROLL
        // Calculate initial roll from accelerometer
        // Roll: rotation around X-axis, uses Y and Z components
        roll = atan2(accelY, accelZ) * 180 / PI;
        gyroRoll = roll;  // Start gyro integration from same angle
    #elif ANGLE_MODE == PITCH
        // Calculate initial pitch from accelerometer
        // Pitch: rotation around Y-axis, uses all three components
        pitch = atan2(-accelX, sqrt(pow(accelY,2)+pow(accelZ,2))) * 180 / PI;
        gyroPitch = pitch;  // Start gyro integration from same angle
    #endif
    
    // Record initial timestamp
    lastTime = millis();
}

void loop() {
    // Check if new data is available from MPU6050
    // getIntStatus() returns true when data ready interrupt is set
    if (mpu.getIntStatus() != false) {
        
        // Read all 6 sensor values at once (atomic read)
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Calculate time elapsed since last reading (in seconds)
        // This is needed for integrating gyroscope rates into angles
        float dt = (millis() - lastTime) / 1000.0;
        lastTime = millis();
        
        // Convert raw accelerometer values to physical units (g's)
        float accelX = ax / ACCEL_SCALE_MODIFIER_2G;
        float accelY = ay / ACCEL_SCALE_MODIFIER_2G;
        float accelZ = az / ACCEL_SCALE_MODIFIER_2G;
        
        // Convert raw gyroscope values to physical units (°/s)
        float gyroX = gx / GYRO_SCALE_MODIFIER_250;
        float gyroY = gy / GYRO_SCALE_MODIFIER_250;
        float gyroZ = gz / GYRO_SCALE_MODIFIER_250;
        
        #if ANGLE_MODE == ROLL
            // Calculate roll angle from accelerometer
            // Uses arctangent of Y/Z to find rotation around X-axis
            // Accurate but noisy, affected by linear acceleration
            float accelRoll = atan2(accelY, accelZ) * 180 / PI;
            
            // Integrate gyroscope to get roll angle change
            // Smooth but drifts over time due to bias and integration error
            gyroRoll += gyroX * dt;
            
            // Apply complementary filter
            // Combines gyro (short-term accuracy) with accel (long-term stability)
            // Formula: filtered = alpha * (previous + gyro_change) + (1-alpha) * accel
            roll = COMPLEMENTARY_FILTER_ALPHA * (roll + gyroX * dt) + 
                   (1 - COMPLEMENTARY_FILTER_ALPHA) * accelRoll;
            
            // Output: accelRoll, gyroRoll, filtered_roll
            Serial.print(accelRoll); Serial.print(",");
            Serial.print(gyroRoll); Serial.print(",");
            Serial.println(roll);
            
        #elif ANGLE_MODE == PITCH
            // Calculate pitch angle from accelerometer
            // Uses all three axes to handle gimbal lock better
            // Negative accelX because pitch is rotation around Y-axis
            float accelPitch = atan2(-accelX, sqrt(pow(accelY,2)+pow(accelZ,2))) * 180 / PI;
            
            // Integrate gyroscope to get pitch angle change
            // Y-axis gyro measures pitch rate
            gyroPitch += gyroY * dt;
            
            // Apply complementary filter to combine both measurements
            pitch = COMPLEMENTARY_FILTER_ALPHA * (pitch + gyroY * dt) + 
                    (1 - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;
            
            // Output: accelPitch, gyroPitch, filtered_pitch
            Serial.print(accelPitch); Serial.print(",");
            Serial.print(gyroPitch); Serial.print(",");
            Serial.println(pitch);
        #endif
        
        // Small delay to prevent overwhelming serial output
        // Also helps maintain consistent sample rate (~100 Hz)
        delay(10);
    }
}