// This program is from https://mjwhite8119.github.io/Robots/mpu6050
// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int acel_deadzone=8;      //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int gyro_deadzone=1;      //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// This is the ideal raw sensor value for 1g acceleration on the Z-axis when flat.
// It depends on the accelerometer's Full Scale Range (FSR).
// The MPU-6050 library defaults to +/- 2g FSR.
//   For +/- 2g  (default): 1g = 32768 / 2 = 16384
//   For +/- 4g:          1g = 32768 / 4 = 8192
//   For +/- 8g:          1g = 32768 / 8 = 4096
//   For +/- 16g:         1g = 32768 / 16 = 2048
//
// If you change the FSR in setup() (e.g., mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);),
// you MUST change this value to match!
const int ACCEL_TARGET_1G = 16384;
//const int ACCEL_TARGET_1G = 8192;
//const int ACCEL_TARGET_1G = 4096;
//const int ACCEL_TARGET_1G = 2048;

MPU6050 mpu(0x68); 

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

int mpuConnection;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);

  // initialize MPU-6050
  mpu.initialize(); 
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  mpu.setSleepEnabled(false); 
  mpu.setRate(4); 
  mpu.setDLPFMode(1); 
  
  // *** OPTIONAL: Set Full Scale Range ***
  // Uncomment one of the following lines to set a different FSR.
  // Make sure to update ACCEL_TARGET_1G in the CONFIGURATION section if you do!
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // Default
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  
  Serial.println("Place the MPU-6050 breakout board in a flat or horizontal position, with SMD components facing up.\n");
  Serial.println(F("Type in any character and press Enter/Send to start MPU-6050 calibration..."));
  
  while (Serial.available() == 0){ } //wait for character to be entered               
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nStarting MPU-6050 Calibration Sketch.");
  delay(1000);
  Serial.println("\nDon't touch the MPU-6050 until you see a \"FINISHED!\" message.");
  delay(2000);
  
  // verify connection
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nVerifying MPU-6050 connection...");
  delay(500);

  mpuConnection = mpu.testConnection(); 
  if (mpuConnection == 0) {
    Serial.println("\nMPU-6050 connection FAILED.");
    Serial.println("\nCheck your boards and connections. Reset the Arduino board to run the calibration sketch again.");
  }
  
  while (mpuConnection == 0) { }  //wait for MPU-6050 connection to be established. 

  Serial.println("\nMPU-6050 connection SUCCESSFUL!");
  delay(500);
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nResetting MPU-6050 offsets.");
  
  // reset offsets
  mpu.setXAccelOffset(0); 
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0); 
  mpu.setXGyroOffset(0); 
  mpu.setYGyroOffset(0); 
  mpu.setZGyroOffset(0);

  delay(500);
}

void loop() {
  if (state==0){
    Serial.println("\nReading accelerometer and gyroscope sensors for first time.");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.println("\n==============================================================");
    Serial.println("\nRESULTS:");
    Serial.println("\nSensor data is listed in the format:\tacelX\tacelY\tacelZ\tgyroX\tgyroY\tgyroZ");
    Serial.print("\nSensor readings INCLUDING offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    
    Serial.print("\nCompare with IDEAL sensor readings:\t0\t0\t");
    Serial.print(ACCEL_TARGET_1G); // Use the constant here
    Serial.println("\t0\t0\t0");
    Serial.println("\n--------------------------------------------------------------");
    
    Serial.println("\nYour MPU-6050 offsets:\n");
    Serial.print("mpu.setXAccelOffset(");
    Serial.print(ax_offset);
    Serial.println(");");
    Serial.print("mpu.setYAccelOffset(");
    Serial.print(ay_offset);
    Serial.println(");");
    Serial.print("mpu.setZAccelOffset(");
    Serial.print(az_offset);
    Serial.println(");");
    Serial.print("mpu.setXGyroOffset(");
    Serial.print(gx_offset);
    Serial.println(");");
    Serial.print("mpu.setYGyroOffset(");
    Serial.print(gy_offset);
    Serial.println(");");
    Serial.print("mpu.setZGyroOffset(");
    Serial.print(gz_offset);
    Serial.println(");");
    
    Serial.println("\nYou can copy and paste the above offsets directly into your sketch. :)");
    
    while (1);
  }
}

void meansensors() {
    // Re-initialize mean values to 0 each time this function is called
    mean_ax = 0;
    mean_ay = 0;
    mean_az = 0;
    mean_gx = 0;
    mean_gy = 0;
    mean_gz = 0;
    int validReadings = 0;   // Counter for the number of valid readings taken

    for (int i = 0; i < 1000 + 100; i++) {
        // Wait until the interrupt status indicates that data is ready
        while (!mpu.getIntStatus()) { 
            // Do nothing, just wait until data is ready
        }

        // Read raw accel/gyro measurements from the device
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 

        // If the reading is after the initial discard period, use it in the averaging
        if (i >= 100) {  // Discard the first 100 readings
            // Update mean values using recursive averaging
            validReadings++;   // Increment the count of valid readings
            
            // Note: This is an implementation of Welford's online algorithm for mean
            mean_ax = mean_ax + (ax - mean_ax) / validReadings;
            mean_ay = mean_ay + (ay - mean_ay) / validReadings;
            mean_az = mean_az + (az - mean_az) / validReadings;
            mean_gx = mean_gx + (gx - mean_gx) / validReadings;
            mean_gy = mean_gy + (gy - mean_gy) / validReadings;
            mean_gz = mean_gz + (gz - mean_gz) / validReadings;
        }
    }
}


void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(ACCEL_TARGET_1G - mean_az)/8; // Use the constant here

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;

 bool ready[6] = {false, false, false, false, false, false};
  while (!allReady(ready, 6)){
    mpu.setXAccelOffset(ax_offset); 
    mpu.setYAccelOffset(ay_offset); 
    mpu.setZAccelOffset(az_offset); 

    mpu.setXGyroOffset(gx_offset); 
    mpu.setYGyroOffset(gy_offset); 
    mpu.setZGyroOffset(gz_offset); 

    // Re-read sensors with new offsets applied
    meansensors();
    Serial.print("...");

    if (abs(mean_ax)<=acel_deadzone) ready[0] = true;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready[1] = true;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(ACCEL_TARGET_1G - mean_az)<=acel_deadzone) ready[2] = true; // Use the constant here
    else az_offset=az_offset+(ACCEL_TARGET_1G - mean_az)/acel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready[3] = true;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone + 1);

    if (abs(mean_gy)<=gyro_deadzone) ready[4] = true;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone + 1);

    if (abs(mean_gz)<=gyro_deadzone) ready[5] = true;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone + 1);

    printReadyArray(ready, 6);
  }
}

    bool allReady(bool ready[], int size) {
    for (int i = 0; i < size; i++) {
        if (!ready[i]) return false; // If any element is false, return false immediately
    }
    return true; // All elements are true
}

void printReadyArray(bool ready[], int size) {
    Serial.print("Ready status: ");
    for (int i = 0; i < size; i++) {
        Serial.print(ready[i]);
        Serial.print(" "); // Add space between values
    }
    Serial.println(); // New line after the array
}
