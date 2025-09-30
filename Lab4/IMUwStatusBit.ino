#define INT_STATUS 0x3A    // register we will poll

void loop() {
    // Request the INT_STATUS register to check for data ready interrupt
    Wire.beginTransmission(MPU);
    Wire.write(INT_STATUS);  // Point to the INT_STATUS register
    Wire.endTransmission(false);  // Send the request, but keep the connection active
    Wire.requestFrom(MPU, 1);  // Request 1 byte from INT_STATUS register

    // Wait for the data to be available
    if (Wire.available()) {
        uint8_t intStatus = Wire.read();  // Read the INT_STATUS register
        
        
        if (condition) {// Check if the DATA_RDY flag (bit 0) is set, indicating new data is ready

	}
    }
}
}

