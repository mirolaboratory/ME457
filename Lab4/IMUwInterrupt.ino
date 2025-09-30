#define INT_PIN 2         // Pin connected to MPU-6050 interrupt pin
volatile bool dataReady = false;  // Flag to indicate data ready

void setup() {
  // YOU NEED TO INSERT BASIC SETUPS FOR INITIALIZATION AND INTERRUPT
  // Set up the external interrupt (on pin 2 in this case)
  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), myISR, RISING);
}
void loop() {
  if (dataReady) {  // Check if data is ready (set by the ISR)
  }
}

// Interrupt Service Routine (ISR) triggered when data is ready
void myISR() {
  dataReady = true;  // Set the flag to indicate new data is available
}

