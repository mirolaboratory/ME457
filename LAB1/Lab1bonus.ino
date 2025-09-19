// Lab: Digital pin as source + ADC read of voltage divider
// Modified to output timestamp,voltage for MATLAB plotting
// Board supply assumed ~5.00 V; update VREF_VOLTS if using 3.3 V board.

const byte SRC_PIN = 9;      // digital output used as V_IN
const byte ADC_PIN = A1;     // analog input measuring V_OUT
const float VREF_VOLTS = 5.00; // DEFAULT reference ≈ Vcc (adjust if 3.3 V board)

// Variables that need to be declared
unsigned long startTime;     // Store the start time
bool firstLoop = true;       // Initialize to true
const uint16_t phaseL = 1000; // 1 second
const uint16_t phaseH = 500; // 0.5 second

void setup() {
  pinMode(SRC_PIN, OUTPUT);
  digitalWrite(SRC_PIN, LOW);
  //analogReference(DEFAULT); // uses Vcc on Uno
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("Format: timestamp,voltage");
  delay(2000); // Give MATLAB time to start
}

void loop() {
  if (firstLoop) {
    startTime = millis();  // Timestamp starts exactly when measurements begin
    firstLoop = false;
  }
  
  // Low phase
  digitalWrite(SRC_PIN, LOW);
  delay(10);
  readPhase(phaseL);

  // HIGH phase
  digitalWrite(SRC_PIN, HIGH);
  delay(10);
  readPhase(phaseH);
}
  void readPhase(uint16_t phase){
  // Keep printing during HIGH phase for phaseH second
  unsigned long phaseStartTime = millis();
  while (millis() - phaseStartTime < phase) {
    unsigned long currentTime = millis() - startTime;
    int currentRaw = analogRead(ADC_PIN);
    float currentV = currentRaw * (VREF_VOLTS / 1023.0);
    
    Serial.print(currentTime);
    Serial.print(",");
    Serial.println(currentV, 4);
    
    delay(50); // Small delay to control print rate
  }
}
