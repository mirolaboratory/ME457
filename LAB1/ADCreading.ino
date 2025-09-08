// Simple demo: Digital pin as source, read voltage divider with ADC

const int SRC_PIN = 9;   // digital output source
const int ADC_PIN = A0;  // analog input reads divider
const float VREF = 5.0;  // assume Arduino Nano powered at 5 V

unsigned long startTime;     // Store the start time
bool firstLoop = true;       // Initialize to true
unsigned long currentTime; 

void setup() {
  pinMode(SRC_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {;}
}

void loop() {
	  if (firstLoop) {
    startTime = millis();  // Timestamp starts exactly when measurements begin
    firstLoop = false;
  }
  // Drive D9 LOW
  digitalWrite(SRC_PIN, LOW);
  delay(10);  // settle time
  currentTime = millis() - startTime;
  int adcLow = analogRead(ADC_PIN);
  float vLow = adcLow * VREF / 1023.0;
  Serial.print(currentTime);
  Serial.print(",");
  Serial.println(vLow, 3);

  delay(1000);

  // Drive D9 HIGH
  digitalWrite(SRC_PIN, HIGH);
  delay(10);
  currentTime = millis() - startTime;
  int adcHigh = analogRead(ADC_PIN);
  float vHigh = adcHigh * VREF / 1023.0;
  Serial.print(currentTime);
  Serial.print(",");
  Serial.println(vHigh, 3);

  delay(1000);
}
