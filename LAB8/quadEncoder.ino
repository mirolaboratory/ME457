// Define encoder pin numbers
#define ENCODER_PIN_A 2   // Channel A (encoder)
#define ENCODER_PIN_B 3   // Channel B (encoder)

// Define motor control pins (DRV8833)
#define MOTOR_IN1 9       // Control pin 1
#define MOTOR_IN2 10      // Control pin 2

// --- ENCODER AND GEAR PARAMETERS ---
// From Pololu Spec: 12 CPR on the *motor shaft*
#define MOTOR_SHAFT_CPR 12.0
// From Pololu Spec: 51.45:1 gear ratio
#define GEAR_RATIO 51.45
// Output shaft CPR: (12.0 * 51.45) = 617.4
#define CPR (MOTOR_SHAFT_CPR * GEAR_RATIO)

// Allowable error in counts
#define tolerance 3
#define deadband 50  // Extra buffer to prevent oscillations

// Global variables
volatile int32_t encoderPosition = 0;  // Position of the encoder (in counts)
volatile int32_t lastEncoded = 0;      // Last encoded value to detect direction

int16_t targetAngle = 0;               // Target angle (set by Serial input)
int32_t targetCounts = 0;              // Target counts (calculated from target angle)
int pwmValue = 200;                    // Base PWM speed (0-255)

bool motorStopped = false;             // Flag to completely disable motor control

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Enter target angle (e.g., 90) or 'stop' to stop the motor");

  // Set encoder pins as input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Set motor control pins as output
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Attach interrupts to handle changes on channel A and B
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  // Stop the motor initially
  stopMotor();
}

void loop() {
  // Handle Serial input for stopping or setting target angle
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    parseSerialInput(inputString);
  }

  // If motor is stopped, keep it stopped and exit
  if (motorStopped) {
    stopMotor();
    return;  // Skip all motor control logic
  }

  // Calculate error (MUST be int32_t to handle large counts)
  int32_t error = targetCounts - encoderPosition;

  if (abs(error) <= tolerance) {
    // Within tolerance: Stop and brake
    brakeMotor();
  } 
  else if (abs(error) <= (tolerance + deadband)) {
    // In the "slow zone": Move at reduced speed
    int reducedPwm = max(pwmValue / 2, 70);  // 50% speed, minimum 70 to overcome friction
    
    if (error > 0) {
      // Forward motion at reduced speed
      analogWrite(MOTOR_IN1, reducedPwm);
      analogWrite(MOTOR_IN2, 0);
    } else {
      // Reverse motion at reduced speed
      analogWrite(MOTOR_IN1, 0);
      analogWrite(MOTOR_IN2, reducedPwm);
    }
  } 
  else {
    // Far from target: Move at normal speed
    if (error > 0) {
      // Forward motion
      analogWrite(MOTOR_IN1, pwmValue);
      analogWrite(MOTOR_IN2, 0);
    } 
    else if (error < 0) {
      // Reverse motion
      analogWrite(MOTOR_IN1, 0);
      analogWrite(MOTOR_IN2, pwmValue);
    }
  }

  // Print position only when it changes significantly (reduce Serial spam)
  static int32_t lastPrintedPos = -999999;
  if (abs(encoderPosition - lastPrintedPos) > 5) {
    Serial.print("Target: ");
    Serial.print(targetCounts);
    Serial.print(" | Current: ");
    Serial.print(encoderPosition);
    Serial.print(" | Error: ");
    Serial.println(error);
    lastPrintedPos = encoderPosition;
  }
  
  delay(5);  // Small delay for smoother control
}

void parseSerialInput(String input) {
  input.trim();
  
  if (input.equalsIgnoreCase("stop")) {
    // Stop the motor completely
    motorStopped = true;
    targetCounts = encoderPosition;  // Set target to current position
    stopMotor();
    Serial.println("Motor stopped by user command.");
  } 
  else {
    // Set new target angle
    motorStopped = false;  // Re-enable motor control
    targetAngle = input.toInt();
    
    // Calculate target counts using floating-point division for precision
    targetCounts = (CPR * targetAngle) / 360.0;
    
    Serial.print("Target Angle Set: ");
    Serial.print(targetAngle);
    Serial.print(" degrees (Target Counts: ");
    Serial.print(targetCounts);
    Serial.println(")");
  }
}

// Function to stop the motor (coast - no braking)
void stopMotor() {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
}

// Function to apply active braking
void brakeMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// Interrupt service routine (ISR) to handle encoder changes
void updateEncoder() {
  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)
  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

  uint8_t encoded = (MSB << 1) | LSB;         // Combine both channels to get a 2-bit value
  uint8_t sum = (lastEncoded << 2) | encoded; // Form a 4-bit combination of last and current state

  // Quadrature decoding logic to update position
if (sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || sum == 0b1101) encoderPosition--; 
if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) encoderPosition++;

  lastEncoded = encoded;  // Save current state for next interrupt
}
