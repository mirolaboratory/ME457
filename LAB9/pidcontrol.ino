void setup() {
  Serial.begin(115200);
  
 //MAX_INTEGRAL for anti-windup limit
  if (ki > 0) {
    // Want max i_term contribution around 150 PWM
    MAX_INTEGRAL = 150.0 / ki;
    
    // Reasonable bounds
    if (MAX_INTEGRAL < 0.5) MAX_INTEGRAL = 0.5;
    if (MAX_INTEGRAL > 50.0) MAX_INTEGRAL = 50.0;
  } else {
    MAX_INTEGRAL = 0.0;  // No integral term
  }
}

void pidControl(int32_t error) {
  // Time difference calculation
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);  
  prevT = currT;
  
  // Avoid division by zero or too small deltaT
  if (deltaT < 0.0002) deltaT = 0.0002; //(200 us)
  
  // Proportional term (always active)
  float p_term = kp * error;
  
  // Integral term (only for PI and PID modes)
  float i_term = 0.0;
  if (ki > 0) {  // PI or PID
    error_int = error_int + error * deltaT;
    
    // Anti-windup: Constrain integral term
    if (error_int > MAX_INTEGRAL) error_int = MAX_INTEGRAL;
    if (error_int < -MAX_INTEGRAL) error_int = -MAX_INTEGRAL;
    
    // Reset integral when crossing target (sign change)
    if ((error > 0 && error_prev < 0) || (error < 0 && error_prev > 0)) {
      error_int = 0;
    }
    
    i_term = ki * error_int;
  }
  
  // Derivative term (only for PID mode)
  float d_term = 0.0;
  if (kd > 0) {  // PID only
    float dedt = (error - error_prev) / deltaT;
    d_term = kd * dedt;
  }
  
  // Calculate total PID output
  float pid_pwm = p_term + i_term + d_term;
  
  // Store previous error
  error_prev = error;
  
  // Motor control based on error and PID output
  if (abs(error) <= tolerance) {
    stopMotor();
    reachedTarget = true;
    error_int = 0;  // Reset integral when at target
    pwmValue = 0;
  }
  else {
    reachedTarget = false;
    
    // Simple PWM constraint - let PID do its job
    pid_pwm = constrain(pid_pwm, -255, 255);
    
    // Ensure minimum PWM to overcome friction
    if (abs(pid_pwm) < min_pwm && abs(error) > tolerance) {
      if (pid_pwm > 0) {
          pid_pwm = min_pwm;
          } else {
          pid_pwm = -min_pwm;
          }
    }
    
    pwmValue = abs(pid_pwm);
    
    // Apply motor control
    if (pid_pwm > 0) {
      moveForward(pwmValue);
    } 
    else {
      moveBackward(pwmValue);
    }
  }
}
