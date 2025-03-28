

#define MotorDirection 7
#define MotorSpeed     6//pwm
#define PotPin         A1

// We'll store the setpoint from Python here
float cameraSetpoint = 0.0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100;

void setup() {
  Serial.begin(115200);//9600//115200 //baud rate must match in serial monitor

  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  // Initialize motor off
  digitalWrite(MotorDirection, LOW);
  analogWrite(MotorSpeed, 0);
}

void loop() {
  // 1) Read any incoming setpoint from Python
  if (Serial.available() > 0) {
    float newSetpoint = Serial.parseFloat();
    if (!isnan(newSetpoint)) {
      cameraSetpoint = newSetpoint;
    }
  }

  // 2) Read the potentiometer (0..1023) -> 0..1
  int potRaw = analogRead(PotPin);
  float potVal = (float)potRaw / 1023.0;
  //IS 3.3V OR 5V INPUT USED??
  // Convert the analog reading (0 - 1023) to voltage (0 - 5V)
  //voltage = potValue * (3.3 / 1023.0) *1.5; //output when input voltage is 3.3 yiels position of 2/3
  //Calculate wiper position 
  //wiperPosition = 10 * (voltage / 3.3) ;//scale in terms of length (cm)

  // 3) Compute a simple difference (error = setpoint - pot)
  float error = cameraSetpoint - potVal;

  // 4) Motor speed is proportional to |error|
  //    Scale up to 0..255 for PWM. 
  //    If error is big, speed is high; if error is small, speed is low.
  float motorSpeedValue = fabs(error) * 255.0;
  if (motorSpeedValue > 255) {
    motorSpeedValue = 255; 
  }

  // 5) Motor direction depends on sign of error
  if (error >= 0) {
    digitalWrite(MotorDirection, HIGH); // forward
  } else {
    digitalWrite(MotorDirection, LOW);  // reverse
  }

  // 6) Write PWM
  analogWrite(MotorSpeed, (int)motorSpeedValue);

  // 7) Print status every 100 ms
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    Serial.print("Setpoint: ");
    Serial.print(cameraSetpoint, 3);
    Serial.print(" | Pot: ");
    Serial.print(potVal, 3);
    Serial.print(" | Error: ");
    Serial.print(error, 3);
    Serial.print(" | PWM: ");
    Serial.println((int)motorSpeedValue);
    lastPrintTime = currentTime;
  }

  //delay(20); // small delay ~50 Hz loop
}
