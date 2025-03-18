#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ---------------------- Pin Definitions ----------------------
#define MotorDirection 7
#define MotorSpeed     6  // PWM pin
#define PotPin         A1 // Linear potentiometer input

// ---------------------- BNO055 Setup ----------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Offsets for zeroing BNO055 angles
float headingOffset = 0.0;
float rollOffset    = 0.0;
float pitchOffset   = 0.0;

// ---------------------- Global Kinematics Variables ----------------------
const float l01A = 261 + 11.3;   // 272.3 mm
const float d01  = 0.0;
const float l01B = 0.0;
const float l45A = 91.0;
const float l45B = 0.0;
const float l56A = 20.0;
const float l56B = 205 + 24.34;   // 229.34 mm

// Base offset (BRF_pt from Python code)
const float BRF_pt[3] = {67.0, 296.5, 0.0};

// Fixed heart point (in mm)
const float heartPt[3] = {347.5, 279.7 + 50, 175.0 - 25};

// Global dotPt starts at heartPt
float dotPt[3] = {347.5, 279.7, 175.0};

// Euclidean distance from tool to dotPt
float dist_tool_dot = 110.0;

// ---------------------- Timing Variables ----------------------
unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 75; // Increase sampling rate (20ms ~ 50 Hz)

// ---------------------- PID & Filter Controller Variables ----------------------
// PID Tuning Parameters - CHANGE THESE VALUES TO TUNE THE CONTROLLER:
float Kp = 2.0;    // Proportional gain
float Ki = 0.05;    // Integral gain
float Kd = 0.05;   // Derivative gain

// Low-Pass Filter Coefficients - CHANGE THESE TO TUNE FILTER RESPONSE:
const float alphaGyro  = 0.2; // Gyro lowpass filter constant (0 < alpha < 1)
const float alphaDeriv = 0.1; // Derivative lowpass filter constant

const float maxIntegral = 100; // Anti-windup limit for the integral term

// Persistent variables for PID controller:
float integralSum = 0.0;
float lastError = 0.0;
float filteredDerivative = 0.0;  // For lowpass filtering the derivative
unsigned long lastControlTime = 0;

// Filtered gyroscope values (global so that they persist)
float filteredHeading = 0.0;
float filteredRoll    = 0.0;
float filteredPitch   = 0.0;
bool firstGyroReading = true;

// ---------------------- Helper Functions ----------------------
// Multiply two 4x4 matrices: C = A * B
void matMult(float A[4][4], float B[4][4], float C[4][4]) {
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      C[i][j] = 0;
      for (int k = 0; k < 4; k++){
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

// Compute standard DH transformation matrix T from parameters (a, alpha, d, theta)
void jointTransformation(float a, float alpha, float d, float theta, float T[4][4]) {
  T[0][0] = cos(theta);
  T[0][1] = -sin(theta) * cos(alpha);
  T[0][2] = sin(theta) * sin(alpha);
  T[0][3] = a * cos(theta);
  T[1][0] = sin(theta);
  T[1][1] = cos(theta) * cos(alpha);
  T[1][2] = -cos(theta) * sin(alpha);
  T[1][3] = a * sin(theta);
  T[2][0] = 0;
  T[2][1] = sin(alpha);
  T[2][2] = cos(alpha);
  T[2][3] = d;
  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;
}

void setup() {
  Serial.begin(115200); // Ensure baud rate matches your setup
  
  // ---------------------- Motor Setup ----------------------
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  digitalWrite(MotorDirection, LOW);
  analogWrite(MotorSpeed, 0);

  // ---------------------- BNO055 Setup ----------------------
  if (!bno.begin()) {
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO055 & Motor Control Ready.");
  
  lastControlTime = millis();
}

void loop() {
  // ---------------------- Serial Command Handling ----------------------
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("z") || line.startsWith("Z")) {
      sensors_event_t orientationData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      headingOffset = orientationData.orientation.x;
      rollOffset    = orientationData.orientation.y;
      pitchOffset   = orientationData.orientation.z;
      Serial.println("Zeroing angles at current orientation!");
      Serial.print("New Offsets -> Heading: ");
      Serial.print(headingOffset);
      Serial.print(", Roll: ");
      Serial.print(rollOffset);
      Serial.print(", Pitch: ");
      Serial.println(pitchOffset);
      Serial.println("---------------------------------------");
    } else {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        String xStr = line.substring(0, firstComma);
        String yStr = line.substring(firstComma + 1, secondComma);
        String zStr = line.substring(secondComma + 1);
        dotPt[0] = xStr.toFloat();
        dotPt[1] = yStr.toFloat();
        dotPt[2] = zStr.toFloat();
      }
    }
  }
  
  // ---------------------- Sensor Reading & Gyro Lowpass Filtering ----------------------
  int potValue = analogRead(PotPin);
  float potExtension = ((float)potValue / 1023.0) * 1.5 * 100.0; // in mm

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  // Convert raw sensor readings to radians (after zeroing)
  float rawHeading = (orientationData.orientation.x - headingOffset) * (PI / 180.0);
  float rawRoll    = (orientationData.orientation.y - rollOffset)    * (PI / 180.0);
  float rawPitch   = (orientationData.orientation.z - pitchOffset)   * (PI / 180.0);
  
  // Initialize filtered values on first run
  if (firstGyroReading) {
    filteredHeading = rawHeading;
    filteredRoll    = rawRoll;
    filteredPitch   = rawPitch;
    firstGyroReading = false;
  }
  
  // Low-pass filter the gyroscope (orientation) data:
  filteredHeading = (alphaGyro * rawHeading) + ((1 - alphaGyro) * filteredHeading);
  filteredRoll    = (alphaGyro * rawRoll)    + ((1 - alphaGyro) * filteredRoll);
  filteredPitch   = (alphaGyro * rawPitch)   + ((1 - alphaGyro) * filteredPitch);

  // ---------------------- Forward Kinematics ----------------------
  float T1[4][4], T2[4][4], T3[4][4], T4[4][4], T5[4][4], T6[4][4], T7[4][4];
  jointTransformation(0, 0, l01A, 0, T1);
  jointTransformation(0, PI/2.0, 0, filteredHeading, T2);
  jointTransformation(0, PI/2.0, 0, filteredPitch + (PI/2.0), T3);
  jointTransformation(0, 0, 0, filteredRoll, T4);
  jointTransformation(l45A, 0, 0, 0, T5);
  jointTransformation(0, 0, l45B, 0, T6);
  jointTransformation(0, 0, (potExtension + l56A + l56B), 0, T7);

  float T_temp1[4][4], T_temp2[4][4], T_temp3[4][4], T_temp4[4][4], T_temp5[4][4], T_total[4][4];
  matMult(T1, T2, T_temp1);
  matMult(T_temp1, T3, T_temp2);
  matMult(T_temp2, T4, T_temp3);
  matMult(T_temp3, T5, T_temp4);
  matMult(T_temp4, T6, T_temp5);
  matMult(T_temp5, T7, T_total);

  float p_ee[3];
  p_ee[0] = T_total[0][3] + BRF_pt[0];
  p_ee[1] = T_total[1][3] + BRF_pt[1];
  p_ee[2] = T_total[2][3] + BRF_pt[2];

  float dx = p_ee[0] - dotPt[0];
  float dy = p_ee[1] - dotPt[1];
  float dz = p_ee[2] - dotPt[2];
  dist_tool_dot = sqrt(dx * dx + dy * dy + dz * dz);

  // ---------------------- PID Controller Calculation with Filtered Derivative ----------------------
  // Desired distance (mm) between tool and target point
  const float desiredDistance = 25;
  
  // Calculate time elapsed since last control update (in seconds)
  unsigned long now = millis();
  float deltaTime = (now - lastControlTime) / 1000.0;
  if(deltaTime <= 0) deltaTime = 0.001;  // Avoid division by zero
  lastControlTime = now;
  
  // Calculate error between desired and actual distance
  float error = desiredDistance - dist_tool_dot;
  
  // Calculate raw derivative term (change in error over time)
  float rawDerivative = (error - lastError) / deltaTime;
  
  // Low-pass filter the derivative term:
  filteredDerivative = (alphaDeriv * rawDerivative) + ((1 - alphaDeriv) * filteredDerivative);
  
  // Update the integral term with anti-windup protection
  integralSum += error * deltaTime;
  integralSum = constrain(integralSum, -maxIntegral, maxIntegral);
  
  // Compute PID control output with scaling factor 3.5
  float controlOutput = 3.5 * ((Kp * error) + (Ki * integralSum) + (Kd * filteredDerivative));
  
  // Save the current error for the next derivative calculation
  lastError = error;
  
  // ---------------------- Motor Command ----------------------
  // Set motor direction based on the sign of the control output
  digitalWrite(MotorDirection, controlOutput < 0 ? HIGH : LOW);
  
  // Map control output magnitude to a PWM value (0-255)
  float motorSpeedValue = fabs(controlOutput);
  motorSpeedValue = (motorSpeedValue / 100.0) * 255;
  motorSpeedValue = constrain(motorSpeedValue, 0, 255);
  analogWrite(MotorSpeed, (int)motorSpeedValue);
  
  // ---------------------- Send Measurement Packet ----------------------
  unsigned long currentTime = millis();
  if (currentTime - lastMeasurementTime >= measurementInterval) {
    Serial.print("M: ");
    Serial.print("heading=");
    Serial.print(filteredHeading * (180.0 / PI));
    Serial.print(", roll=");
    Serial.print(filteredRoll * (180.0 / PI));
    Serial.print(", pitch=");
    Serial.print(filteredPitch * (180.0 / PI));
    Serial.print(", dist_tool_dot=");
    Serial.print(dist_tool_dot);
    Serial.print(", motorPWM=");
    Serial.print((int)motorSpeedValue);
    Serial.print(", error=");
    Serial.print(error);
    Serial.print(", pot_dist=");
    Serial.print(potExtension);
    Serial.print(", Desired_dist=");
    Serial.println(desiredDistance);
    lastMeasurementTime = currentTime;
  }
}
