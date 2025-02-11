/****************************************************************************** 
  This sketch demonstrates reading Euler angles (heading, roll, pitch) + 
  linear potentiometer from an Arduino. It outputs 8 CSV values:

    heading, roll, pitch, sysCal, gyroCal, accelCal, magCal, potValue

  Usage:
    - Press 'z' (or 'Z') in Serial Monitor to zero (tare) the BNO055 angles.
    - This data is meant to be read by a Python script for live robot animation.
******************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------------- BNO055 Setup ----------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// We'll store offsets in these variables to "zero" the sensor
float headingOffset = 0.0;
float rollOffset    = 0.0;
float pitchOffset   = 0.0;
// Variables to store voltage and wiper position
float voltage;
float wiperPosition;

// ---------------------- Potentiometer Setup ----------------------
const int potPin = A1;  // Analog input pin for the linear potentiometer

void setup() {
  // Start Serial at 115200
  Serial.begin(115200); //baud rate must match in serial monitor
  while (!Serial) {
    // Needed on some boards to wait for Serial
  }

  Serial.println("BNO055 + Linear Potentiometer Demo");
  Serial.println("----------------------------------");
  Serial.println("Type 'z' in Serial Monitor to zero (tare) the BNO055 angles.");
  Serial.println("Output format: heading,roll,pitch,sys,gyro,accel,mag,potValue\n");

  // Initialize the BNO055
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check wiring/I2C address!");
    while (1);
  }

  // Optional: Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  // Wait a moment for the device to be fully started
  delay(1000);

  Serial.println("BNO055 initialized successfully!");
  Serial.println("Move sensor gently in all directions to calibrate.\n");
}

void loop() {
  // ---------------------- Check for 'z' to zero angles ----------------------
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == 'z' || incomingChar == 'Z') {
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
      Serial.println("---------------------------------------\n");
    }
  }

  // ---------------------- Read Orientation from BNO055 ----------------------
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float rawHeading = orientationData.orientation.x; // in degrees
  float rawRoll    = orientationData.orientation.y;
  float rawPitch   = orientationData.orientation.z;

  // Apply offsets
  float heading = rawHeading - headingOffset;
  float roll    = rawRoll    - rollOffset;
  float pitch   = rawPitch   - pitchOffset;

  // ---------------------- Get Calibration Status ----------------------
  uint8_t sysCal, gyroCal, accelCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

  // ---------------------- Read Potentiometer ----------------------
  // (raw value 0..1023). You can scale to mm/voltage if desired
  int potValue = analogRead(potPin);
  // Convert the analog reading (0 - 1023) to voltage (0 - 5V)
  voltage = potValue * (3.3 / 1023.0) *1.5; //output when input voltage is 3.3 yiels position of 2/3
 //Calculate wiper position as a percentage
 wiperPosition = 10 * (voltage / 3.3) ;//scale in terms of length
  // ---------------------- Print CSV Output ----------------------
  // Format: heading,roll,pitch,sys,gyro,accel,mag,potValue
  Serial.print(heading);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(sysCal);
  Serial.print(",");
  Serial.print(gyroCal);
  Serial.print(",");
  Serial.print(accelCal);
  Serial.print(",");
  Serial.print(magCal);
  Serial.print(",");
  Serial.println(wiperPosition);

  delay(100);  // 10 Hz update (adjust as needed)
}
