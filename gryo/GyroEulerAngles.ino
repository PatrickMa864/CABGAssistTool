/****************************************************************************** 
  This sketch demonstrates:
    - Reading Euler angles (heading, roll, pitch) from the Adafruit BNO055
    - "Zeroing" (taring) angles on a Serial keyboard input ('z')
    - Printing the angles + calibration status (sys, gyro, accel, mag)
      in a CSV format suitable for MATLAB or other tools.

  Hardware:
    - BNO055 sensor
    - Arduino Mega 2560 (or other board)
    - I2C connections (SDA -> pin 20, SCL -> pin 21 on Mega)

  Libraries:
    - Adafruit_BNO055
    - Adafruit Unified Sensor
******************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create the BNO object
// If your ADR pin is tied LOW, the I2C address is 0x28.
// If your ADR pin is tied HIGH, the address is 0x29.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// We'll store offsets in these variables to "zero" the sensor
float headingOffset = 0.0;
float rollOffset    = 0.0;
float pitchOffset   = 0.0;

void setup() {
  // Start serial communication
  Serial.begin(115200);
  while (!Serial) {
    // Needed on some boards to wait for the Serial port
  }

  Serial.println("BNO055 Orientation + Calibration Demo");
  Serial.println("--------------------------------------");
  Serial.println("Type 'z' in Serial Monitor to zero (tare) the angles.");
  Serial.println("Output format: heading,roll,pitch,sys,gyro,accel,mag\n");

  // Initialize the BNO055
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C address!");
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
  // Check if there's any data in the Serial buffer
  if (Serial.available() > 0) {
    // Read one character
    char incomingChar = Serial.read();

    // If it's 'z', we set the current orientation as our zero reference
    if (incomingChar == 'z' || incomingChar == 'Z') {
      // Create a sensor event object
      sensors_event_t orientationData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

      // Store the current orientation as offsets
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

  // Read orientation from the BNO055
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // Get raw readings (in degrees)
  float rawHeading = orientationData.orientation.x;
  float rawRoll    = orientationData.orientation.y;
  float rawPitch   = orientationData.orientation.z;

  // Apply offsets to "zero" the angles
  float heading = rawHeading - headingOffset;
  float roll    = rawRoll    - rollOffset;
  float pitch   = rawPitch   - pitchOffset;

  // Read the calibration status
  uint8_t sysCal, gyroCal, accelCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

  // Print the angles + calibration in CSV format:
  // heading,roll,pitch,sys,gyro,accel,mag
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
  Serial.println(magCal);

  // Optional: If you want to only print once the system is fully calibrated:
  // if (sysCal == 3 && gyroCal == 3 && accelCal == 3 && magCal == 3) {
  //   // It's fully calibrated, do something or print a message
  // }

  delay(100);  // Adjust as needed for update rate
}
