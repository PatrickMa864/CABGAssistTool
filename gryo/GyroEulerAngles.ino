/******************************************************************************
  This sketch demonstrates reading Euler angles (heading, roll, pitch) from
  the Adafruit BNO055 absolute orientation sensor. The BNO055 handles the
  sensor fusion internally, so you get stable, drift-corrected data without
  needing to manually process raw gyroscope/accelerometer data.

  Hardware:
    - BNO055 sensor
    - Arduino Mega 2560 R3
    - I2C connections (SDA -> pin 20, SCL -> pin 21)

  Libraries:
    - Adafruit_BNO055
    - Adafruit Unified Sensor

 ******************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>        // Unified sensor library
#include <Adafruit_BNO055.h>        // BNO055 library
#include <utility/imumaths.h>       // Math helper library (optional)

// Create the BNO object
// If your ADR pin is tied LOW, the I2C address is 0x28.
// If your ADR pin is tied HIGH, the address is 0x29.
// If you’re unsure, try 0x28 first, or check your breakout board’s documentation.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // Start serial communication
  Serial.begin(115200);
  while (!Serial) {
    // On some boards (e.g., Leonardo, Micro), you need this to wait for the Serial port to come online
  }

  Serial.println("BNO055 Orientation Sensor Test");
  Serial.println("--------------------------------");

  // Initialize the BNO055
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C address!");
    while (1);
  }

  // Optional: Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  // Wait a bit for device to be fully started
  delay(1000);

  Serial.println("BNO055 initialized successfully!");
  Serial.println("Calibration may take a bit. Move sensor gently in all directions.");
  Serial.println("Once calibrated, heading/roll/pitch will be stable and accurate.\n");
}

void loop() {
  // The Adafruit_BNO055 library has a built-in function to get orientation data.
  // We'll request Euler angles: heading (x), roll (z), and pitch (y).

  // Create a sensor event object
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // orientationData.orientation.x = heading in degrees
  // orientationData.orientation.y = roll in degrees
  // orientationData.orientation.z = pitch in degrees
  float heading = orientationData.orientation.x;
  float roll    = orientationData.orientation.z;
  float pitch   = orientationData.orientation.y;

  // Print the results
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("°, \t");

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("°, \t");

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.println("°");

  // (Optional) Display calibration status
  // (Sys, Gyro, Accel, Mag). Each ranges from 0 (uncalibrated) to 3 (fully calibrated).
  uint8_t systemStatus, gyroStatus, accelStatus, magStatus;
  bno.getCalibration(&systemStatus, &gyroStatus, &accelStatus, &magStatus);

  Serial.print("Calibration -> Sys: ");
  Serial.print(systemStatus, DEC);
  Serial.print(" Gyro: ");
  Serial.print(gyroStatus, DEC);
  Serial.print(" Accel: ");
  Serial.print(accelStatus, DEC);
  Serial.print(" Mag: ");
  Serial.println(magStatus, DEC);

  Serial.println("--------------------------------\n");

  // Delay so we can read the output comfortably
  delay(500);
}

