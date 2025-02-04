// Define the analog input pin
const int potPin = A1;

// Variables to store voltage and wiper position
float voltage;
float wiperPosition;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set the analog reference to the default 5V
  analogReference(DEFAULT);
}

void loop() {
  // Read the analog value
  int sensorValue = analogRead(potPin);
  
  // Convert the analog reading (0 - 1023) to voltage (0 - 5V)
  voltage = sensorValue * (5.0 / 1023.0);
  
  // Calculate wiper position as a percentage
  wiperPosition = 100 * (voltage / 5.0);
  
  // Print the results
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);  // Display voltage with 2 decimal places
  Serial.print(" V, Wiper Position: ");
  Serial.print(wiperPosition, 1);  // Display position with 1 decimal place
  Serial.println("");
  
  // Short delay before next reading
  delay(100);
}
