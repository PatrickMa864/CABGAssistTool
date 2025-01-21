/*
   Arduino code to turn LED on pin 13 ON/OFF based on serial input:
   - '1' => LED ON
   - '0' => LED OFF
*/

const int LED_PIN = 13;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);  // Must match Python's baud rate
}

void loop() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == '1') {
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
    } else if (incomingByte == '0') {
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
    }
  }
}
