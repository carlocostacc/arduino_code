const int sensorPin = A6; // Connect the GP2Y0A21 IR rangefinder to analog pin A0
const int ledPin = 11; // Connect the LED to digital pin 3

void setup() {
  Serial.begin(9600); // Initialize the serial communication at 9600 baud
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read the value from the IR rangefinder

  float voltage = (sensorValue / 1024.0) * 3.35; // Convert the ADC value to voltage
  float distance = 17 * pow(voltage, -1.101); // Apply the formula to convert voltage to distance

  Serial.println(distance); // Print the distance value to the serial monitor

  // Check the distance and adjust the LED brightness accordingly
  if (distance >= 45) {
    analogWrite(ledPin, 255); // 100% brightness
  } else if (distance <= 15) {
    analogWrite(ledPin, 0); // 0% brightness
  } else {
    int brightness = map(distance, 45, 15, 0, 255); // Linear increase from 45 cm to 15 cm
    analogWrite(ledPin, brightness);
  }

  delay(100); // Wait for 100 milliseconds before reading the sensor again
}

