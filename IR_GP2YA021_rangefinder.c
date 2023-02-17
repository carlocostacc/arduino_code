const int sensorPin = A6; // Connect the GP2Y0A21 IR rangefinder to analog pin A0

void setup() {
  Serial.begin(9600); // Start serial communication for debugging purposes
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read the value from the IR rangefinder

  float voltage = (sensorValue / 1024.0) * 3.35; // Convert the ADC value to voltage
  float distance = 17 * pow(voltage, -1.101); // Apply the formula to convert voltage to distance

  Serial.print("ADC Value: ");
  Serial.print(sensorValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage);
  Serial.print(", Distance: ");
  Serial.println(distance);

  delay(100); // Wait for 100 milliseconds before reading the sensor again
}