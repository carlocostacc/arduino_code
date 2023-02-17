const int sensorPin = A0; // input pin for the rangefinder
const int ledPin = 13; // PB5, connected to the red LED

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int distance = analogRead(sensorPin);

  // Convert the analog reading to distance in cm
  distance = 13363 * pow(distance, -1.15);

  if (distance < 15 || distance > 45) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  Serial.println(distance);
  delay(100);
}
