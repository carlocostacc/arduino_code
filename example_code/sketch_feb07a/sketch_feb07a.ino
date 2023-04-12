#define TRIGGER_PIN  11   // Trigger pin on the HC-SR04
#define ECHO_PIN     2   // Echo pin on the HC-SR04

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);  // Set trigger pin as output
  pinMode(ECHO_PIN, INPUT);      // Set echo pin as input
  Serial.begin(9600);            // Start serial communication at 9600 baud
}

void loop() {
  long duration, distance;

  // Trigger the measurement by sending a 10 us pulse
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the duration of the echo pulse
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance based on the speed of sound
  distance = (duration / 2) * 0.0343;

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait for a short time before the next measurement
  delay(50);
}
