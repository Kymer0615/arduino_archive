int laserPin = 5; 

void setup() {
  pinMode(laserPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("p")) {
      int brightness = input.substring(1).toInt();
      brightness = map(brightness, 0, 100, 0, 255);
      Serial.println(brightness);
      analogWrite(laserPin, brightness);
      Serial.println(input); // Acknowledge the command
      Serial.flush();
    }
  }
}
