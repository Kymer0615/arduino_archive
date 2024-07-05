int laserPin = 5; 

void setup() {
  pinMode(laserPin, OUTPUT);
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("<Arduino is ready>");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("p")) {
      int brightness = input.substring(1).toInt();
      brightness = map(brightness, 0, 100, 0, 255);
      analogWrite(laserPin, brightness);
      Serial.print("<");
      Serial.print(brightness);
      Serial.println(">");
    }
  }
}
