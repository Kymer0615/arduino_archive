#include <Arduino.h>

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(38400);
}

void loop() {
  // Example data to send
  byte slaveAddr = 0xE0; // Slave address, can range from 0xE0 to 0xE9
  byte functionCode = 0x30; // Function code, for example, 0x80 for calibration
  byte data = 0x30; // Data byte
  
  // Calculate the Checksum (CRC)
  byte checksum = calculateChecksum(slaveAddr, functionCode, data);

  // Send data over serial
  sendSerialData(slaveAddr, functionCode, data, checksum);

  // Delay to avoid spamming; adjust the delay as needed
  delay(2000);
}

byte calculateChecksum(byte addr, byte func, byte data) {
  // As per the description: tCHK = (0xE0 + 0x30) & 0xFF = 0x10 & 0xFF = 0x10
  return (addr + func + data) & 0xFF;
}

void sendSerialData(byte addr, byte func, byte data, byte crc) {
  Serial.print("Sending Packet: ");
  Serial.print(addr, HEX);
  Serial.print(" ");
  Serial.print(func, HEX);
  Serial.print(" ");
  Serial.print(data, HEX);
  Serial.print(" ");
  Serial.print(crc, HEX);
  Serial.println();

  // Actually send the bytes
  Serial.write(addr);
  Serial.write(func);
//  Serial.write(data);
  Serial.write(crc);
}
