#include <Arduino.h>
#include <SoftwareSerial.h>


byte txBuffer[3];
SoftwareSerial motor_0(2, 3); // RX, TX

void setup() {
  motor_0.begin(38400);
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // Example data to send
  byte slaveAddr = 0xE0; // Slave address, can range from 0xE0 to 0xE9
  byte functionCode = 0x3E; // Function code, for example, 0x80 for calibration
  byte data = 0x00; // Data byte
  
  // Calculate the Checksum (CRC)
  byte checksum = calculateChecksum(slaveAddr, functionCode, data);

  // Send data over serial
  sendSerialData(slaveAddr, functionCode, data, checksum);
  // Delay to avoid spamming; adjust the delay as needed
  delay(2000);

  if (motor_0.available() > 0) {  // Check if data is available to read
//    byte recvByte = ;  // Read the incoming byte

    // Print the hexadecimal value of the received byte
    Serial.print("Received byte in hex: ");
    Serial.print(motor_0.read());  // Print as hexadecimal
    Serial.println();
  }
}

byte calculateChecksum(byte addr, byte func, byte data) {
  // As per the description: tCHK = (0xE0 + 0x30) & 0xFF = 0x10 & 0xFF = 0x10
  return (addr + func + data) & 0xFF;
}

void sendSerialData(byte addr, byte func, byte data, byte crc) {
//  Serial.print("Sending Packet: ");
  Serial.print(addr, HEX);
  Serial.print(" ");
  Serial.print(func, HEX);
  Serial.print(" ");
//  Serial.print(data, HEX);
//  Serial.print(" ");
  Serial.print(crc, HEX);
  Serial.println();

  // Actually send the bytes
//  Serial.write(addr);
//  Serial.write(func);
////  Serial.write(data);
//  Serial.write(crc);

  txBuffer[0] = addr;       //帧头
  txBuffer[1] = func;  //从机地址
  txBuffer[2] = crc;       //功能码
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令
  delay(2000);
//  txBuffer[0] = 0xFA;       //帧头
//  txBuffer[1] = addr;  //从机地址
//  txBuffer[2] = 0x31;       //功能码
//  txBuffer[3] = crc;  //计算校验和
//  Serial.write(txBuffer,4);   //串口发出读取实时位置指令

  
}
