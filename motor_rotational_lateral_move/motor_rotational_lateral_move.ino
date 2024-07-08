#include <Arduino.h>
#include <SoftwareSerial.h>


///////////////////////
// X-axis VARIABLES ///
///////////////////////
const int z_dirPin = 9;
const int z_stepPin = 8;

////////////////////////////
// PRE-DEFINED VARIABLES ///
////////////////////////////
byte txBuffer[20];
byte rxBuffer[20]; 
uint8_t rxCnt=0; 
uint8_t getCheckSum(uint8_t *buffer,uint8_t len);
bool waitingForACK(uint8_t len, int timeout, byte slaveAddr);
void printToMonitor(uint8_t *value);
SoftwareSerial motor(2, 3); // RX, TX
char startMarker = '<';
char endMarker = '>';

////////////////
// FUNCTIONS ///
////////////////
void getRealTimeLocation(byte slaveAddr);
void getPulseNumber(byte slaveAddr);
void getError(byte slaveAddr);
void getEnStatus(byte slaveAddr);
void getLockedRotorProtection(byte slaveAddr);
void getShaftProtection(byte slaveAddr);
void calibrateEncoder(byte slaveAddr);
void motorMove(byte slaveAddr, byte dir, byte speed);
void motorStop(byte slaveAddr);
void gotoZero(byte slaveAddr);
void setZero(byte slaveAddr);
void motorMoveWithPulses(byte slaveAddr, byte dir, byte speed, int32_t pulseNumber);
void z_move(int steps, int direction);

void setup() {
  motor.begin(38400);
  Serial.begin(9600);
  pinMode(z_stepPin, OUTPUT);
  pinMode(z_dirPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("<Arduino is ready>");
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the input until newline
    
    int underscoreIndex = input.indexOf('_');  // Find the position of the underscore
    if (underscoreIndex == -1) return;  // If no underscore, invalid command

    String command = input.substring(0, underscoreIndex);  // Extract command
    String hexString = input.substring(underscoreIndex + 1);  // Extract hex string
    byte slaveAddr = strtol(hexString.c_str(), NULL, 16);  // Convert hex string to byte

    if (command == "getRealTimeLocation") {
      getRealTimeLocation(slaveAddr);
    } else if (command == "getPulseNumber") {
      getPulseNumber(slaveAddr);
    } else if (command == "getError") {
      getError(slaveAddr);
    } else if (command == "getEnStatus") {
      getEnStatus(slaveAddr);
    } else if (command == "getLockedRotorProtection") {
      getLockedRotorProtection(slaveAddr);
    } else if (command == "getShaftProtection") {
      getShaftProtection(slaveAddr);
    } else if (command == "calibrateEncoder") {
      calibrateEncoder(slaveAddr);
    } else if (command == "motorMove") {
      // Additional parameters needed for motorMove
      int secondUnderscoreIndex = input.indexOf('_', underscoreIndex + 1);
      String dirString = input.substring(underscoreIndex + 1, secondUnderscoreIndex);
      byte dir = dirString.toInt();
      byte speed = input.substring(secondUnderscoreIndex + 1).toInt();
      motorMove(slaveAddr, dir, speed);
    } else if (command == "motorStop") {
      motorStop(slaveAddr);
    } else if (command == "gotoZero") {
      gotoZero(slaveAddr);
    } else if (command == "setZero") {
      setZero(slaveAddr);
    } else if (command == "zMove") {
      int secondUnderscoreIndex = input.indexOf('_', underscoreIndex + 1);
      if (secondUnderscoreIndex == -1) {
        Serial.println("Invalid parameter format for z_move");
        return;
      }
      String stepString = input.substring(underscoreIndex + 1, secondUnderscoreIndex);  // Extract step
      String directionString = input.substring(secondUnderscoreIndex + 1);  // Extract direction
      int step = stepString.toInt();  // Convert step to integer
      int direction = directionString.toInt();  // Convert direction to integer
      Serial.println(step);
      Serial.println(direction);
      z_move(step, direction);
    }else if (command == "motorMoveWithPulses") {
    // Additional parameters for motorMoveWithPulses
    int secondUnderscoreIndex = input.indexOf('_', underscoreIndex + 1);
    int thirdUnderscoreIndex = input.indexOf('_', secondUnderscoreIndex + 1);
    int fourthUnderscoreIndex = input.indexOf('_', thirdUnderscoreIndex + 1);
    
    if (secondUnderscoreIndex == -1 || thirdUnderscoreIndex == -1 || fourthUnderscoreIndex == -1) {
      Serial.print(startMarker);
      
      Serial.print("Invalid parameter format for motorMoveWithPulses");
      Serial.print(";");
   
      Serial.print(endMarker);
      return;
    }

    // Extract slaveAddr from the first part
    String slaveAddrString = input.substring(underscoreIndex + 1, secondUnderscoreIndex);
    byte slaveAddr = strtol(slaveAddrString.c_str(), NULL, 16); // Convert hex string to byte

    // Extract dir from the second part
    String dirString = input.substring(secondUnderscoreIndex + 1, thirdUnderscoreIndex);
    byte dir = dirString.toInt();

    // Extract speed from the third part
    String speedString = input.substring(thirdUnderscoreIndex + 1, fourthUnderscoreIndex);
    byte speed = speedString.toInt();

    // Extract pulseNumber from the fourth part
    int32_t pulseNumber = input.substring(fourthUnderscoreIndex + 1).toInt();

    // Call the function with the parsed parameters
    motorMoveWithPulses(slaveAddr, dir, speed, pulseNumber);
  }else {
      Serial.print(startMarker);
      
      Serial.print("Invalid command");
      Serial.print(";");
   
      Serial.print(endMarker);
    }
  }
}


//////////////////////
// z_axis FUNCTIONS //
//////////////////////
void z_move(int steps, int direction) {
  // Set the direction:
  Serial.print(startMarker);
  if (direction == 1) {
    Serial.print("Backward");
    digitalWrite(z_dirPin, HIGH); 
  } else {
    Serial.print("Forward");
    digitalWrite(z_dirPin, LOW); 
  }
  Serial.print(";");                                              
  Serial.print("start");
  Serial.print(";");
 
  Serial.println(endMarker);
  // Make the stepper take the specified number of steps:
  for (int i = 0; i < steps; i++) {
    digitalWrite(z_stepPin, HIGH);
    delayMicroseconds(1000); // Adjust this delay to control the speed
    digitalWrite(z_stepPin, LOW);
    delayMicroseconds(1000);
  }
}

//////////////////////
// GETTER FUNCTIONS //
//////////////////////
void getRealTimeLocation(byte slaveAddr)
{ 
  bool ackStatus;
  int32_t carry;
  uint16_t value;

  int32_t location;
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x30;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(8, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {

    carry = (int32_t)(
                      ((uint32_t)rxBuffer[1] << 24)    |
                      ((uint32_t)rxBuffer[2] << 16)    |
                      ((uint32_t)rxBuffer[3] << 8)     |
                      ((uint32_t)rxBuffer[4] << 0)
                    );
    value = (uint16_t)(
                      ((uint16_t)rxBuffer[5] << 8)    |
                      ((uint16_t)rxBuffer[6] << 0)  
                    );

    location = carry * 65536L + (carry < 0 ? -value : value); // Adjust based on carry
    digitalWrite(LED_BUILTIN, LOW); //灭灯

    Serial.print(startMarker);
    
    Serial.print("location = ");
    Serial.print(location);
    Serial.print(";");
    
    Serial.print("carry = ");
    Serial.print(carry);
    Serial.print(";");
    
    Serial.print("value = ");
    Serial.print(value);
    Serial.print(";");
    
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

  
}


void getPulseNumber(byte slaveAddr)
{ 
  bool ackStatus;
  int32_t pulseNumber;

  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x33;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(6, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    pulseNumber = (int32_t)(
                      ((uint32_t)rxBuffer[1] << 24)    |
                      ((uint32_t)rxBuffer[2] << 16)    |
                      ((uint32_t)rxBuffer[3] << 8)     |
                      ((uint32_t)rxBuffer[4] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);

    Serial.print("pulseNumber = ");
    Serial.print(pulseNumber);
    Serial.print(";");
   
    Serial.print(endMarker);

  }
  else{
   errorLED();
  }

 
}

void getError(byte slaveAddr)
{ 
  bool ackStatus;
  int16_t error;
  float error_float;
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x39;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(4, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    error = (int16_t)(
                      ((int16_t)rxBuffer[1] << 8)     |
                      ((int16_t)rxBuffer[2] << 0)
                    );
    error_float = error / (65536L/360);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("error = ");
    Serial.print(error_float);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}

void getEnStatus(byte slaveAddr)
{ 
  bool ackStatus;
  int8_t EnStatus;
  bool EnStatusBool;
  
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x3A;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    EnStatus = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    EnStatusBool = (EnStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("EnStatus = ");
    Serial.print(EnStatusBool);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}

void getLockedRotorProtection(byte slaveAddr)
{ 
  bool ackStatus;
  uint8_t protectStatus;
  bool protectStatusBool;
  
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x3D;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    protectStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
//    protectStatusBool = (protectStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("Locked-rotor protection status = ");
    Serial.print(protectStatus);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}

void getShaftProtection(byte slaveAddr)
{ 
  bool ackStatus;
  uint8_t protectStatus;
  bool protectStatusBool;
  
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  //从机地址
  txBuffer[1] = 0x3E;       //功能码
  txBuffer[2] = getCheckSum(txBuffer,2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    protectStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
    protectStatusBool = (protectStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("Motor shaft protection status = ");
    Serial.print(protectStatusBool);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}


//////////////////////
// SETTER FUNCTIONS //
//////////////////////
void calibrateEncoder(byte slaveAddr)
{ 
  bool ackStatus;
  uint8_t calibrationStatus;
  bool calibrationStatusBool;
  
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0x80;       //function 
  txBuffer[2] = 0x00;       //data
  txBuffer[3] = getCheckSum(txBuffer, 3);  //计算校验和
  motor.write(txBuffer,4);   

  ackStatus = waitingForACK(3, 30000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    calibrationStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
    calibrationStatusBool = (calibrationStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    Serial.print("Calibration status = ");
    Serial.print(calibrationStatusBool);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}


void motorMove(byte slaveAddr, byte dir, byte speed)
{ 
  bool ackStatus;
  int8_t status;

  byte data = 0b00000000;
  
  if (speed >= 0 && speed <= 127 && dir < 2) {
    data |= speed;
    data |= (dir << 7); 
  } else {
    Serial.print(startMarker);
    Serial.print("Input speed value out of range (0 to 127).");
    Serial.print(";");
    Serial.print(endMarker);
  }
      
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0xF6;       //function 
  txBuffer[2] = data;       //data
  txBuffer[3] = getCheckSum(txBuffer, 3);  //计算校验和
  motor.write(txBuffer,4);   

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    Serial.print("Move motor = ");
    Serial.print(status);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}

void motorStop(byte slaveAddr)
{ 
  bool ackStatus;
  int8_t status;

  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0xF7;       //function 
  txBuffer[2] = getCheckSum(txBuffer, 2);  //计算校验和
  motor.write(txBuffer,3);   

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("Move stop = ");
    Serial.print(status);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

 
}

void gotoZero(byte slaveAddr)
{ 
  bool ackStatus;
  int8_t status;

  byte data = 0x00;
      
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0x94;       //function 
  txBuffer[2] = data;       //data
  txBuffer[3] = getCheckSum(txBuffer, 3);  //计算校验和
  motor.write(txBuffer,4);   

  ackStatus = waitingForACK(3, 10000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("gotoZero status = ");
    Serial.print(status);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

}

void setZero(byte slaveAddr)
{ 
  bool ackStatus;
  int8_t status;

  byte data = 0x00;
      
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0x91;       //function 
  txBuffer[2] = data;       //data
  txBuffer[3] = getCheckSum(txBuffer, 3);  //计算校验和
  motor.write(txBuffer,4);   

  ackStatus = waitingForACK(3, 10000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print(startMarker);
    
    Serial.print("Set zero status = ");
    Serial.print(status);
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  else{
   errorLED();
  }

}

void motorMoveWithPulses(byte slaveAddr, byte dir, byte speed, int32_t pulseNumber)
{ 
  bool ackStatus;
  int8_t status;
  String status_str;
  byte specs = 0;
  
  if (speed >= 0 && speed <= 127 && dir < 2 && 0 <= dir ) {
    specs |= speed;
    specs |= (dir << 7); 
  } else {
    Serial.print(startMarker);
    
    Serial.print("Input speed or dir value out of range (0 to 127).");
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  if (pulseNumber>= 0 && pulseNumber < 4294967296L){
    txBuffer[0] = slaveAddr;  
    txBuffer[1] = 0xFD;       //function 
    txBuffer[2] = specs;
    txBuffer[3] = (pulseNumber >> 24) & 0xFF; 
    txBuffer[4] = (pulseNumber >> 16) & 0xFF; 
    txBuffer[5] = (pulseNumber >> 8) & 0xFF; 
    txBuffer[6] = (pulseNumber >> 0) & 0xFF; 
    txBuffer[7] = getCheckSum(txBuffer, 7);  //计算校验和
  } else {
    Serial.print(startMarker);
    
    Serial.print("pulseNumber out of range (0 to 4294967296L).");
    Serial.print(";");
   
    Serial.print(endMarker);
  }
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  
  motor.write(txBuffer,8);   

  Serial.print(startMarker);
  Serial.print("start;");
  Serial.print(endMarker);


//  ackStatus = waitingForACK(3, 6000, slaveAddr);      //等待电机应答
//
//  if(ackStatus == true)        //接收到位置信息
//  {
//    status = (int8_t)(
//                      ((int8_t)rxBuffer[1] << 0)
//                    );
//    if (status == 0){
//      status_str = "fail";
//    }else if (status == 1){
//      status_str = "starting";
//    }else if (status == 2){
//      status_str = "complete";
//    }
//    digitalWrite(LED_BUILTIN, LOW); //灭灯
//    Serial.print(startMarker);
//    
//    Serial.print("Move motor with pulse status = ");
//    Serial.print(status_str);
//    Serial.print(";");
//   
//    Serial.print(endMarker);
//  }
//  else{
//   errorLED();
//  }

 
}
//////////////////////
// HELPER FUNCTIONS //
//////////////////////
bool waitingForACK(uint8_t len, int timeout, byte slaveAddr)
{
  bool retVal;       //返回值
  unsigned long sTime;  //计时起始时刻
  unsigned long time;  //当前时刻
  uint8_t rxByte;      

  sTime = millis();    //获取当前时刻
  rxCnt = 0;           //接收计数值置0
  while(1)
  {
    if (motor.available() > 0)     //串口接收到数据
    {
      rxByte = motor.read();       //读取1字节数据
      if(rxCnt != 0)
      {
        rxBuffer[rxCnt++] = rxByte; //存储数据
      }
      else if(rxByte == slaveAddr)       //判断是否帧头
      {
        rxBuffer[rxCnt++] = rxByte;   //存储帧头
      }
    }

    if(rxCnt == len)    //接收完成
    {
      if(rxBuffer[len-1] == getCheckSum(rxBuffer,len-1))
      {
        retVal = true;   //校验正确
        break;                  //退出while(1)
      }
      else
      {
        rxCnt = 0;  //校验错误，重新接收应答
      }
    }

    time = millis();
    if((time - sTime) > timeout)   //判断是否超时
    {
      retVal = false;
      break;                    //超时，退出while(1)
    }
  }
  return(retVal);
}


byte getCheckSum(byte *buffer,byte size)
{
  uint8_t i;
  uint16_t sum=0;
  for(i=0;i<size;i++)
    {
      sum += buffer[i];  //计算累加值
    }
  return(sum&0xFF);     //返回校验和
}

void errorLED()
{
  Serial.print(startMarker);
  Serial.print("ERROR");
  Serial.println(endMarker);
  for (int i=0; i<=5;  i++){               //快速闪灯，提示运行失败
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}
