#include <Arduino.h>
#include <SoftwareSerial.h>


byte txBuffer[3];
byte rxBuffer[20]; 
uint8_t rxCnt=0; 

uint8_t getCheckSum(uint8_t *buffer,uint8_t len);
bool waitingForACK(uint8_t len, int timeout, byte slaveAddr);
void printToMonitor(uint8_t *value);

SoftwareSerial motor_0(2, 3); // RX, TX

void setup() {
  motor_0.begin(38400);
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
//  getRealTimeLocation(0xE0);
//  getPulseNumber(0xE0);
//  getError(0xE0);
//  getEnStatus(0xE0);
//  getLockedRotorProtection(0xE0);
//  getShaftProtection(0xE0);
//
//  calibrateEncoder(0xE0);
//  Serial.print("fished");
//
//  motorMove(0XE0, 1, 10);
//  delay(2000);
//  motorStop(0XE0);
//  gotoZero(0XE0);
//  motorMoveWithPulses(0XE0, 0, 5, 1000); //slaveAddr dir speed pulses
//  getRealTimeLocation(0xE0);
//  delay(20000000);
//  setLockedRotorProtection(0xE0, 0x00);
//  setZero(0XE0);


//  getRealTimeLocation(0xE0);
//  delay(2000);
//  motorMoveWithPulses(0XE0, 0, 5, 1000);
//  delay(2000);
//  getRealTimeLocation(0xE0);
//  delay(2000);
//  motorStop(0XE0);
//  delay(200000);
//  getRealTimeLocation(0xE0);
//  delay(20000);

//  setZero(0XE0);
//  gotoZero(0XE0);
//    delay(20000000);
//  
//  delay(2000);
//  getRealTimeLocation(0xE0);
//  delay(2000);
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

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
    Serial.print("location = ");
    Serial.println(location);

    Serial.print("carry = ");
    Serial.println(carry);

    Serial.print("value = ");
    Serial.println(value);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

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
    Serial.print("pulseNumber = ");
    Serial.println(pulseNumber);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(4, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    error = (int16_t)(
                      ((int16_t)rxBuffer[1] << 8)     |
                      ((int16_t)rxBuffer[2] << 0)
                    );
    error_float = error / (65536L/360);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("error = ");
    Serial.println(error_float);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    EnStatus = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    EnStatusBool = (EnStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("EnStatus = ");
    Serial.println(EnStatusBool);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    protectStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
//    protectStatusBool = (protectStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Locked-rotor protection status = ");
    Serial.println(protectStatus);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    protectStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
    protectStatusBool = (protectStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Motor shaft protection status = ");
    Serial.println(protectStatusBool);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,4);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 300000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    calibrationStatus = (uint8_t)(
                      ((uint8_t)rxBuffer[1] << 0)
                    );
    calibrationStatusBool = (calibrationStatus == 1 ? true : false);
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Calibration status = ");
    Serial.println(calibrationStatusBool);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
    Serial.println("Input speed value out of range (0 to 127).");
  }
      
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  txBuffer[0] = slaveAddr;  
  txBuffer[1] = 0xF6;       //function 
  txBuffer[2] = data;       //data
  txBuffer[3] = getCheckSum(txBuffer, 3);  //计算校验和
  motor_0.write(txBuffer,4);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Move motor = ");
    Serial.println(status);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,3);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 3000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Move stop = ");
    Serial.println(status);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,4);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 10000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("gotoZero status = ");
    Serial.println(status);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
  motor_0.write(txBuffer,4);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 10000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Set zero status = ");
    Serial.println(status);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
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
    Serial.println("Input speed or dir value out of range (0 to 127).");
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
    Serial.println("pulseNumber out of range (0 to 4294967296L).");
  }
  digitalWrite(LED_BUILTIN, HIGH); //亮灯

  
  motor_0.write(txBuffer,8);   //串口发出读取实时位置指令

  ackStatus = waitingForACK(3, 6000, slaveAddr);      //等待电机应答

  if(ackStatus == true)        //接收到位置信息
  {
    status = (int8_t)(
                      ((int8_t)rxBuffer[1] << 0)
                    );
    if (status == 0){
      status_str = "fail";
    }else if (status == 1){
      status_str = "starting";
    }else if (status == 2){
      status_str = "complete";
    }
    digitalWrite(LED_BUILTIN, LOW); //灭灯
    Serial.print("Move motor with pulse status = ");
    Serial.println(status_str);
  }
  else                      //接收位置信息失败（1.检查串口线连接；2.检查电机是否上电；3.检查从机地址，波特率）
  {
    while(1)                //快速闪灯，提示运行失败
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
 
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
    if (motor_0.available() > 0)     //串口接收到数据
    {
      rxByte = motor_0.read();       //读取1字节数据
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
