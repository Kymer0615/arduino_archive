/**************** MKS SERVOxxD闭环步进电机 ***************
******************** arduino系列实验1 ********************
**  实验名称：脉冲控制电机运行
**  实验目的：arduino UNO 主板发送脉冲信号，控制电机循环正反转动
**  实验现象：程序运行后，可观察到:
** 1. LED灯亮,电机正转一圈；
** 2. LED灯灭,电机反转一圈；
** 3. 如此不断循环
** 注意事项：电机工作模式设置为CR_vFOC 或者 CR_CLOSE    
** 其他资源：
**  CSDN博客：https://blog.csdn.net/gjy_skyblue
**  B站视频：https://space.bilibili.com/393688975/channel/series
**  百度网盘：https://pan.baidu.com/s/1BjrK9SC8pWnDoU32F8jHqA?pwd=mks2
**  QQ技术群：948665794             
**********************************************************/

int EN_PIN = 4;     //定义使能信号端口
int STP_PIN = 3;    //定义脉冲信号端口
int DIR_PIN = 2;    //定义方向信号端口

int i;
int PULSE_DECAY = 1200;
int PULSE_NUM = 301;
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(38400);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
    
  pinMode(EN_PIN, OUTPUT);    //设置使能信号端口为输出模式
  pinMode(STP_PIN, OUTPUT);   //设置脉冲信号端口为输出模式
  pinMode(DIR_PIN, OUTPUT);   //设置方向信号端口为输出模式

  digitalWrite(EN_PIN, HIGH);   //设置使能信号高电平
  digitalWrite(STP_PIN, HIGH);  //设置脉冲信号高电平
  digitalWrite(DIR_PIN, HIGH);  //设置方向信号高电平
  
  delay(1000);                  //延时1000毫秒
}



// the loop function runs over and over again forever
void loop() {
/*输出脉冲信号，控制电机正(反)转1圈(16细分)
 *脉冲频率越高，电机转速越快
 *脉冲频率通过延时函数delayMicroseconds()改变*/
    digitalWrite(EN_PIN, LOW);          //使能信号输出低电平，电机锁轴
    digitalWrite(DIR_PIN, LOW);         //方向信号输出低电平，电机正（反）向转动
    digitalWrite(LED_BUILTIN, HIGH);    //点亮LED灯 
    for(i=0;i<PULSE_NUM;i++)                 //输出3200个脉冲信号
    {
      digitalWrite(STP_PIN, HIGH);      //脉冲信号输出高电平 
      delayMicroseconds(PULSE_DECAY);           //延时100微秒
      digitalWrite(STP_PIN, LOW);       //脉冲信号输出低电平 
      delayMicroseconds(PULSE_DECAY);           //延时100微秒
    }
    digitalWrite(EN_PIN, HIGH);         //使能信号输出高电平，电机松轴
    delay(4000);                        //延时1000毫秒

/*输出脉冲信号，控制电机正(反)转1圈(16细分)
 *脉冲频率越高，电机转速越快
 *脉冲频率通过延时函数delayMicroseconds()改变*/
    digitalWrite(EN_PIN, LOW);          //使能信号输出低电平，电机锁轴
    digitalWrite(DIR_PIN, HIGH);        //方向信号输出高电平，电机反（正）向转动
    digitalWrite(LED_BUILTIN, LOW);     //熄灭LED灯 
    for(i=0;i<PULSE_NUM;i++)                 //输出3200个脉冲信号
    {
      digitalWrite(STP_PIN, HIGH);      //脉冲信号输出高电平 
      delayMicroseconds(PULSE_DECAY);           //延时100微秒
      digitalWrite(STP_PIN, LOW);       //脉冲信号输出低电平 
      delayMicroseconds(PULSE_DECAY);           //延时100微秒
    }
    digitalWrite(EN_PIN, HIGH);         //使能信号输出高电平，电机松轴
    delay(4000);                        //延时1000毫秒
}
