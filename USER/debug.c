#include "headfile.h"
#include "isr.h"
#include "stdint.h"
#include "img_process.h"

/*********************************************************
  @seusmartcar
  名优科创上位机破解协议
  @author：SEU_SmartCar
  @2019.10.11
  @for 直立组
*********************************************************/
extern int16_t left_encoder;
extern int16_t right_encoder;
extern int _kind;
float Variable[16]={0};
float Parameter[14]={0};
char SendPara=0,send_data=0;
extern int _kind_sum;
extern int steer_pwm;
extern float slope;
extern int16_t left_speed;
extern int16_t right_speed;
extern float target_pwm_left;
extern float target_pwm_right;
extern float sp_cha_k;
extern RoadType road_type;

/*
 * 取一个数据的各个位
 */
#define BYTE0(Temp)       (*(char *)(&Temp))
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))

void my_putchar(char temp)
{
    uart_putchar(UART_3, temp);
}

/*用来通知上位机新的一组数据开始，要保存数据必须发送它*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}


//设置需要发送的参数  可以设16个
void Testdata_generate()
{
  static int data;
  static long  i;
  data=data+1;
  if(data>1000) data=0;

  Variable[0]=left_encoder;
  Variable[1]=right_encoder;
  Variable[2]=_kind;
  Variable[3] = _kind_sum;
  Variable[4] = steer_pwm;
  Variable[5] = slope;
  Variable[6] = left_speed;
  Variable[7] = right_speed;
  Variable[8] = target_pwm_left;
  Variable[9] = target_pwm_right;
  Variable[10] = road_type;
  Variable[11] = 0;
  Variable[12] = 0;
  Variable[13] = 0;
  Variable[14] = 0;
  Variable[15] = 0;
  //Variable[3]=speed_filter.data_average;
  //Variable[4]=duty;
  //Variable[5]=1-dynamic_decrease_Velocity;
  //Variable[6]=adc_information.voltage_bus;
  //Variable[7]= mec_zero - dynamic_mec_zero- BLDC_expect_angle * (1-dynamic_decrease_Velocity);
  //Variable[8]=mec_zero-dynamic_mec_zero;
  //Variable[9]=BLDC_real_duty;
  //Variable[10]=BLDC_out_current;
  //Variable[11]=speed;
  //Variable[12]=(1-dynamic_decrease_Velocity)*BLDC_expect_angle;
  //Variable[13]=BLDC_balance_PID.Integration;
  //Variable[14]=feedforward;
  //Variable[15]=BLDC_DIR;
}



//发送实时变量
void Send_Variable()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Variable_num=16;
  my_putchar(85);//协议头_1
  my_putchar(170);//协议头_2
  my_putchar(255);//协议头_3
  my_putchar(1);//协议头_4
  my_putchar(Variable_num);//发送出的变量个数
  for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
      my_putchar(ch);
    ch=BYTE1(temp);
      my_putchar(ch);
    ch=BYTE2(temp);
      my_putchar(ch);
    ch=BYTE3(temp);
      my_putchar(ch);
  }
 my_putchar(1);//协议尾
}





//获取到上位机参数后，修改代码内的参数
void Modify_Parameter(uint8 *buff)
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num=14; //14个可改参数
  /*修改参数数组*/
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE1(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE2(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE3(temp)=*(uint8*)(buff+addr);
       addr++;
       Parameter[i]=temp;
   }
   
 /*从参数数组中更新参数值  示例*/
   //PID_P=Parameter[0];
   //PID_I=Parameter[1];
   //PID_D=Parameter[2];
}
//发送14个调参量  参数回传
void Send_Parameter()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=14;  //14个可改参数
  
  
  /*将参数值更新到参数数组中  示例*/
   //Parameter[0]=PID_P;
   //Parameter[1]=PID_I;
   //Parameter[2]=PID_D;
  /*                           */
  my_putchar(85);//协议头_1
  my_putchar(170);//协议头_2
  my_putchar(255);//协议头_3
  my_putchar(2);//协议头_4
  my_putchar(Parameter_num);
  for(i=0;i<Parameter_num;i++)
  { 
    //PEout(25)=1;
    temp=Parameter[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
  my_putchar(2);////协议尾
}

//蓝牙中断接收上位机14个调参数据
void UART3_RX_IRQHandler(uint16 bytereceive)//蓝牙串口接收数据的中断函数，bytereceive为接收到的数据
{
    uint16 recv=bytereceive;
  static uint8 data_cnt=0;
  static uint8 predata[10];
  static uint8 Recv_Buff[100];
  static uint8 Data_Receiving=FALSE;
  if(Data_Receiving)//代表正在接收来自上位机的参数数据
  {
    if(data_cnt<56)//4个字节*14个数据
    {
      Recv_Buff[data_cnt]= recv;//一个数据4字节
      data_cnt++;
    }
    else
    {
      data_cnt=0;    //达到帧长
      Data_Receiving=FALSE;
      if(recv==2)  //帧尾
      {
         Modify_Parameter(Recv_Buff);//获取到上位机参数后，修改代码内的参数
         SendPara=1;      //参数回传给上位机，确认参数修改完成
      }
    }
  }
  if( predata[1]==0x55&&predata[0]==0xAA)//协议
  {
    switch(recv)         //根据协议判断功能字
    { 
    case 1:           //读取参数 
      if(SendPara==0) SendPara=1;
      break;
    case 2:           //修改参数
      Data_Receiving=TRUE;
      data_cnt=0;
      break;
    case 3://保存参数
      break;        
    case 4://功能开关1
      break;    
    case 5://功能开关2
      break;     
    case 6://功能开关3
      break; 
    case 7://功能开关4
      break;        
    default:           //
      break;
    }
  }
  predata[3]=predata[2];
  predata[2]=predata[1];
  predata[1]=predata[0];
  predata[0]=recv;
}

void Send_Inductor()//电磁上位机
{
  //阉割
}



void send_picture()//摄像头上位机，只支持80*60的分辨率    不要使用   等我之后有空魔改一下这个软件
{
  
}
