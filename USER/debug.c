#include "headfile.h"
#include "isr.h"
#include "stdint.h"
#include "img_process.h"

/*********************************************************
  @seusmartcar
  ���ſƴ���λ���ƽ�Э��
  @author��SEU_SmartCar
  @2019.10.11
  @for ֱ����
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
 * ȡһ�����ݵĸ���λ
 */
#define BYTE0(Temp)       (*(char *)(&Temp))
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))

void my_putchar(char temp)
{
    uart_putchar(UART_3, temp);
}

/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}


//������Ҫ���͵Ĳ���  ������16��
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



//����ʵʱ����
void Send_Variable()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Variable_num=16;
  my_putchar(85);//Э��ͷ_1
  my_putchar(170);//Э��ͷ_2
  my_putchar(255);//Э��ͷ_3
  my_putchar(1);//Э��ͷ_4
  my_putchar(Variable_num);//���ͳ��ı�������
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
 my_putchar(1);//Э��β
}





//��ȡ����λ���������޸Ĵ����ڵĲ���
void Modify_Parameter(uint8 *buff)
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num=14; //14���ɸĲ���
  /*�޸Ĳ�������*/
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
   
 /*�Ӳ��������и��²���ֵ  ʾ��*/
   //PID_P=Parameter[0];
   //PID_I=Parameter[1];
   //PID_D=Parameter[2];
}
//����14��������  �����ش�
void Send_Parameter()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=14;  //14���ɸĲ���
  
  
  /*������ֵ���µ�����������  ʾ��*/
   //Parameter[0]=PID_P;
   //Parameter[1]=PID_I;
   //Parameter[2]=PID_D;
  /*                           */
  my_putchar(85);//Э��ͷ_1
  my_putchar(170);//Э��ͷ_2
  my_putchar(255);//Э��ͷ_3
  my_putchar(2);//Э��ͷ_4
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
  my_putchar(2);////Э��β
}

//�����жϽ�����λ��14����������
void UART3_RX_IRQHandler(uint16 bytereceive)//�������ڽ������ݵ��жϺ�����bytereceiveΪ���յ�������
{
    uint16 recv=bytereceive;
  static uint8 data_cnt=0;
  static uint8 predata[10];
  static uint8 Recv_Buff[100];
  static uint8 Data_Receiving=FALSE;
  if(Data_Receiving)//�������ڽ���������λ���Ĳ�������
  {
    if(data_cnt<56)//4���ֽ�*14������
    {
      Recv_Buff[data_cnt]= recv;//һ������4�ֽ�
      data_cnt++;
    }
    else
    {
      data_cnt=0;    //�ﵽ֡��
      Data_Receiving=FALSE;
      if(recv==2)  //֡β
      {
         Modify_Parameter(Recv_Buff);//��ȡ����λ���������޸Ĵ����ڵĲ���
         SendPara=1;      //�����ش�����λ����ȷ�ϲ����޸����
      }
    }
  }
  if( predata[1]==0x55&&predata[0]==0xAA)//Э��
  {
    switch(recv)         //����Э���жϹ�����
    { 
    case 1:           //��ȡ���� 
      if(SendPara==0) SendPara=1;
      break;
    case 2:           //�޸Ĳ���
      Data_Receiving=TRUE;
      data_cnt=0;
      break;
    case 3://�������
      break;        
    case 4://���ܿ���1
      break;    
    case 5://���ܿ���2
      break;     
    case 6://���ܿ���3
      break; 
    case 7://���ܿ���4
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

void Send_Inductor()//�����λ��
{
  //�˸�
}



void send_picture()//����ͷ��λ����ֻ֧��80*60�ķֱ���    ��Ҫʹ��   ����֮���п�ħ��һ��������
{
  
}
