#include "headfile.h"
extern char SendPara;
void supermonitor()//���ſƴ���λ��
{
  if(SendPara)
  {
    SendPara=0;
    Send_Parameter();//�����ش�����λ��
  }
  Testdata_generate();//������Ҫ���͵Ĳ���
  Send_Variable(); //���Ͳ���
  Send_Inductor();//�����λ��������ʹ��
  //send_picture();//��У���������ò��ˣ���Ϊ���ſƴ�������ͷ��λ����bugֻ֧��80*60�ķֱ���   ʹ��ɽ����λ����
  Send_Begin();
}






//ɽ��๦�ܵ���������λ��������ʾ������ʾ����
//wareaddr    ����������ʼ��ַ
//waresize    ��������ռ�ÿռ�Ĵ�С
//ʹ������     �� vcan_sendware((uint8_t *)data, sizeof(data));    data�Ǹ�����
void vcan_sendware(void *wareaddr, uint32 waresize)
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����
  
  uart_putbuff(UART_0, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
  uart_putbuff(UART_0, (char *)wareaddr, sizeof(wareaddr));   //��������
  uart_putbuff(UART_0, cmdr, sizeof(cmdr));    //���ͺ�����
}





// ɽ��๦�ܵ���������λ��������ͷ��ʾ����
//imgaddr    ͼ����ʼ��ַ
//imgsize    ͼ��ռ�ÿռ�Ĵ�С
//ʹ��������   vcan_sendimg(imgadd,row_num*col_num);   imgaddΪ��Ҫ�����ͼ������
void vcan_sendimg(void *imgaddr, uint32 imgsize)
{
#define CMD_IMG     1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //ɽ����λ�� ʹ�õ�����
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //ɽ����λ�� ʹ�õ�����

    uart_putbuff(UART_0, cmdf, sizeof(cmdf));    //�ȷ�������

    uart_putbuff(UART_0, (char *)imgaddr, imgsize); //�ٷ���ͼ��

    uart_putbuff(UART_0, cmdr, sizeof(cmdr));    //�ȷ�������
}
