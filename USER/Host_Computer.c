#include "headfile.h"
extern char SendPara;
void supermonitor()//名优科创上位机
{
  if(SendPara)
  {
    SendPara=0;
    Send_Parameter();//参数回传给上位机
  }
  Testdata_generate();//设置需要发送的参数
  Send_Variable(); //发送参数
  Send_Inductor();//电磁上位机，可以使用
  //send_picture();//在校赛板子上用不了，因为名优科创的摄像头上位机的bug只支持80*60的分辨率   使用山外上位机吧
  Send_Begin();
}






//山外多功能调试助手上位机，虚拟示波器显示函数
//wareaddr    波形数组起始地址
//waresize    波形数组占用空间的大小
//使用例程     如 vcan_sendware((uint8_t *)data, sizeof(data));    data是个数组
void vcan_sendware(void *wareaddr, uint32 waresize)
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令
  
  uart_putbuff(UART_0, cmdf, sizeof(cmdf));    //先发送前命令
  uart_putbuff(UART_0, (char *)wareaddr, sizeof(wareaddr));   //发送数据
  uart_putbuff(UART_0, cmdr, sizeof(cmdr));    //发送后命令
}





// 山外多功能调试助手上位机，摄像头显示函数
//imgaddr    图像起始地址
//imgsize    图像占用空间的大小
//使用例程如   vcan_sendimg(imgadd,row_num*col_num);   imgadd为需要传输的图像数据
void vcan_sendimg(void *imgaddr, uint32 imgsize)
{
#define CMD_IMG     1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //山外上位机 使用的命令
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //山外上位机 使用的命令

    uart_putbuff(UART_0, cmdf, sizeof(cmdf));    //先发送命令

    uart_putbuff(UART_0, (char *)imgaddr, imgsize); //再发送图像

    uart_putbuff(UART_0, cmdr, sizeof(cmdr));    //先发送命令
}
