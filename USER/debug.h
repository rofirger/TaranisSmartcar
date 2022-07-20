#ifndef __DEBUG_H__
#define __DEBUG_H__
//取一个数据的各个位
extern float Variable[16];
extern float Parameter[14];

extern char send_data,SendPara;



void Testdata_generate();
void Send_Begin();
void Send_Parameter();
void Send_Variable();
void UART3_RX_IRQHandler(uint16 bytereceive);
void send_picture();

#endif
