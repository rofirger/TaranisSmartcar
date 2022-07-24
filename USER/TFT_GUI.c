/*
 * TFT_GUI.c
 *
 *  Created on: 2022��2��13��
 *      Author: fengq
 */
//ң������ձ�
/*
 * ���ϼ�ͷ��98 206
 * ���¼�ͷ��42 106
 * �����ͷ��66 222
 * ���Ҽ�ͷ��106 74
 * OK��226 142
 *    *��
 *   #��192 158
 *   1��
 *   2��
 *   3��
 *   4��
 *   5��
 *   6��
 *   7��
 *   8��
 *   9��
 */
#include"TFT_GUI.h"
//#include "pid.h"
//#include "Balance.h"
#include "zf_ccu6_pit.h"
#include "control.h"
#include "img_process.h"
static ERU_PIN_enum TFT_Pin;
//data�����������ÿ�ε���page1ParaSaveʱ���data����ˢ��
double paraData[TotalParaNumber];
//page0Ϊ�˵�ҳ��
static char page0[10][17];
//page1XΪ����ҳ��
static char page1[10][17];
#if TotalParaNumber>9
static char page11[10][17];
#if TotalParaNumber>18
static char page12[10][17];
#if TotalParaNumber>27
static char page13[10][17];
#endif
#endif
#endif
static char page2[10][17];
//location��¼��ǰ�۽���λ�ã�location[0]�Ǿ۽�ҳ�棬location[1]�Ǿ۽�����
//ҳ��һΪ����ҳ�棬ҳ���Ϊ�۲�ͼ�������ҳ������в���
uint8 location[2];
//page1NumberΪ��ǰ���εľ۽�ҳ�����
static int page1Number = 0;
//��settingFlagΪ1ʱ�������ڶ����ֽ�������
static uint8 settingFlag = 0;
//��controlFlagΪ1ʱ�������ڿ���С��
static uint8 controlFlag = 0;
//settingBit��ʾ�������õ�λ,ȡֵ��Χ0~7
static uint8 settingBit = 0;
uint8 wifiInitOK=0;

uint8 PID_Matrix1[7][7],PID_Matrix2[7][7],PID_Matrix3[7][7];
float PID_Matrix4[3][4];

//��ʼ��TFT��Ļ
void GUI_init (ERU_PIN_enum eru_pin)
{
    //�ڴ������������
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 16; j++)
        {
            page0[i][j] = ' ';

            page1[i][j] = ' ';
#if TotalParaNumber>9
            page11[i][j] = ' ';
#if TotalParaNumber>18
            page12[i][j] = ' ';
#if TotalParaNumber>27
            page13[i][j] = ' ';
#endif
#endif
#endif
        }
    //lcd��ʼ��
    lcd_init();
    TFT_Pin = eru_pin;
    location[0] = 0;
    location[1] = 0;
    page1Number = 0;
    //ҳ���ʼ��
    page_init();
    //paraData��ʼ��
    GUI_Load_Data();
    //debug��ע���ж�
    eru_init(eru_pin, FALLING);
    refresh();
    eru_enable_interrupt(eru_pin);
}
//�������жϱ���������ô˺���
void eru_triggered ()
{
    eru_disable_interrupt(TFT_Pin);
    //str_cpy(page1[0],"Return || Page 1",16);
    //���źŽ���NEC����
    uint64 time = now() / 100;
    while (!gpio_get(P15_8))
    {
    }
    if ((now() / 100 - time < 8900) || (now() / 100 - time > 9100))
    {
        //lcd_showint32(0,2,now()/100-time,10);
        eru_enable_interrupt(TFT_Pin);
        return;
    }
    while (gpio_get(P15_8))
    {
    }
    if ((now() / 100 - time > 13000))
    {
        uint64 timeA = now() / 100;
        uint8 a = 0, b = 0;
        for (uint8 i = 0; i < 32; i++)
        {
            while (!gpio_get(P15_8))
            {
            }
            uint64 timeA1 = now() / 100 - timeA;
            while (gpio_get(P15_8))
            {
            }
            if ((now() / 100 - timeA) > (timeA1 * 3))
            {
                timeA = now() / 100;
                if (i < 25)
                {
                    a += 1;
                    a *= 2;
                }
                else
                {
                    b += 1;
                    b *= 2;
                }
            }
            else
            {
                timeA = now() / 100;
                if (i < 25)
                {
                    a *= 2;
                }
                else
                {
                    b *= 2;
                }
            }

        }
        if(wifiInitOK==0)
        received(a, b);
        else
        {
            receivedWithoutGUI(a,b);
        }
    }
    else
    {
    }
    while (!gpio_get(P15_8))
    {
    }    //�ȴ��ͷ�
    eru_enable_interrupt(TFT_Pin);
}
void wifiInit (ERU_PIN_enum eru_pin)
{
    eru_init(eru_pin, FALLING);
    spi_init(TFT_SPIN, TFT_SCL, TFT_SDA, TFT_SDA_IN, TFT_CS, 0, 10 * 1000 * 1000);  //Ӳ��SPI��ʼ��
    gpio_init(P15_4, GPI, 0, PULLUP);
    eru_enable_interrupt(eru_pin);
    wifiInitOK=1;
}
static sendToWifiCount = 0;
void sentImageToWifi (uint8 imageToWifi[][188])
{
    //���ͱ�ͷ
    uint8 headerFirst[] = {0x01, 0xcf, 0x01, 0xcf, 0x01, 0xcf, 0x01, 0xcf, 0x01, 0xcf};
    for (int i = 0; i < 10; i++)
    {
        while (gpio_get(P15_4) == 0){}
        sentCharToWifi(headerFirst[i]);
    }
    sendToWifiCount = 10;
    for (int j = 0; j < 70; j++)
    {
        for (int i = 0; i < 188; i++)
        {
            //uint8 tempDataOfImage=image[i*70+j];
            while (gpio_get(P15_4) == 0){}
            if(imageToWifi[j][i]==0)
            {
                sentCharToWifi(0x02);
            }
            else
            {
                sentCharToWifi(imageToWifi[j][i]);
            }
            sendToWifiCount++;
            //���Ѿ�������100��ͼƬ���ݺ�
            if (sendToWifiCount == 110)
            {
                //���Խ�fifo����
                while (gpio_get(P15_4) == 1)
                    sentCharToWifi(0x02);
                //fifo������
                sendToWifiCount = 0;
                //���ͷ�֡��ͷ
                uint8 headerSecond[] = {0x01, 0xef, 0x01, 0xef, 0x01, 0xef, 0x01, 0xef, 0x01, 0xef};
                for (int k = 0; k < 10; k++)
                {
                    while (gpio_get(P15_4) == 0){}
                    sentCharToWifi(headerSecond[k]);
                    while (gpio_get(P15_4) == 0){}
                }
                sendToWifiCount = 10;
            }
        }
    }
    //���Խ�fifo����
    while (gpio_get(P15_4) == 1)
        sentCharToWifi(0x02);
    while (gpio_get(P15_4) == 0)
        ;
    sendToWifiCount = 0;
    //fifo������
}
void sentCharToWifi (uint8 aCharToWifi)
{
    uint8 tempZeroData = aCharToWifi;
    spi_mosi(SPI_2, SPI2_CS0_P15_2, &tempZeroData, NULL, 1, 0);
}
void receivedWithoutGUI(uint8 a,uint8 b)
{
    if(a==42&&b==106)controlBackward();//����
    if(a==98&&b==206)controlForward();//����
    if(a==66&&b==222)controlLeft();//����
    if(a==106&&b==74)controlRight();//����
    if(a==42&&b==106)controlClear();//����
}
void drawRectangle (uint16 x, uint16 y, uint8 length, uint8 width, uint16 color)
{
    for (uint8 i = 0; i < length; i++)
    {
        lcd_drawpoint(x + i, y, color);
        lcd_drawpoint(x + i, y + 1, color);
        lcd_drawpoint(x + i, y + width - 1, color);
        lcd_drawpoint(x + i, y + width, color);

    }
    for (uint8 j = 0; j < width; j++)
    {
        lcd_drawpoint(x, y + j, color);
        lcd_drawpoint(x + 1, y + j, color);
        lcd_drawpoint(x + length - 1, y + j, color);
        lcd_drawpoint(x + length, y + j, color);
    }
}
//���ݲ�����ӡ��ҳ��page1��page11��
void refresh ()
{
    GUI_limited();
    lcd_clear(0xFFFF);
    if (location[0] == 1)
    {
        //��ӡ��һҳ
        if (page1Number == 0)
        {
            for (int i = 0; i < 10; i++)
            {
                page1[i][16] = 0;
            }
            for (int y = 0; y < 10; y++)
            {
                for (int j = 0; j < 16; j++)
                {
                    lcd_showchar(j * 8, y * 16, page1[y][j]);
                }
            }
        }
#if TotalParaNumber>9
        //��ӡpage11
        if (page1Number == 1)
        {
            for (int i = 0; i < 10; i++)
            {
                page11[i][16] = 0;
            }
            for (int y = 0; y < 10; y++)
            {
                for (int j = 0; j < 16; j++)
                {
                    lcd_showchar(j * 8, y * 16, page11[y][j]);
                }
            }
        }
#if TotalParaNumber > 18
        //��ӡpage12
        if (page1Number == 2)
        {
            for (int i = 0; i < 10; i++)
            {
                page12[i][16] = 0;
            }

            for (int y = 0; y < 10; y++)
            {
                for (int j = 0; j < 16; j++)
                {
                    lcd_showchar(j * 8, y * 16, page12[y][j]);
                }
            }
        }
#if TotalParaNumber > 27
        //��ӡpage13
        if (page1Number == 3)
        {
            for (int i = 0; i < 10; i++)
            {
                page13[i][16] = 0;
            }
            for (int y = 0; y < 10; y++)
            {
                for (int j = 0; j < 16; j++)
                {
                    lcd_showchar(j * 8, y * 16, page13[y][j]);
                }
            }
        }
#endif
#endif
#endif
        drawRectangle(0, location[1] * 16, 127, 15, BLACK);
        if (settingFlag == 1)
        {
            drawRectangle(0, location[1] * 16, 127, 15, RED);
            drawRectangle(8 * 7 + settingBit * 8 - 1, location[1] * 16, 8, 15, GREEN);
        }
    }
    //��ӡ��ҳ
    if (location[0] == 0)
    {
        for (int i = 0; i < 10; i++)
        {
            page0[i][16] = 0;
        }
        for (int y = 0; y < 10; y++)
        {
            for (int j = 0; j < 16; j++)
            {
                lcd_showchar(j * 8, y * 16, page0[y][j]);
            }
        }
        drawRectangle(0, location[1] * 16, 127, 15, BLACK);
    }
    //��ӡͼƬҳ
    if (location[0] == 2)
    {
        for (int i = 0; i < 10; i++)
        {
            page2[i][16] = 0;
        }
        for (int y = 0; y < 10; y++)
        {
            for (int j = 0; j < 16; j++)
            {
                lcd_showchar(j * 8, y * 16, page2[y][j]);
            }
        }
        //lcd_showint16(6 * 8, 7, target_speed);
    }

}
void str_cpy (char *Dest, char *Source, uint8 length)
{
    for (int i = 0; i < length; i++)
    {
        Dest[i] = Source[i];
    }
}
void received (uint8 a, uint8 b)
{
    if ((a == 98) && (b == 206))    //NEC����Ϊ���ϼ�ͷ
    {
        if (controlFlag == 1)
        {
            controlForward();
        }
        if ((settingFlag == 0) && (controlFlag == 0))
        {
            //page1X

            if (location[1] == 0)
            {
                location[1] = 9;
            }
            else if (location[1] > 0)
            {
                location[1]--;
            }
        }
        else
        {
            //page1ʱ���ϵ���
            if (page1Number == 0)
            {
                if ((page1[location[1]][settingBit + 7]) < 58)
                {
                    if (page1[location[1]][settingBit + 7] == 46)
                        page1[location[1]][settingBit + 7] = 48;
                    else
                        page1[location[1]][settingBit + 7]++;
                }
                if ((page1[location[1]][settingBit + 7]) < 48)
                {
                    page1[location[1]][settingBit + 7] = 46;
                }
                if ((page1[location[1]][settingBit + 7]) > 57)
                {
                    page1[location[1]][settingBit + 7] = 46;
                }
            }
#if TotalParaNumber>9
            //page11ʱ���ϵ���
            else if (page1Number == 1)
            {
                if ((page11[location[1]][settingBit + 7]) < 58)
                {
                    if (page11[location[1]][settingBit + 7] == 46)
                        page11[location[1]][settingBit + 7] = 48;
                    else
                        page11[location[1]][settingBit + 7]++;
                }
                if ((page11[location[1]][settingBit + 7]) < 48)
                {
                    page11[location[1]][settingBit + 7] = 46;
                }
                if ((page11[location[1]][settingBit + 7]) > 57)
                {
                    page11[location[1]][settingBit + 7] = 46;
                }
            }
            //page12ʱ���ϵ���
#if TotalParaNumber>18
            else if (page1Number == 2)
            {
                if ((page12[location[1]][settingBit + 7]) < 58)
                {
                    if (page12[location[1]][settingBit + 7] == 46)
                        page12[location[1]][settingBit + 7] = 48;
                    else
                        page12[location[1]][settingBit + 7]++;
                }
                if ((page12[location[1]][settingBit + 7]) < 48)
                {
                    page12[location[1]][settingBit + 7] = 46;
                }
                if ((page12[location[1]][settingBit + 7]) > 57)
                {
                    page12[location[1]][settingBit + 7] = 46;
                }
            }
#if TotalParaNumber>27
            else if (page1Number == 3)
            {
                if ((page13[location[1]][settingBit + 7]) < 58)
                {
                    if (page13[location[1]][settingBit + 7] == 46)
                        page13[location[1]][settingBit + 7] = 48;
                    else
                        page13[location[1]][settingBit + 7]++;
                }
                if ((page13[location[1]][settingBit + 7]) < 48)
                {
                    page13[location[1]][settingBit + 7] = 46;
                }
                if ((page13[location[1]][settingBit + 7]) > 57)
                {
                    page13[location[1]][settingBit + 7] = 46;
                }
            }
#endif
#endif
#endif
        }
    }
    if ((a == 42) && (b == 106))    //NEC����Ϊ���¼�ͷ
    {
        if (controlFlag == 1)
        {
            controlBackward();
        }
        if ((settingFlag == 0) && (controlFlag == 0))
        {

            if (location[1] < 9)
            {
                location[1]++;
            }
            else if (location[1] == 9)
            {
                location[1] = 0;
            }

        }
        else
        {
            //page1ʱ���µ���
            if (page1Number == 0)
            {
                if ((page1[location[1]][settingBit + 7] == 46) || (page1[location[1]][settingBit + 7]) > 47)
                {
                    if (page1[location[1]][settingBit + 7] == 46)
                        page1[location[1]][settingBit + 7] = 48;
                    else
                        page1[location[1]][settingBit + 7]--;
                }
                if ((page1[location[1]][settingBit + 7]) < 48)
                {
                    page1[location[1]][settingBit + 7] = 46;
                }
                if ((page1[location[1]][settingBit + 7]) > 57)
                {
                    page1[location[1]][settingBit + 7] = 46;
                }
            }
#if TotalParaNumber>9
            //page11ʱ���µ���
            else if (page1Number == 1)
            {
                if ((page11[location[1]][settingBit + 7] == 46) || (page11[location[1]][settingBit + 7]) > 47)
                {
                    if (page11[location[1]][settingBit + 7] == 46)
                        page11[location[1]][settingBit + 7] = 48;
                    else
                        page11[location[1]][settingBit + 7]--;
                }
                if ((page11[location[1]][settingBit + 7]) < 48)
                {
                    page11[location[1]][settingBit + 7] = 46;
                }
                if ((page11[location[1]][settingBit + 7]) > 57)
                {
                    page11[location[1]][settingBit + 7] = 46;
                }
            }
#if TotalParaNumber>18
            //page12ʱ���µ���
            else if (page1Number == 2)
            {
                if ((page12[location[1]][settingBit + 7] == 46) || (page12[location[1]][settingBit + 7]) > 47)
                {
                    if (page12[location[1]][settingBit + 7] == 46)
                        page12[location[1]][settingBit + 7] = 48;
                    else
                        page12[location[1]][settingBit + 7]--;
                }
                if ((page12[location[1]][settingBit + 7]) < 48)
                {
                    page12[location[1]][settingBit + 7] = 46;
                }
                if ((page12[location[1]][settingBit + 7]) > 57)
                {
                    page12[location[1]][settingBit + 7] = 46;
                }
            }
#if TotalParaNumber > 27
            //page13ʱ���µ���
            else if (page1Number == 3)
            {
                if ((page13[location[1]][settingBit + 7] == 46) || (page13[location[1]][settingBit + 7]) > 47)
                {
                    if (page13[location[1]][settingBit + 7] == 46)
                        page13[location[1]][settingBit + 7] = 48;
                    else
                        page13[location[1]][settingBit + 7]--;
                }
                if ((page13[location[1]][settingBit + 7]) < 48)
                {
                    page13[location[1]][settingBit + 7] = 46;
                }
                if ((page13[location[1]][settingBit + 7]) > 57)
                {
                    page13[location[1]][settingBit + 7] = 46;
                }
            }
#endif
#endif
#endif
        }
    }
    if ((a == 226) && (b == 142))    //NEC����OK��
    {
        //location[0]Ϊ�۽�ҳ�棬location[1]Ϊ�۽�����
        //��page1���水��OK��������״̬
        //�ٴΰ���OK�˳����ý���
        if ((location[1] != 0) && (location[0] == 1))
        {
            if (settingFlag == 0)
                settingFlag = 1;
            else if (settingFlag == 1)
            {
                if (GUI_Para_Check())
                {
                    page1ParaSave();
                    GUI_Load_Data();
                    settingFlag ^= 1;
                    settingBit = 0;
                }
            }
        }
        //����page1��returnѡ��ʱ������OK���ز˵�
        else if ((location[1] == 0) && (location[0] == 1))
        {
            location[0] = 0;
        }
        //�����ڲ˵�page0�ĵ�һ��ʱ������OK�����һҳ
        else if (location[0] == 0 && (location[1] == 1))
        {
            location[0] = 1;
        }
        //�����ڲ˵�page1�ĵڶ���ʱ������OK������ڶ�ҳͬʱ����controlģʽ
        else if ((location[0] == 0) && (location[1] == 2))
        {
            location[0] = 2;
            location[1] = 1;
            controlFlag = 1;
        }
        //�����ڲ˵�page0�ĵ�����ʱ������ok�������ҳ
        else if ((location[0] == 0) && (location[1] == 3))
        {
            location[0] = 3;
            location[1] = 1;
        }
    }
    if ((a == 192) && (b == 158) && (settingFlag == 0))        //NEC����Ϊ#�Ҳ���������״̬��
    {
        location[0] = 0;
        location[1] = 1;
        //�˳�controlģʽ
        if (controlFlag == 1)
        {
            controlFlag = 0;
            controlClear();
        }
    }
    //controlģʽ�°������
    if ((a == 66) && (b == 222) && (controlFlag == 1))
    {
        controlLeft();
    }
    if ((a == 66) && (b == 222) && (settingFlag == 1))        //�����ͷ���Ҵ�������ģʽ
    {
        if (settingBit > 0)
            settingBit--;
        if (settingBit < 0)
            settingBit = 0;
        if (settingBit > 7)
            settingBit = 7;
    }
    if ((a == 66) && (b == 222) && (settingFlag == 0) && (location[0] == 1))        //�����ͷ���Ҳ���������ģʽ
    {
        page1Number--;
        if (page1Number < 0)
        {
            page1Number = (TotalParaNumber - 1) / 9;
        }
    }
    //controlģʽ�°����Ҽ�
    if ((a == 106) && (b == 74) && (controlFlag == 1))
    {
        controlRight();
    }
    if ((a == 106) && (b == 74) && (settingFlag == 1))        //���Ҽ�ͷ�Ҵ�������ģʽ
    {
        if (settingBit < 7)
            settingBit++;
        if (settingBit < 0)
            settingBit = 0;
        if (settingBit > 7)
            settingBit = 7;
    }
    if ((a == 106) && (b == 74) && (settingFlag == 0) && (location[0] == 1))        //���Ҽ�ͷ�Ҳ���������ģʽ
    {
        page1Number++;
        if (page1Number > ((TotalParaNumber - 1) / 9))
            page1Number = 0;
    }
    if (settingFlag == 1)
    {
        if ((a == 136) && (b == 186))        //NEC����Ϊ1
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '1';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '1';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '1';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '1';
#endif
            settingBit++;
        }
        else if ((a == 138) && (b == 58))        //NEC����Ϊ2
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '2';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '2';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '2';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '2';
#endif
            settingBit++;
        }
        else if ((a == 136) && (b == 58))        //NEC����Ϊ3
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '3';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '3';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '3';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '3';
#endif
            settingBit++;
        }
        else if ((a == 138) && (b == 186))        //NEC����Ϊ4
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '4';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '4';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '4';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '4';
#endif
            settingBit++;
        }
        else if ((a == 10) && (b == 250))        //NEC����Ϊ5
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '5';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '5';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '5';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '5';
#endif
            settingBit++;
        }
        else if ((a == 8) && (b == 122))        //NEC����Ϊ6
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '6';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '6';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '6';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '6';
#endif
            settingBit++;
        }
        else if ((a == 128) && (b == 62))        //NEC����Ϊ7
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '7';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '7';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '7';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '7';
#endif
            settingBit++;
        }
        else if ((a == 160) && (b == 174))        //NEC����Ϊ8
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '8';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '8';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '8';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '8';
#endif
            settingBit++;
        }
        else if ((a == 64) && (b == 222))        //NEC����Ϊ9
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '9';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '9';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '9';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '9';
#endif
            settingBit++;
        }
        else if ((a == 162) && (b == 46))        //NEC����Ϊ*����*��ΪС����
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '.';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '.';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '.';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '.';
#endif
            settingBit++;
        }
        else if ((a == 96) && (b == 206))        //NEC����Ϊ0
        {
            if (page1Number == 0)
                page1[location[1]][settingBit + 7] = '0';
#if TotalParaNumber>9
            else if (page1Number == 1)
                page11[location[1]][settingBit + 7] = '0';
#endif
#if TotalParaNumber>18
            else if (page1Number == 2)
                page12[location[1]][settingBit + 7] = '0';
#endif
#if TotalParaNumber>27
            else if (page1Number == 3)
                page13[location[1]][settingBit + 7] = '0';
#endif
            settingBit++;
        }
        if (settingBit < 0)
            settingBit = 0;
        if (settingBit > 7)
            settingBit = 7;
    }
    refresh();
}
void page_init ()
{
    str_cpy(page0[0], "****WELCOME****", 15);
    str_cpy(page0[1], "|  Set Param   |", 16);
    str_cpy(page0[2], "| Control Mode |", 16);
    str_cpy(page0[3], "|  Image View  |", 16);
    str_cpy(page1[0], "Return || Page 1", 16);        //paraXX�е�XX������������paraData[XX]
    str_cpy(page1[1], "str_NP:", 7);
    str_cpy(page1[2], "str_ND:", 7);
    str_cpy(page1[3], "left_P:", 7);
    str_cpy(page1[4], "left_I:", 7);
    str_cpy(page1[5], "rightP:", 7);
    str_cpy(page1[6], "rightI:", 7);
    str_cpy(page1[7], "bend_P:", 7);
    str_cpy(page1[8], "bend_I:", 7);
    str_cpy(page1[9], "bend_D:", 7);
#if TotalParaNumber>9
    str_cpy(page11[0], "Return || Page 2", 16);
    str_cpy(page11[1], "speedL:", 7);
    str_cpy(page11[2], "speedR:", 7);
    str_cpy(page11[3], "sp_Nst:", 7);
    str_cpy(page11[4], "sp_Ned:", 7);
    str_cpy(page11[5], "str_SP:", 7);
    str_cpy(page11[6], "str_SD:", 7);
    str_cpy(page11[7], "sp_Sst:", 7);
    str_cpy(page11[8], "sp_Sed:", 7);
    str_cpy(page11[9], "******:", 7);
#if TotalParaNumber>18
    str_cpy(page12[0], "Return || Page 3", 16);
    str_cpy(page12[1], "dif_Ls:", 7);     //22
    str_cpy(page12[2], "dif_Rs:", 7);       //22
    str_cpy(page12[3], "acc_rt:", 7);     //22
    str_cpy(page12[4], "gyro_r:", 7);     //21
    str_cpy(page12[5], "bendTH:", 7);      //22
    str_cpy(page12[6], "O_bdTH:", 7);
    str_cpy(page12[7], "NOfsTH:", 7);
    str_cpy(page12[8], "bdSSPD:", 7);
    str_cpy(page12[9], "******:", 7);
#if TotalParaNumber>27
    str_cpy(page13[0], "Return || Page 4", 16);
    str_cpy(page13[1], "******:", 7);
    str_cpy(page13[2], "******:", 7);
    str_cpy(page13[3], "******:", 7);
    str_cpy(page13[4], "******:", 7);
    str_cpy(page13[5], "******:", 7);
    str_cpy(page13[6], "******:", 7);
    str_cpy(page13[7], "******:", 7);
    str_cpy(page13[8], "******:", 7);
    str_cpy(page13[9], "******:", 7);
#endif
#endif
#endif
    str_cpy(page2[0], "        ^       ", 16);
    str_cpy(page2[1], "        |       ", 16);
    str_cpy(page2[2], "        |       ", 16);
    str_cpy(page2[3], "<--           ->", 16);
    str_cpy(page2[4], "        |       ", 16);
    str_cpy(page2[5], "        |       ", 16);
    str_cpy(page2[6], "        v       ", 16);
    str_cpy(page2[7], "                ", 16);
    str_cpy(page2[8], "                ", 16);
    str_cpy(page2[9], "                ", 16);
    page1ParaInit();
}
//��flash��ȡ��page1��page11������
void page1ParaInit ()
{
    uint32 temp_data_H, temp_data_L;
    for (int i = 0; i < 9; i++)
    {
        //��flash�ж�ȡpage1
        temp_data_H = flash_read(11, (1023 - 2 * i), uint32);
        temp_data_L = flash_read(11, (1022 - 2 * i), uint32);
        for (int j = 0; j < 4; j++)
        {
            page1[i + 1][7 + j] = (uint8) (temp_data_H >> (8 * j));
            page1[i + 1][11 + j] = (uint8) (temp_data_L >> (8 * j));
        }
#if TotalParaNumber>9
        //��flash�ж�ȡpage11
        temp_data_H = flash_read(11, (1005 - 2 * i), uint32);
        temp_data_L = flash_read(11, (1004 - 2 * i), uint32);
        for (int j = 0; j < 4; j++)
        {
            page11[i + 1][7 + j] = (uint8) (temp_data_H >> (8 * j));
            page11[i + 1][11 + j] = (uint8) (temp_data_L >> (8 * j));
        }
#if TotalParaNumber>18
        //��flash�ж�ȡpage12
        temp_data_H = flash_read(11, (987 - 2 * i), uint32);
        temp_data_L = flash_read(11, (986 - 2 * i), uint32);
        for (int j = 0; j < 4; j++)
        {
            page12[i + 1][7 + j] = (uint8) (temp_data_H >> (8 * j));
            page12[i + 1][11 + j] = (uint8) (temp_data_L >> (8 * j));
        }
#if TotalParaNumber>27
        //��flash�ж�ȡpage13
        temp_data_H = flash_read(11, (969 - 2 * i), uint32);
        temp_data_L = flash_read(11, (968 - 2 * i), uint32);
        for (int j = 0; j < 4; j++)
        {
            page13[i + 1][7 + j] = (uint8) (temp_data_H >> (8 * j));
            page13[i + 1][11 + j] = (uint8) (temp_data_L >> (8 * j));
        }
#endif
#endif
#endif
    }
}
//��page1X��������ݴ���flash
void page1ParaSave ()
{
    eeprom_erase_sector(11);
    uint32 temp_data_H, temp_data_L;
    for (int i = 1; i < 10; i++)
    {
        //����page1
        temp_data_H = (page1[i][10] << 24) + (page1[i][9] << 16) + (page1[i][8] << 8) + page1[i][7];
        temp_data_L = (page1[i][14] << 24) + (page1[i][13] << 16) + (page1[i][12] << 8) + page1[i][11];
        eeprom_page_program(11, 1025 - i * 2, &temp_data_H);
        eeprom_page_program(11, 1024 - i * 2, &temp_data_L);
#if TotalParaNumber>9
        //�洢page11
        temp_data_H = (page11[i][10] << 24) + (page11[i][9] << 16) + (page11[i][8] << 8) + page11[i][7];
        temp_data_L = (page11[i][14] << 24) + (page11[i][13] << 16) + (page11[i][12] << 8) + page11[i][11];
        eeprom_page_program(11, 1007 - i * 2, &temp_data_H);
        eeprom_page_program(11, 1006 - i * 2, &temp_data_L);
#if TotalParaNumber>18
        //�洢page12
        temp_data_H = (page12[i][10] << 24) + (page12[i][9] << 16) + (page12[i][8] << 8) + page12[i][7];
        temp_data_L = (page12[i][14] << 24) + (page12[i][13] << 16) + (page12[i][12] << 8) + page12[i][11];
        eeprom_page_program(11, 989 - i * 2, &temp_data_H);
        eeprom_page_program(11, 988 - i * 2, &temp_data_L);
#if TotalParaNumber>27
        //�洢page13
        temp_data_H = (page13[i][10] << 24) + (page13[i][9] << 16) + (page13[i][8] << 8) + page13[i][7];
        temp_data_L = (page13[i][14] << 24) + (page13[i][13] << 16) + (page13[i][12] << 8) + page13[i][11];
        eeprom_page_program(11, 971 - i * 2, &temp_data_H);
        eeprom_page_program(11, 970 - i * 2, &temp_data_L);
#endif
#endif
#endif
    }
}
//���Ʋ˵���page1X
void GUI_limited ()
{
    //������ѡ�в˵��е�welcome
    if ((location[0] == 0) && (location[1] == 0))
        location[1] = 1;
    //�޶����˵�ֻ����ѡ������
    if ((location[0] == 0) && (location[1] > 3))
        location[1] = 3;
    //�޶�page1X����ֻ��9��
    if ((location[0] == 1) && (location[1] > 9))
        location[1] = 9;
}
//���page1��page11��14������
uint8 GUI_Para_Check ()
{
    //���page1
    for (int i = 0; i < 9; i++)
    {
        uint8 floatCounter = 0;

        for (int j = 0; j < 8; j++)
        {
            if (page1[i + 1][j + 7] == 46)
                floatCounter++;
        }
        if (floatCounter > 1)
            return 0;
    }
#if TotalParaNumber>9
    //���page11
    for (int i = 0; i < 9; i++)
    {
        uint8 floatCounter = 0;
        for (int j = 0; j < 8; j++)
        {
            if (page11[i + 1][j + 7] == 46)
                floatCounter++;
        }
        if (floatCounter > 1)
            return 0;
    }
#if TotalParaNumber>18
    //���page12
    for (int i = 0; i < 9; i++)
    {
        uint8 floatCounter = 0;
        for (int j = 0; j < 8; j++)
        {
            if (page12[i + 1][j + 7] == 46)
                floatCounter++;
        }
        if (floatCounter > 1)
            return 0;
    }
#if TotalParaNumber>27
    //���page13
    for (int i = 0; i < 9; i++)
    {
        uint8 floatCounter = 0;
        for (int j = 0; j < 8; j++)
        {
            if (page13[i + 1][j + 7] == 46)
                floatCounter++;
        }
        if (floatCounter > 1)
            return 0;
    }
#endif
#endif
#endif
    return 1;
}
//��ǰ����14������
void GUI_Load_Data ()
{
    //��ȡ��Ϊ�ַ�����page1д��paraData
    for (int h = 0; h < 9; h++)
    {
        paraData[h] = 0;
        float pointerFloat = 0;
        for (float i = 7; i < 15; i++)
        {
            if (page1[h + 1][(int) i] == 46)
            {
                pointerFloat = i;
                break;
            }
        }
        if (pointerFloat != 0)
        {
            for (int j = pointerFloat + 1; j < 15; j++)
            {

                double tempData = (page1[1 + h][j] - 48);
                for (int k = j - pointerFloat; k > 0; k--)
                {
                    tempData = tempData / 10;
                }
                paraData[h] += tempData;
            }
            for (int j = pointerFloat - 1; j > 6; j--)
            {
                double tempData = (page1[h + 1][j] - 48);
                for (int k = pointerFloat - j; k > 1; k--)
                {
                    tempData *= 10;
                }
                paraData[h] += tempData;
            }
        }
        if (pointerFloat == 0)
        {

            for (int j = 14; j > 6; j--)
            {
                double tempData = 0;
                tempData = (page1[h + 1][j] - 48);
                for (int k = 14 - j; k > 0; k--)
                {
                    tempData = tempData * 10;
                }
                paraData[h] += tempData;
            }
        }
    }
#if TotalParaNumber>9
    for (int h = 0; h < 9; h++)
    {
        //��ȡ��Ϊ�ַ�����page11д��paraData
        paraData[h + 9] = 0;
        float pointerFloat = 0;
        for (float i = 7; i < 15; i++)
        {
            if (page11[h + 1][(int) i] == 46)
            {
                pointerFloat = i;
                break;
            }
        }
        if (pointerFloat != 0)
        {
            for (int j = pointerFloat + 1; j < 15; j++)
            {

                double tempData = (page11[1 + h][j] - 48);
                for (int k = j - pointerFloat; k > 0; k--)
                {
                    tempData = tempData / 10;
                }
                paraData[h + 9] += tempData;
            }
            for (int j = pointerFloat - 1; j > 6; j--)
            {
                double tempData = (page11[h + 1][j] - 48);
                for (int k = pointerFloat - j; k > 1; k--)
                {
                    tempData *= 10;
                }
                paraData[h + 9] += tempData;
            }
        }
        if (pointerFloat == 0)
        {

            for (int j = 14; j > 6; j--)
            {
                double tempData = 0;
                tempData = (page11[h + 1][j] - 48);
                for (int k = 14 - j; k > 0; k--)
                {
                    tempData = tempData * 10;
                }
                paraData[h + 9] += tempData;
            }
        }
    }
#if TotalParaNumber>18
    for (int h = 0; h < 9; h++)
    {
        //��ȡ��Ϊ�ַ�����page12д��paraData
        paraData[h + 18] = 0;
        float pointerFloat = 0;
        for (float i = 7; i < 15; i++)
        {
            if (page12[h + 1][(int) i] == 46)
            {
                pointerFloat = i;
                break;
            }
        }
        if (pointerFloat != 0)
        {
            for (int j = pointerFloat + 1; j < 15; j++)
            {

                double tempData = (page12[1 + h][j] - 48);
                for (int k = j - pointerFloat; k > 0; k--)
                {
                    tempData = tempData / 10;
                }
                paraData[h + 18] += tempData;
            }
            for (int j = pointerFloat - 1; j > 6; j--)
            {
                double tempData = (page12[h + 1][j] - 48);
                for (int k = pointerFloat - j; k > 1; k--)
                {
                    tempData *= 10;
                }
                paraData[h + 18] += tempData;
            }
        }
        if (pointerFloat == 0)
        {

            for (int j = 14; j > 6; j--)
            {
                double tempData = 0;
                tempData = (page12[h + 1][j] - 48);
                for (int k = 14 - j; k > 0; k--)
                {
                    tempData = tempData * 10;
                }
                paraData[h + 18] += tempData;
            }
        }
    }
#if TotalParaNumber>27
    for (int h = 0; h < 9; h++)
    {
        //��ȡ��Ϊ�ַ�����page13д��paraData
        paraData[h + 27] = 0;
        float pointerFloat = 0;
        for (float i = 7; i < 15; i++)
        {
            if (page13[h + 1][(int) i] == 46)
            {
                pointerFloat = i;
                break;
            }
        }
        if (pointerFloat != 0)
        {
            for (int j = pointerFloat + 1; j < 15; j++)
            {

                double tempData = (page13[1 + h][j] - 48);
                for (int k = j - pointerFloat; k > 0; k--)
                {
                    tempData = tempData / 10;
                }
                paraData[h + 27] += tempData;
            }
            for (int j = pointerFloat - 1; j > 6; j--)
            {
                double tempData = (page13[h + 1][j] - 48);
                for (int k = pointerFloat - j; k > 1; k--)
                {
                    tempData *= 10;
                }
                paraData[h + 27] += tempData;
            }
        }
        if (pointerFloat == 0)
        {

            for (int j = 14; j > 6; j--)
            {
                double tempData = 0;
                tempData = (page13[h + 1][j] - 48);
                for (int k = 14 - j; k > 0; k--)
                {
                    tempData = tempData * 10;
                }
                paraData[h + 27] += tempData;
            }
        }
    }
#endif
#endif
#endif

    //�˴����ⲿ������ֵ
    PID_Matrix1_val(PID_Matrix1[0]);
    PID_Matrix2_val(PID_Matrix2[0]);
    PID_Matrix3_val(PID_Matrix3[0]);
    PID_Matrix4_val(PID_Matrix4[0]);

    // page 1
    pid_steer.P = paraData[0];
    pid_steer.D = paraData[1];
    pid_motor_left.P = paraData[2];
    pid_motor_left.I = paraData[3];
    pid_motor_right.P = paraData[4];
    pid_motor_right.I = paraData[5];
    pid_sharp_bend.P = paraData[6];
    pid_sharp_bend.I = paraData[7];
    pid_sharp_bend.D = paraData[8];


    // page 2
    LEFT_SPEED_BASE = paraData[9];
    RIGHT_SPEED_BASE = paraData[10];
    slope_cal._start_cal_y = paraData[11];
    slope_cal._end_cal_y = paraData[12];
    pid_steer_sharp.P = paraData[13];
    pid_steer_sharp.D = paraData[14];
    sharp_slope_cal._start_cal_y = paraData[15];
    sharp_slope_cal._end_cal_y = paraData[16];

    // page3
    diff_speed._left_speed_factor = paraData[18];
    diff_speed._right_speed_factor = paraData[19];
    slide_acc_ratio = paraData[20];
    slide_gyro_ratio = paraData[21];
    bend_threshold = paraData[22];
    out_bend_threshold = paraData[23];
    no_bend_offset_threshold = paraData[24];
    sharp_bend_sub_speed = paraData[25];
}


uint8 GET_NUMBER(double pdata , int nwei)
{
    int temp_n = 1;
    int temp_data = 0;
    for (int i = nwei; i < 8; i++)
    {
        temp_n = temp_n * 10;
    }
    temp_data = (int)pdata / temp_n;
    temp_data = temp_data % 10;
    return (uint8)temp_data;
}


void PID_Matrix1_val(uint8 *PID_Matrix11)
{
    for (int i = 0; i <= 6; i++)
    {
        for (int j = 1; j <= 7; j++)
        {
            PID_Matrix11[i*7+j-1] = GET_NUMBER(paraData[i], j);
        }
    }
}


void PID_Matrix2_val(uint8 *PID_Matrix22)
{
    for (int i = 9; i <= 15; i++)
    {
        for (int j = 1; j <= 7; j++)
        {
            PID_Matrix22[(i-9)*7+j-1] = GET_NUMBER(paraData[i], j);
        }
    }
}


void PID_Matrix3_val(uint8 *PID_Matrix33)
{
    for (int i = 18; i <= 24; i++)
    {
        for (int j = 1; j <= 7; j++)
        {
            PID_Matrix33[(i-18)*7+j-1] = GET_NUMBER(paraData[i], j);
        }
    }
}


void PID_Matrix4_val(float *PID_Matrix44)
{
    PID_Matrix44[0] = paraData[7];
    PID_Matrix44[1] = paraData[8];
    PID_Matrix44[2] = paraData[16];
    PID_Matrix44[3] = paraData[17];
    PID_Matrix44[4] = paraData[25];
    PID_Matrix44[5] = paraData[26];
    PID_Matrix44[6] = paraData[27];
    PID_Matrix44[7] = paraData[28];
    PID_Matrix44[8] = paraData[29];
    PID_Matrix44[9] = paraData[30];
    PID_Matrix44[10] = paraData[31];
    PID_Matrix44[11] = paraData[32];
}



 void controlLeft()
 {

 }
 void controlRight()
 {

 }
 static int speed_confor = 0;
 void controlForward()
 {

     speed_confor = speed_confor + 150;
     pwm_duty(ATOM1_CH4_P02_4,speed_confor);
     pwm_duty(ATOM1_CH5_P02_5,speed_confor);
     pwm_duty(ATOM1_CH6_P02_6,0);
     pwm_duty(ATOM1_CH7_P02_7,0);
 }
#include "img_process.h"
extern bool is_go;
 void controlBackward()
 {
     is_go = false;
     speed_confor = 0;
     pwm_duty(ATOM1_CH4_P02_4,0);
     pwm_duty(ATOM1_CH5_P02_5,0);
     pwm_duty(ATOM1_CH6_P02_6,0);
     pwm_duty(ATOM1_CH7_P02_7,0);

 }
 void controlClear(){}


