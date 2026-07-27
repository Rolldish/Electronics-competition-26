#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
//#include "board.h"
#include "stdio.h"

uint8_t OLED_GRAM[128][8];	 
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
�������ܣ�ˢ��OLED��Ļ
���ڲ�������
����  ֵ����
**************************************************************************/
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
�������ܣ���OLEDд��һ���ֽ�
���ڲ�����dat:Ҫд��������/���cmd:����/������־ 0,��ʾ����;1,��ʾ����
����  ֵ����
**************************************************************************/  
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{	
	uint8_t i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 
/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
�������ܣ�����OLED��ʾ 
���ڲ�������
����  ֵ����
**************************************************************************/  
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
Function: Turn off the OLED display
Input   : none
Output  : none
�������ܣ��ر�OLED��ʾ 
���ڲ�������			  
����  ֵ����
**************************************************************************/  
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
�������ܣ���������,������,������Ļ�Ǻ�ɫ�ģ���û����һ��
���ڲ�������		  
����  ֵ����
**************************************************************************/  
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram(); //Update the display //������ʾ
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
�������ܣ����� 
���ڲ�����x,y :��������; t:1,����,0,����			  
����  ֵ����
**************************************************************************/ 
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
�������ܣ���ָ��λ����ʾһ���ַ�,���������ַ�
���ڲ�����x,y :��������; len :���ֵ�λ��; size:������С; mode:0,������ʾ,1,������ʾ	   
����  ֵ����
**************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' '; //Get the offset value //�õ�ƫ�ƺ���ֵ				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font   //����1206����
		else temp=oled_asc2_1608[chr][t];		      //Invoke the 1608 font //����1608���� 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
�������ܣ���m��n�η��ĺ���
���ڲ�����m��������n���η���
����  ֵ����
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
�������ܣ���ʾ2������
���ڲ�����x,y :��������; len :���ֵ�λ��; size:������С; mode:ģʽ, 0,����ģʽ, 1,����ģʽ; num:��ֵ(0~4294967295);	 
����  ֵ����
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
�������ܣ���ʾ�ַ���
���ڲ�����x,y :��������; *p:�ַ�����ʼ��ַ 
����  ֵ����
**************************************************************************/
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	 
/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
�������ܣ���ʼ��OLED	
���ڲ���: �� 
����  ֵ����
**************************************************************************/	    
void OLED_Init(void)
{ 	


	OLED_RST_Clr();
	Tick_delay(120);

	OLED_RST_Set(); 
				  
	OLED_WR_Byte(0xAE,OLED_CMD); //Close display //�ر���ʾ
	OLED_WR_Byte(0xD5,OLED_CMD); //The frequency frequency factor, the frequency of the shock //����ʱ�ӷ�Ƶ����,����Ƶ��
	OLED_WR_Byte(80,OLED_CMD);   //[3:0], the frequency dividing factor;[7:4], oscillation frequency //[3:0],��Ƶ����;[7:4],����Ƶ��
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of driver paths //��������·��
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0x3f(1/64) //Ĭ��0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //Setting display deviation //������ʾƫ��
	OLED_WR_Byte(0X00,OLED_CMD); //Default is 0//Ĭ��Ϊ0

	OLED_WR_Byte(0x40,OLED_CMD); //Sets the number of rows to display starting line [5:0] //������ʾ��ʼ�� [5:0],����
													
	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump setup //���ɱ�����
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on/off //bit2������/�ر�
	OLED_WR_Byte(0x20,OLED_CMD); //Set up the memory address mode //�����ڴ���ַģʽ
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10; //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127; //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths//����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA,OLED_CMD); //Set the COM hardware pin configuration //����COMӲ����������
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]configuration //[5:4]����
	 
	OLED_WR_Byte(0x81,OLED_CMD); //Contrast Settings //�Աȶ�����
	OLED_WR_Byte(0xEF,OLED_CMD); //1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set the pre-charging cycle //����Ԥ��������
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Setting vcomh voltage multiplier//����VCOMH ��ѹ����
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //Global display; Bit0:1, open; 0, close; (white screen/black screen)//ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6,OLED_CMD); //Settings display mode; Bit0:1, anti-phase display; 0, normal display//������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display //������ʾ	 
	OLED_Clear();
}  

/**************************************************************************
Function: Display character
Input   : x: indicates the horizontal coordinates displayed; Y: the vertical coordinates that show the display;
          no: the line number in the array of the Chinese character (module) in the hzk-and "array", which is determined by the line number to determine the characters shown in the array,
          The value of the width of the font here must be consistent with the size of the dot matrix value of the use of the word mold.
          font_height: the font is high for the use of the word mold, because my screen pixels are 32hours, 128----0~ 7, and four bits per page
Output  : none
Note: this method is used to show that the Chinese character must satisfy the size of the word that the word model generates the software to generate the same size as the dot matrix
�������ܣ���ʾ����	
���ڲ���: x����ʾ��ʾ��ˮƽ����; y: ��ʾ��ʾ�Ĵ�ֱ����;
          no: ��ʾҪ��ʾ�ĺ��֣�ģ�飩��hzk[][]�����е��к�,ͨ���к���ȷ����������Ҫ��ʾ�ĺ���,
              ���������Ŀ�font_width��ֵ����������ģ��������������ģʱ�ĵ���ֵ��Сһ��;
          font_height:Ϊ����ģ��������������ģʱ�����ĸ�,�����ҵ�������Ϊ32*128-----0~7��8ҳ��ÿҳ4��λ
����  ֵ����
ע�⣺�����ַ�������ʾ����һ��Ҫ��������ģ�����������ɵ��ֿ���������С��ͬ���У�������������
**************************************************************************/	    
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height)
{     			    
	 uint8_t t, i;
   for(i=0;i<(font_height/8);i++)	//The maximum height of font_height is 32. this screen is only 8 pages (line), four digits per page
	                                //font_height����ֵΪ32��������ֻ��8��ҳ���У���ÿҳ4��λ
	 {
			OLED_Set_Pos(x,y+i);	
			for(t=0;t<font_width;t++)		//The maximum value of font_width is 128. the screen is only that large 
		                              //font_width����ֵΪ128����Ļֻ����ô��
			{	
					OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
			}		
	 }
}	 
/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
�������ܣ����ú�������Ļ����ʾ�����꣨λ�ã�
���ڲ���: x,y :��������
����  ֵ����
**************************************************************************/	  
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 	
	 OLED_WR_Byte(0xb0+y,OLED_CMD);
	 OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	 OLED_WR_Byte((x&0x0f),OLED_CMD); 
} 


void OLED_RST_Clr(void)
{
DL_GPIO_clearPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");

} 	
		  //RST
void OLED_RST_Set(void)
{
 DL_GPIO_setPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}  

void OLED_RS_Clr(void)
{

DL_GPIO_clearPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN); ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_RS_Set(void) 
{
 DL_GPIO_setPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN) ; ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	

void OLED_SCLK_Clr(void)
{

DL_GPIO_clearPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SCLK_Set(void)
{
DL_GPIO_setPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Clr(void)
{
DL_GPIO_clearPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); // //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Set(void) 
{
DL_GPIO_setPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); //   //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	

