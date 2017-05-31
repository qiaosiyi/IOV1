#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "beep.h"
 int main(void)
 {
	vu8 key=0;	
	u16 t =0;
	u8 pre = 0;
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  		//初始化与LED连接的硬件接口
	BEEP_Init();         	//初始化蜂鸣器端口
	KEY_Init();         	//初始化与按键连接的硬件接口
	LED0=0;					//先点亮红灯
	while(1)
	{
		while(key == 0){
			key = KEY_Scan(0);
			delay_ms(100);
		}
		P4G = 0;
		delay_ms(1000);
		delay_ms(1000);
		P4G = 1;
		while(1){
			key=KEY_Scan(0);	//得到键值
				   
			switch(key)
			{				 
				case WKUP_PRES:	//控制蜂鸣器
					BEEP=!BEEP;
					break; 
				case KEY1_PRES:	//控制LED1翻转	 
					LED1=!LED1;
					break;
				case KEY0_PRES:	//同时控制LED0,LED1翻转 
					LED0=!LED0;
					LED1=!LED1;
					break;
			
				default:
					if(pre == key)
						t++;
					else
						t=0;
					
			}
			
			delay_ms(10); 
			
			if(t>600){ //多长个10ms没有dog信号则重启
				RASP = 0;
				delay_ms(50);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				RASP = 1;
				delay_ms(50);
				t = 0;
				break;
			}
			pre = key;
		}
	}
}

