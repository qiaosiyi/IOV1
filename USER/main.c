#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h"  
#include "key.h"  
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "rtc.h"
#include "niming.h"
#include "malloc.h"
#include "sdio_sdcard.h" 
#include "w25qxx.h"  
#include "ff.h" 
#include "exfuns.h" 
#include "text.h"
#include "touch.h"	
#include "usart3.h"
#include "sim900a.h"  
//MPU6050 ������ά���ٶȣ������������ǣ�����¼��Сֵ��ʵ�飬
//ģ���������U5�ڣ�P8��ñ��������2016��11��4��23:42:44
//qiaosiyi
int main(void){		
	u8 t=0,time=0,report=1,start=0;			//Ĭ�Ͽ����ϱ�
	u8 t60=0;
	//u8 key,fontok=0;
	u8 key;
	float pitch,roll,yaw,dertayaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	    
 	dertayaw = 0.013445;//per three second
	Init();
	//mpuDmpInit();//�ϵ�ֱ�ӳ�ʼ���ں����������
 	while(1){
		key=KEY_Scan(0);
		if(key==KEY0_PRES){
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}	
		if(key==KEY1_PRES&&start==0){
			start=1;	
		}
		if(start==1){//����֮���ٿ�ʼ��ʼ��dmp�ںϣ�������һ����ʱ��
			mpuDmpInit();
			start=2;
		}
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){ //�ɹ���ȡһ��mpu�ں�֮���ֵ
			yaw = yaw - dertayaw;//������ת���Ժ�ʹ�õشżƿ����ú����
			if(yaw < 0) yaw+=360; 
			if(yaw > 360) yaw-=360; //����yaw�ķ�Χ��0��--360��
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			temp=windowfilterT(temp);		//���¶�������ٶȽ����˲���ƽ�����ڡ�
			aacx=windowfilterAx(aacx);
			aacy=windowfilterAy(aacy);
			aacz=windowfilterAz(aacz);
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	
			if((t%13)==0){
				updateScaleLine();//�������
				showxyupdatepo(aacx,aacy,0,0);//��ָʾ��
				showXandY(aacx,aacy);
				showzupdate(aacz,RED);
				showprylineupdate(&pitch,&roll,&yaw);
				recordAcxyzMax(aacx,aacy,aacz);
				showNumtxyzrpy(temp,aacx,aacy,aacz,roll,pitch,yaw);
				t=0;
			}
		}
		if(time != calendar.sec){
			time = calendar.sec;
			showClockNum(calendar);
			if(t60%60==0){
				LED0=!LED0;
				//usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
				//printf("\t\n");
				t60=0;
				LED0=!LED0;
			}
			if(t60%3==0){
				dertayaw += 0.013445;
				LED0=!LED0;
			}
			t60++;	
		}
		t++; 
	} 	
}






