/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K66 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v6.0
 * @date       2017-11-04
 */

#include "common.h"
#include "include.h"
#include "oled.h"

//ǿ����һ��ģ��ͬһʱ��ֻ�ܸ�һ�������Ƶ�ʱ�������ͬ�ġ�
//FTM�Ƕ๦�ܵĶ�ʱ������ʵ�ֶ��ֹ��ܡ�
//��ͬһʱ�䣬һ��FTM0��ȥPWM������Ͳ�Ҫ���������������롢���벶׽�������������顣
//���ң�FTM0��ͨ��0���Ƶ��Ϊ20k��ͨ��1Ҳ���룬ֻ�����20k��pwm��
void PIT0_IRQHandler(void);

/*!
 *  @brief      main����
 *  @since      v6.0
 *  @note       FTM PWM ����(ʾ����������ռ�ձȺ�Ƶ���Ƿ���ȷ)
 */
  struct _pid
  {
    float SetSpeed; //�����趨ֵ
    float val_real; //����ʵ��ֵ
    float val2_real; //����ʵ��ֵ
    float err; //����ƫ��ֵ
    float err_next; //������һ��ƫ��ֵ
    float err_last; //��������ǰ��ƫ��ֵ
    float Kp,Ki,Kd; //������������֡�΢��ϵ��
    float pwm;
  }pid;
  void PID_init();
  float PID_realize(float speed,float real);
unsigned char vbuf[6];
unsigned char vbuf2[6];
unsigned char pwmbuf[6];
int16 val,val2;
float pwm,pwm2;
float MOTOR_PWM_ERR(int32 val_EX,int32 val_Real);
void main(void)
{
     OLED_Init();
     PID_init();
    ftm_quad_init(FTM1,FTM_PS_1,FTM1_QDPHA_PIN,FTM1_QDPHB_PIN); //FTM1 ���������ʼ��//���õĹܽſɲ� port_cfg.h �� FTM1_QDPHA_PIN �� FTM1_QDPHB_PIN
    ftm_quad_init(FTM2,FTM_PS_1,FTM2_QDPHA_PIN,FTM2_QDPHB_PIN);
    pit_init_ms(PIT0, 10);                                 //��ʼ��PIT0����ʱʱ��Ϊ�� 500ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�

    led_init(LED0);

    ftm_pwm_init(FTM3, FTM_CH4,10*1000,0,FTM3_CH4_PIN);        //��ʼ�� FTM PWM ��ʹ�� FTM0_CH3��Ƶ��Ϊ200k ��ռ�ձ�Ϊ 30 %
    ftm_pwm_init(FTM3, FTM_CH5,10*1000,0,FTM3_CH5_PIN); // app/inc/port_cfg.h �� ���� FTM0_CH3_PIN ��ӦΪ PTA6

    ftm_pwm_init(FTM3, FTM_CH6,10*1000,0,FTM3_CH6_PIN);        //��ʼ�� FTM PWM ��ʹ�� FTM0_CH3��Ƶ��Ϊ200k ��ռ�ձ�Ϊ 30 %
    ftm_pwm_init(FTM3, FTM_CH7,10*1000,0,FTM3_CH7_PIN); // app/inc/port_cfg.h �� ���� FTM0_CH3_PIN ��ӦΪ PTA6

    while(1)
    {     
          led(LED0,0);
          ftm_pwm_duty(FTM3, FTM_CH4,0);     //����ռ�ձ� Ϊ 30%
          ftm_pwm_duty(FTM3, FTM_CH6,pwm);     //����ռ�ձ� Ϊ 30%//val

          ftm_pwm_duty(FTM3, FTM_CH5,pwm2);     //����ռ�ձ� Ϊ 0%
          ftm_pwm_duty(FTM3, FTM_CH7,0);     //����ռ�ձ� Ϊ 0%  //val2

          sprintf((char*)vbuf,"%6d",val);            //����������ֵ
          sprintf((char*)vbuf2,"%6d",val2);          // ����������ֵ 
          sprintf((char*)pwmbuf,"%6f",pwm);          // ����������ֵ 
          OLED_P6x8Str( 0, 0, "******text******");
          OLED_P6x8Str( 0, 2, vbuf);
	  OLED_P6x8Str( 0, 3, vbuf2);
          OLED_P6x8Str( 0, 5, pwmbuf);

          
    }
}
void PIT0_IRQHandler(void)
{
   
    val2 = ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)
    val = ftm_quad_get(FTM2);          //��ȡFTM �������� ��������(������ʾ������)
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);

    
     if(val2>=0)
    {
        val2=val2;
    }
    else
    {
        val2=-val2;
    }
    if(val>=0)
    {
        val=val;
    }
    else
    {
        val=-val;
    }
    pid.val_real=val*100*60/512;
    pid.val2_real=val2*100*60/512;
    pwm=PID_realize(1500,pid.val_real);
    pwm2=PID_realize(1500,pid.val2_real);
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

void PID_init()
{
    pid.SetSpeed=0.0;
    pid.val_real=0.0; 
    pid.val2_real=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.err_next=0.0;
    pid.Kp=0.09;
    pid.Ki=0.005;
    pid.Kd=0;
}
float PID_realize(float speed,float real)
{   float incrementSpeed;
    pid.SetSpeed=speed;
    pid.err=pid.SetSpeed-real;
    incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    pid.pwm+=incrementSpeed;
    pid.err_last=pid.err_next;
    pid.err_next=pid.err;
    if(pid.pwm>50.0)
    {
      pid.pwm=50;
    }
       if(pid.pwm<0)
    {
      pid.pwm=0;
    }
    
    return pid.pwm;
}
