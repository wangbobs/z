/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K66 平台主程序
 * @author     山外科技
 * @version    v6.0
 * @date       2017-11-04
 */

#include "common.h"
#include "include.h"
#include "oled.h"

//强调：一个模块同一时间只能干一个活，而且频率必须是相同的。
//FTM是多功能的定时器，可实现多种功能。
//但同一时间，一个FTM0拿去PWM输出，就不要再用来做正交解码、输入捕捉，或者其他事情。
//而且，FTM0的通道0输出频率为20k，通道1也必须，只能输出20k的pwm。
void PIT0_IRQHandler(void);

/*!
 *  @brief      main函数
 *  @since      v6.0
 *  @note       FTM PWM 测试(示波器来测试占空比和频率是否正确)
 */
  struct _pid
  {
    float SetSpeed; //定义设定值
    float val_real; //定义实际值
    float val2_real; //定义实际值
    float err; //定义偏差值
    float err_next; //定义上一个偏差值
    float err_last; //定义最上前的偏差值
    float Kp,Ki,Kd; //定义比例、积分、微分系数
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
    ftm_quad_init(FTM1,FTM_PS_1,FTM1_QDPHA_PIN,FTM1_QDPHB_PIN); //FTM1 正交解码初始化//所用的管脚可查 port_cfg.h 的 FTM1_QDPHA_PIN 和 FTM1_QDPHB_PIN
    ftm_quad_init(FTM2,FTM_PS_1,FTM2_QDPHA_PIN,FTM2_QDPHB_PIN);
    pit_init_ms(PIT0, 10);                                 //初始化PIT0，定时时间为： 500ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断

    led_init(LED0);

    ftm_pwm_init(FTM3, FTM_CH4,10*1000,0,FTM3_CH4_PIN);        //初始化 FTM PWM ，使用 FTM0_CH3，频率为200k ，占空比为 30 %
    ftm_pwm_init(FTM3, FTM_CH5,10*1000,0,FTM3_CH5_PIN); // app/inc/port_cfg.h 里 配置 FTM0_CH3_PIN 对应为 PTA6

    ftm_pwm_init(FTM3, FTM_CH6,10*1000,0,FTM3_CH6_PIN);        //初始化 FTM PWM ，使用 FTM0_CH3，频率为200k ，占空比为 30 %
    ftm_pwm_init(FTM3, FTM_CH7,10*1000,0,FTM3_CH7_PIN); // app/inc/port_cfg.h 里 配置 FTM0_CH3_PIN 对应为 PTA6

    while(1)
    {     
          led(LED0,0);
          ftm_pwm_duty(FTM3, FTM_CH4,0);     //设置占空比 为 30%
          ftm_pwm_duty(FTM3, FTM_CH6,pwm);     //设置占空比 为 30%//val

          ftm_pwm_duty(FTM3, FTM_CH5,pwm2);     //设置占空比 为 0%
          ftm_pwm_duty(FTM3, FTM_CH7,0);     //设置占空比 为 0%  //val2

          sprintf((char*)vbuf,"%6d",val);            //编码器脉冲值
          sprintf((char*)vbuf2,"%6d",val2);          // 编码器脉冲值 
          sprintf((char*)pwmbuf,"%6f",pwm);          // 编码器脉冲值 
          OLED_P6x8Str( 0, 0, "******text******");
          OLED_P6x8Str( 0, 2, vbuf);
	  OLED_P6x8Str( 0, 3, vbuf2);
          OLED_P6x8Str( 0, 5, pwmbuf);

          
    }
}
void PIT0_IRQHandler(void)
{
   
    val2 = ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    val = ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
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
    PIT_Flag_Clear(PIT0);       //清中断标志位
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
