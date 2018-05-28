/*
 * wmy_control_algorithm.c
 *
 *  Created on: 2018年5月24日
 *      Author: wmy
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "time.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "string.h"
#include "driverlib/timer.h"
#include "wmy_control_algorithm.h"

/**************************************************
                          PID清空内部寄存器
 *************************************************/
void PID_Init(SPID *pid)
{
    pid->P_TERM=0;
    pid->I_TERM=0;
    pid->D_TERM=0;
    pid->Integration_Sum=0;
    pid->LAST_Error=0;
    pid->NOW_Error=0;
    pid->Old_Position=0;
    pid->NOW_Out=0;
    pid->LAST_Out=0;
    pid->Out_Increment=0;
}

/**************************************************
                               PID参数配置
 *************************************************/
void PID_Configure(SPID *pid, double kp, double ki, double kd, double setpoint)
{
    pid->KP=kp;
    pid->KI=ki;
    pid->KD=kd;
    pid->SET_Point=setpoint;
    PID_Init(pid);
}

/**************************************************
                  位置式PID控制，开定时器使用
 *************************************************/
double PID_Position_Type_Control(SPID *pid, double position)//位置试PID
{
    pid->NOW_Error = pid->SET_Point - position;//求当前误差
    pid->Integration_Sum = pid->Integration_Sum + pid->NOW_Error;//积分项累加
    pid->LAST_Error = pid->SET_Point - pid->Old_Position;//求上次误差
    pid->Old_Position = position;
    pid->P_TERM = pid->KP * pid->NOW_Error;
    pid->I_TERM = pid->KI * pid->Integration_Sum;
    pid->D_TERM = pid->KD * ( pid->NOW_Error - pid->LAST_Error );
    return pid->P_TERM + pid->I_TERM + pid->D_TERM;
}

/**************************************************
                  增量式PID控制，开定时器使用
 *************************************************/
double PID_Increment_Type_Control(SPID *pid, double position)//增量试PID
{
    pid->LAST_Out = pid->NOW_Out;
    pid->NOW_Error = pid->SET_Point - position;//求当前误差
    pid->Integration_Sum = pid->Integration_Sum + pid->NOW_Error;//积分项累加
    pid->LAST_Error = pid->SET_Point - pid->Old_Position;//求上次误差
    pid->Old_Position = position;
    pid->P_TERM = pid->KP * pid->NOW_Error;
    pid->I_TERM = pid->KI * pid->Integration_Sum;
    pid->D_TERM = pid->KD * ( pid->NOW_Error - pid->LAST_Error );
    pid->NOW_Out = pid->P_TERM + pid->I_TERM + pid->D_TERM;
    pid->Out_Increment = pid->NOW_Out - pid->LAST_Out;
    return pid->NOW_Out + pid->Out_Increment;
}

/**************************************************
                          浮点数取绝对值函数
 *************************************************/
double Absolute_FPU(double x)//取绝对值
{
    if(x>0)
    {
        return x;
    }
    else if(x<0)
    {
        return (-x);
    }
    return 0;
}

/**************************************************
                          一阶低通滤波算法
 *************************************************/
void First_Order_Low_Pass_Filter(double  *value, double newvalue, double a)//a<1 a为新采样数据的比重值
{
    *value = *value * (1-a);
    *value = *value + newvalue * a;
}

/**
* Init_KalmanInfo   初始化滤波器的初始值
*  info  滤波器指针
*  Q 预测噪声方差 由系统外部测定给定
*  R 测量噪声方差 由系统外部测定给定
*/

/**************************************************
                          一维卡拉曼初始化
 *************************************************/
void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
{
    info->A = 1;  //标量卡尔曼
    info->H = 1;  //
    info->P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
    info->Q = Q;    //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
    info->R = R;    //测量（观测）噪声方差 可以通过实验手段获得
    info->filterValue = 0;// 测量的初始值
}

/**************************************************
                          一维卡拉曼滤波器
 *************************************************/
double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
{
    //预测下一时刻的值
    kalmanInfo->predictValue = kalmanInfo->A* kalmanInfo->filterValue;
    //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
    //求协方差
    kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;
    //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
    kalmanInfo->preValue = kalmanInfo->filterValue;
    //记录上次实际坐标的值
    //计算kalman增益
    kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);
    //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
    //修正结果，即计算滤波值
    kalmanInfo->filterValue = kalmanInfo->predictValue + (lastMeasurement - kalmanInfo->predictValue)*kalmanInfo->kalmanGain;
    //对于一维的情况，最优估计由下式给出：x[n|n]=x[n|n-1]+K[n]*{z[n]-x[n|n-1]}。其中z[n]为观测值
    //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
    //更新后验估计
    kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;
    //计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
    return  kalmanInfo->filterValue;
}
