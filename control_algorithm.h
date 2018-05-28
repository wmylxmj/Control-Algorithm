/*
 * wmy_control_algorithm.h
 *
 *  Created on: 2018年5月24日
 *      Author: wmy
 */

#ifndef CONTROL_WMY_CONTROL_ALGORITHM_H_
#define CONTROL_WMY_CONTROL_ALGORITHM_H_

typedef struct
{
    double KP;//比例系数
    double KI;//积分系数
    double KD;//微分系数
    double SET_Point;//设定值
    double Old_Position;//上一次的位置
    double Integration_Sum;//误差积分项
    double P_TERM;//比例控制的返回值
    double I_TERM;//积分控制的返回值
    double D_TERM;//微分控制的返回值
    double NOW_Error;//当前误差
    double LAST_Error;//上一次误差
    double NOW_Out;
    double LAST_Out;
    double Out_Increment;
} SPID;

// 一维滤波器信息结构体
typedef  struct{
    double filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
    double kalmanGain;   //   Kalman增益
    double predictValue;    //预测下一时刻的值
    double preValue;    //记录上次实际坐标的值
    double A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
    double H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
    double Q;   //预测过程噪声偏差的方差
    double R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    double P;   //估计误差协方差
}  KalmanInfo;

extern void PID_Init(SPID *pid);
extern void PID_Configure(SPID *pid, double kp, double ki, double kd, double setpoint);
extern double PID_Position_Type_Control(SPID *pid, double position);//位置试PID
extern double PID_Increment_Type_Control(SPID *pid, double position);//增量试PID
extern double Absolute_FPU(double x);//取绝对值
extern void First_Order_Low_Pass_Filter(double  *value, double newvalue, double a);//a<1 a为新采样数据的比重值
extern void Init_KalmanInfo(KalmanInfo* info, double Q, double R);
extern double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);

#endif /* CONTROL_WMY_CONTROL_ALGORITHM_H_ */
