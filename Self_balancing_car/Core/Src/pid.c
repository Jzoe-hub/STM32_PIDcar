#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

uint8_t Stop;
extern float distance;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern uint8_t fore,back,left,right;
//PID参数
float Vertical_Kp=-180,Vertical_Kd=-0.9;//数量级 Kp:0~1000;Kd:0~10
float Velocity_Kp=-0.8,Velocity_Ki=-0.004;//数量级 Kp:0~1;Ki:Kp/200
float Turn_Kp=10,Turn_Kd=0.6;//数量级 Kd:0~1
//读取的传感器变量
int Encoder_L,Encoder_R;   //编码器本身是整数
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short ax,ay,az;
//各个环输出以及输入
float Target_Speed,Target_Turn;//遥控设定
float	Med_jiaodu;//平衡角度偏移量
int MotorL,MotorR,Vertical_Out,Velocity_Out,Turn_Out;

#define SPEED_Y 10 //俯仰限幅
#define SPEED_Z 100//转向限幅

/*直立闭环控制器（PD）
输入：期望角度、实际角度、角速度（倾角的微分）（MPU6050滤过波*/
int Vertical(float except_Angle,float actual_Angle,float gyro_x)
{
	int temp;
	temp = Vertical_Kp*(actual_Angle-except_Angle)+Vertical_Kd*gyro_x;
	return temp;
}

/*速度闭环控制器（PI比例积分）
输入：目标速度、左编码器、右编码器*/
int Velocity(float target_Speed,float encoderL,float encoderR)
{
	int temp,Err,Err_filtered;
	float a=0.7;
	static int Err_filtered_last,Err_filtered_S;
	//计算偏差
	Err = (encoderL+encoderR)-target_Speed;
	//低通滤波
	Err_filtered = Err*(1-a)+Err_filtered_last*a;
	Err_filtered_last = Err_filtered;//更新
	//积分（离散值相当于累加）
	Err_filtered_S += Err_filtered;
	//积分限幅
	Err_filtered_S = (Err_filtered_S>20000)?20000:(Err_filtered_S<-20000)?-20000:Err_filtered_S;
	
	//加入翻车保护
	if(Stop==1)
	{
		Err_filtered_S=0;
		Stop=0;//stop只是个瞬间倒地的标志，如果一直是1会造成偏差值一直0，速度环失效
	}
	//计算速度环输出
	temp = Velocity_Kp*Err_filtered+Velocity_Ki*Err_filtered_S;
	return temp;
}

/*转向环控制器（PD）   -->控制转弯 保证直行
输入：角度值、角速度（倾角的微分）（dmp滤过波*/
int Turn(float Target_turn,float gyro_z)
{
	int temp;
	temp = Turn_Kp*Target_turn+Turn_Kd*gyro_z;
	return temp;
}

/*核心控制函数 10ms调用一次*/
void Control(void)
{
	int PWM_Out;
	//#1.读取传感器编码器和陀螺仪的值
	//1.1编码器
	Encoder_L = Encoder_ReadSpeed(&htim2);
	Encoder_R = -Encoder_ReadSpeed(&htim4);
	//1.2陀螺仪
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&ax,&ay,&az);
	
	//#2.蓝牙遥控
	//2.1前后
	if((fore==0)&&(back==0))
		Target_Speed=0;
	if(fore==1)
	{
		if(distance<30)
			Target_Speed++;//超声波检测到有障碍，后退
		else
			Target_Speed--;
	}
	if(back==1)
		Target_Speed++;
	Target_Speed = (Target_Speed>SPEED_Y)?SPEED_Y:(Target_Speed<-SPEED_Y)?-SPEED_Y:Target_Speed;
	//2.2左右
	if((left==0)&&(right==0))
		Target_Turn=0;
	if(left==1)
		Target_Turn+=30;
	if(right==1)
		Target_Turn-=30;
	Target_Turn = (Target_Turn>SPEED_Z)?SPEED_Z:(Target_Turn<-SPEED_Z)?-SPEED_Z:Target_Turn;
	//2.3转向约束
	if((left==0)&&(right==0))
		Turn_Kd=0.6;
	else if((left==1)||(right==1))
		Turn_Kd=0;
	
	//#3.将数据传入PID控制器，计算出结果（左右电机转速值）
	//3.1速度控制器输出
	Velocity_Out = Velocity(Target_Speed,Encoder_L,Encoder_R);
	//3.2直立控制器输出
	Vertical_Out = Vertical(Velocity_Out+Med_jiaodu,roll,gyrox);
	//3.3转向控制器输出
	Turn_Out = Turn(Target_Turn,gyroz);
	//3.4输出到电机（有turnout就差速转向，没有就直行）
	PWM_Out = Vertical_Out;
	MotorL = PWM_Out-Turn_Out;
	MotorR = PWM_Out+Turn_Out;
	Motor_Limit(&MotorL,&MotorR);
	Motor_Set(MotorL,MotorR);
	Motor_Stop(&Med_jiaodu,&roll);
}
