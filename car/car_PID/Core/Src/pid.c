#include "pid.h"
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motor.h"

extern float distance;
extern TIM_HandleTypeDef htim2,htim4;
extern uint8_t Fore,Back,Left,Right;
//PID控制参数=-180=-0.9=0.0005
float Vertical_kp=-180,Vertical_kd=-0.9;//数量级 Kp:0~1000;Kd:0~10
float Velocity_kp=-0.8,Velocity_ki=-0.004;//数量级 Kp:0~1;Ki:Kp/200
float Turn_kp=10,Turn_kd=0.6;				//数量级 Kd:0~1
uint8_t stop;
//传感器数据变量
int Encoder_left,Encoder_right;
float pitch,roll,yaw;
short	gyrox,gyroy,gyroz;
short aacx,aacy,aacz;
//闭环控制器的中间变量
int Vertical_out,Velocity_out,Turn_out,Motor_L,Motor_R;
float Med_angle=0;//平衡角度值偏移量
int	Target_speed,Target_turn;//遥控设定

#define SPEED_Y 10  //俯仰（前后）最大速度
#define SPEED_Z 100 //偏航（左右）最大速度

/*直立闭环控制器（PD）

输入：期望角度、实际角度、角速度（倾角的微分）（MPU6050滤过波*/
int Vertical(float expectAngle,float actualAngle,float gyro_y)
{
	int temp;
	temp = Vertical_kp*(actualAngle-expectAngle)+Vertical_kd*gyro_y;
	return temp;
}

/*速度闭环控制器（PI比例积分）

输入：目标速度、左编码器、右编码器*/
int Velocity(float targetSpeed,float encoder_L,float encoder_R)
{	
	int Err,Err_lowOut,temp;
	static int Err_lowOut_last,Err_S;
	static float a=0.7;

	//#1.算偏差值
	Err = (encoder_L+encoder_R)-targetSpeed;
	//#2.低通滤波
	Err_lowOut = Err*(1-a)+a*Err_lowOut_last;
	Err_lowOut_last = Err_lowOut;
	//#3.积分
	Err_S += Err_lowOut;
	//#4.积分限幅(-20000~20000)
	Err_S  = (Err_S>20000)?20000:(Err_S<-20000)?-20000:Err_S;
	if(stop==1)Err_S=0,stop=0;
	//#5.速度环计算
	temp = Velocity_kp*Err_lowOut+Velocity_ki*Err_S;
	return temp;
}

/*转向环控制器（PD）-->保证直行
输入：角度值、角速度（倾角的微分）（MPU6050滤过波*/
int Turn(float Target_Turn,float gyro_z)
{
	int temp;
	temp = Turn_kp*Target_Turn+Turn_kd*gyro_z;//非标准公式
	return temp;	
}

//核心控制函数，10ms调用一次
void Control(void)
{
	int PWM_Out,Turn_out;
	//#1.读取传感器编码器和陀螺仪的值
	//1.1编码器
	Encoder_left = Read_Speed(&htim2);
	Encoder_right = -Read_Speed(&htim4);
	//1.2陀螺仪:角度，角速度，加速度
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	//#2.蓝牙控制
	//2.1前后
	if((Fore==0)&&(Back==0))
		Target_speed=0;
	if(Fore==1)
	{
		if(distance<50)
			Target_speed++;//超声波检测到有障碍，后退
		else
			Target_speed--;
	}
	if(Back==1)
		Target_speed++;
	Target_speed  = (Target_speed>SPEED_Y)?SPEED_Y:(Target_speed<-SPEED_Y)?-SPEED_Y:Target_speed;//限幅
	//2.2左右
	if((Left==0)&&(Right==0))
		Target_turn=0;
	if(Left==1)
		Target_turn+=30;
	if(Right==1)
		Target_turn-=30;
	Target_turn  = (Target_turn>SPEED_Z)?SPEED_Z:(Target_turn<-SPEED_Z)?-SPEED_Z:Target_turn;//限幅
	//2.3转向约束开启，关闭 （保证不转向时候直行稳定）  
	if((Left==0)&&(Right==0))
		Turn_kd=0.6;
	else if((Left==1)||(Right==1))
		Turn_kd=0;	
	
	//#3.将数据传入PID控制器，计算出结果（左右电机转速值）
	//3.1速度控制器输出
	Velocity_out = Velocity(Target_speed,Encoder_left,Encoder_right);
	//3.2直立控制器输出
	Vertical_out = Vertical(Velocity_out+Med_angle,roll,gyrox);//roll
	//3.3转向控制器输出
	Turn_out = Turn(Target_turn,gyroz);
	//3.4小车电机差速转向（有turnout就转向，没有就直行）
	PWM_Out = Vertical_out;
	Motor_L = PWM_Out-Turn_out;
	Motor_R = PWM_Out+Turn_out;
	Moter_Limit(&Motor_L,&Motor_R);//限速
	Load(Motor_L,Motor_R);
	Stop(&Med_angle,&roll);
}
