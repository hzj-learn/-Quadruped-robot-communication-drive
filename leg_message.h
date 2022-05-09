#ifndef _leg_message
#define _leg_message

#include <stdint.h>

// 60 bytes
// 30 16-bit words
//备注：SPI的位置、速度、标志位、校验位数据（实际值）
struct spi_data_t
{
    float q_abad[2];			//胯关节的位置的SPI数据，两个变量
    float q_hip[2];				//髋关节的位置的SPI数据
    float q_knee[2];			//膝关节的位置的SPI数据
	
    float qd_abad[2];			//胯关节的速度的SPI数据
    float qd_hip[2];			//髋关节的速度的SPI数据
    float qd_knee[2];			//膝关节的速度的SPI数据
	
    int32_t flags[2];
    int32_t checksum;
};

// 132 bytes
// 66 16-bit words
//备注：（1）因为这套程序时适用于两个单片机的，所以同一个类型的变量要有两个
//			（2）SPI的位置、数据、力矩增益、标志位校验位的命令（期望值）
struct spi_command_t      
{
    float q_des_abad[2];  //胯关节的位置信息命令
    float q_des_hip[2];		//髋关节的位置信息命令
    float q_des_knee[2];	//膝关节的位置信息命令
	
    float qd_des_abad[2]; //胯关节的速度信息命令
    float qd_des_hip[2];	//髋关节的速度信息命令
    float qd_des_knee[2];	//膝关节的速度信息命令
	
    float kp_abad[2];     //胯关节的KP增益信息命令
    float kp_hip[2];			//髋关节的KP增益信息命令
    float kp_knee[2];			//膝关节的KP增益信息命令
	
    float kd_abad[2];     //胯关节的KP增益信息命令
    float kd_hip[2];			//髋关节的KP增益信息命令
    float kd_knee[2];			//膝关节的KP增益信息命令
	
    float tau_abad_ff[2]; //跨关节的前馈扭矩增益命令
    float tau_hip_ff[2];	//髋关节的前馈扭矩增益命令
    float tau_knee_ff[2]; //膝关节的前馈扭矩增益命令
    int32_t flags[2];			//SPI切换两块单片机CAN用的
    int32_t checksum;
};


//关节电机发送的五个命令的结构体：
//(1)期望位置p_des
//(2)期望速度v_des
//(3)KP
//(4)KD
//(5)关节力矩t_ff
struct joint_control
	{
   float p_des, v_des, kp, kd, t_ff;
	};


//关节状态的结构体
//（1）关节位置p
//（2）关节速度v
//（3）关节力矩t
struct joint_state
	{
   float p, v, t;
  };

	
	
//腿的状态的结构体
//（1）
//（2）	
//（3）	
struct leg_state
	{
   joint_state a, h, k;
  };
	
	
	
//关节控制的结构体
//（1）	
//（2）	
//（3）	
struct leg_control
	{
   joint_control a, h, k;
  }
    ;
#endif