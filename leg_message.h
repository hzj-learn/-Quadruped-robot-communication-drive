#ifndef _leg_message
#define _leg_message

#include <stdint.h>

// 60 bytes
// 30 16-bit words
//��ע��SPI��λ�á��ٶȡ���־λ��У��λ���ݣ�ʵ��ֵ��
struct spi_data_t
{
    float q_abad[2];			//��ؽڵ�λ�õ�SPI���ݣ���������
    float q_hip[2];				//�Źؽڵ�λ�õ�SPI����
    float q_knee[2];			//ϥ�ؽڵ�λ�õ�SPI����
	
    float qd_abad[2];			//��ؽڵ��ٶȵ�SPI����
    float qd_hip[2];			//�Źؽڵ��ٶȵ�SPI����
    float qd_knee[2];			//ϥ�ؽڵ��ٶȵ�SPI����
	
    int32_t flags[2];
    int32_t checksum;
};

// 132 bytes
// 66 16-bit words
//��ע����1����Ϊ���׳���ʱ������������Ƭ���ģ�����ͬһ�����͵ı���Ҫ������
//			��2��SPI��λ�á����ݡ��������桢��־λУ��λ���������ֵ��
struct spi_command_t      
{
    float q_des_abad[2];  //��ؽڵ�λ����Ϣ����
    float q_des_hip[2];		//�Źؽڵ�λ����Ϣ����
    float q_des_knee[2];	//ϥ�ؽڵ�λ����Ϣ����
	
    float qd_des_abad[2]; //��ؽڵ��ٶ���Ϣ����
    float qd_des_hip[2];	//�Źؽڵ��ٶ���Ϣ����
    float qd_des_knee[2];	//ϥ�ؽڵ��ٶ���Ϣ����
	
    float kp_abad[2];     //��ؽڵ�KP������Ϣ����
    float kp_hip[2];			//�Źؽڵ�KP������Ϣ����
    float kp_knee[2];			//ϥ�ؽڵ�KP������Ϣ����
	
    float kd_abad[2];     //��ؽڵ�KP������Ϣ����
    float kd_hip[2];			//�Źؽڵ�KP������Ϣ����
    float kd_knee[2];			//ϥ�ؽڵ�KP������Ϣ����
	
    float tau_abad_ff[2]; //��ؽڵ�ǰ��Ť����������
    float tau_hip_ff[2];	//�Źؽڵ�ǰ��Ť����������
    float tau_knee_ff[2]; //ϥ�ؽڵ�ǰ��Ť����������
    int32_t flags[2];			//SPI�л����鵥Ƭ��CAN�õ�
    int32_t checksum;
};


//�ؽڵ�����͵��������Ľṹ�壺
//(1)����λ��p_des
//(2)�����ٶ�v_des
//(3)KP
//(4)KD
//(5)�ؽ�����t_ff
struct joint_control
	{
   float p_des, v_des, kp, kd, t_ff;
	};


//�ؽ�״̬�Ľṹ��
//��1���ؽ�λ��p
//��2���ؽ��ٶ�v
//��3���ؽ�����t
struct joint_state
	{
   float p, v, t;
  };

	
	
//�ȵ�״̬�Ľṹ��
//��1��
//��2��	
//��3��	
struct leg_state
	{
   joint_state a, h, k;
  };
	
	
	
//�ؽڿ��ƵĽṹ��
//��1��	
//��2��	
//��3��	
struct leg_control
	{
   joint_control a, h, k;
  }
    ;
#endif