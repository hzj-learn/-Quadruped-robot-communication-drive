#include "mbed.h"
#include "math_ops.h"
#include <cstring>
#include "leg_message.h"

//����/���ͻ������ĳ���
#define RX_LEN 66
#define TX_LEN 66

//����/������Ϣ�ĳ���
#define DATA_LEN 30
#define CMD_LEN  66

//������CAN ID
#define CAN_ID 0x0

/// ��ֵ���� ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
 
 /// �ؽ���ֹͣ ///
#define A_LIM_P 1.5f
#define A_LIM_N -1.5f
#define H_LIM_P 5.0f
#define H_LIM_N -5.0f
#define K_LIM_P 0.2f
#define K_LIM_N 7.7f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f;

#define ENABLE_CMD 0xFFFF
#define DISABLE_CMD 0x1F1F

spi_data_t spi_data;       //��ͨѶ�嵽upboard������ 
spi_command_t spi_command; //��upboard��ͨѶ�������

// spi ������
uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

DigitalOut led(PC_5);

Serial       pc(PA_2, PA_3);
CAN          can1(PB_12, PB_13);  // CAN Rx���Ŷ���, CAN Tx���Ŷ���
CAN          can2(PB_8, PB_9); 	  // CAN Rx���Ŷ���, CAN Tx���Ŷ���

CANMessage   rxMsg1, rxMsg2;
CANMessage   txMsg1, txMsg2;
CANMessage   a1_can, a2_can, h1_can, h2_can, k1_can, k2_can;    //TX Messages
int                     ledState;
Ticker                  sendCAN;
int                     counter = 0;
volatile bool           msgAvailable = false;
Ticker loop;

int spi_enabled = 0;
InterruptIn cs(PA_4);
DigitalIn estop(PA_14);
//SPISlave spi(PA_7, PA_6, PA_5, PA_4);

leg_state l1_state, l2_state;;
leg_control l1_control, l2_control;

uint16_t x = 0;
uint16_t x2 = 0;
uint16_t count = 0;
uint16_t counter2 = 0;

int control_mode = 1;
int is_standing = 0;
int enabled = 0;

void test_control();//��SPI��������αSPI���� 
void control();

/// CAN ������ṹ ///
/// 16 bit λ������ 	��λ��Χ�� -4*pi and 4*pi
/// 12 bit �ٶ����� 	��λ��Χ�� -30 and + 30 rad/s
/// 12 bit kp 				��λ��Χ�� 0 and 500 N-m/rad
/// 12 bit kd 				��λ��Χ��0 and 100 N-m*s/rad
/// 12 bit ��������		��λ��Χ�� -18 and 18 N-m
/// CAN �������8���ֽڵģ���ʽ���£������κ�Ƶ�ʣ�
/// bit 0 �� LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
/// CAN ������ṹ������MCU�����õ�///
void pack_cmd(CANMessage * msg, joint_control joint)
{
     
     /// �����������ڷ�Χ�� ///
     float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);                    
     float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
     float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
     float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
     float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
     /// ��������ת��λ�޷������� ///
     uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     ///��intsװ��can������  ///
     msg->data[0] = p_int>>8;                                       
     msg->data[1] = p_int&0xFF;
     msg->data[2] = v_int>>4;
     msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     msg->data[4] = kp_int&0xFF;
     msg->data[5] = kd_int>>4;
     msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     msg->data[7] = t_int&0xff;
     }
/////////////////////////////////////////////////////////////////


	 
/// CANӦ����ṹ ///
/// 16 bitλ������ 			��λ��Χ�� -4*pi and 4*pi
/// 12 bit�ٶ����� 			��λ��Χ�� -30 and + 30 rad/s
/// 12 bit��������      ��λ��Χ�� -40A and 40A;
/// CAN ���ݰ�ʱ58���ֽڣ���ʽ���£������κ�Ƶ�ʣ�
/// bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]	 
/// CANӦ����ṹ ,Ӧ�����ź��õ� ///	
//���ܣ���can�յ������λ�á��ٶȡ�������Ϣ��λ��ѹ����������������ת���ɸ����ͣ�������ID�Ű�������Ϣ���з���
//��ע�����͵�ʱ��ת��������ʽ��Ϊ��������С
void unpack_reply(CANMessage msg, leg_state * leg)
{
	/// ��CAN��������ѹints ///
	uint16_t id = msg.data[0];
	uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
	uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
	uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
	/// ��uint�����ַ����Σ� ת����float�������ͣ�����///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
	
	if(id==1)					//ID1�������ؽڣ�
		{
			leg->a.p = p;
			leg->a.v = v;
			leg->a.t = t;
		}
	else if(id==2)	  //ID2��������ȹؽڣ�
		{
			leg->h.p = p;
			leg->h.v = v;
			leg->h.t = t;
		}
	else if(id==3)		//ID3�����С�ȹؽڣ�
		{
			leg->k.p = p;
			leg->k.v = v;
			leg->k.t = t;
		}
	} 
/////////////////////////////////////////////////////////////////
	
	
	
/*CAN1�����жϽ��շ�����*/
//���ܣ�	��1����CAN���ݶ�дRX��Ϣ������
//				��2��Ӧ��������ʾ�յ���Ϣ
//û���õ���������main����ѭ������ʵ����
void rxISR1() 
{
	can1.read(rxMsg1);                    //��CAN���ݶ�дRX��Ϣ������
	unpack_reply(rxMsg1, &l1_state);
}
/////////////////////////////////////////////////////////////////
	
	
/*CAN2�����жϽ��շ�����*/
//���ܣ�	��1����CAN���ݶ�дRX��Ϣ������
//				��2��Ӧ��������ʾ�յ���Ϣ
//û���õ���������main����ѭ������ʵ����
void rxISR2()
{
	can2.read(rxMsg2);									//��CAN���ݶ�дRX��Ϣ������
	unpack_reply(rxMsg2, &l2_state);
}
/////////////////////////////////////////////////////////////////
	
	
	
/*ѹ������CAN����*/
void PackAll()
{
	//��ѹ��ؽڵ��can�ź�
	pack_cmd(&a1_can, l1_control.a); 
	pack_cmd(&a2_can, l2_control.a); 
	//��ѹ���ȹؽڵ��can�ź�
	pack_cmd(&h1_can, l1_control.h); 
	pack_cmd(&h2_can, l2_control.h); 
	//��ѹС�ȹؽڵ��can�ź�
	pack_cmd(&k1_can, l1_control.k); 
	pack_cmd(&k2_can, l2_control.k); 
}
/////////////////////////////////////////////////////////////////


	
/*����·CAN����д���Ӧ�Ļ�������CAN�շ������Զ����͵�*/
void WriteAll()
{
	//toggle = 1;
	can1.write(a1_can);
	wait(.00002);
	can2.write(a2_can);
	wait(.00002);
	can1.write(h1_can);
	wait(.00002);
	can2.write(h2_can);
	wait(.00002);
	can1.write(k1_can);
	wait(.00002);
	can2.write(k2_can);
	wait(.00002);
	//toggle = 0;
}
/////////////////////////////////////////////////////////////////
	
	
	
/*���������*/
//���ܣ�1����ѹ����CAN����
//		��2������·CAN����д���Ӧ�Ļ�������CAN�շ������Զ����͵�
void sendCMD()
{
	counter ++;
	PackAll();
	if(counter>100)
		{
			printf("%.3f %.3f %.3f   %.3f %.3f %.3f\n\r", l1_state.a.p, l1_state.h.p, l1_state.k.p, l2_state.a.p, l2_state.h.p, l2_state.k.p);
			counter = 0 ;
		}
	WriteAll();   
}
/////////////////////////////////////////////////////////////////


/*��can��������*/   
void Zero(CANMessage * msg)
{
	msg->data[0] = 0xFF;
	msg->data[1] = 0xFF;
	msg->data[2] = 0xFF;
	msg->data[3] = 0xFF;
	msg->data[4] = 0xFF;
	msg->data[5] = 0xFF;
	msg->data[6] = 0xFF;
	msg->data[7] = 0xFE;
	WriteAll();
}
/////////////////////////////////////////////////////////////////
	
	
	
/*����������ģʽ*/	
//��ע�������can�����Ƕ�Ӧ����can������ָ���	
void EnterMotorMode(CANMessage * msg)
{
	msg->data[0] = 0xFF;
	msg->data[1] = 0xFF;
	msg->data[2] = 0xFF;
	msg->data[3] = 0xFF;
	msg->data[4] = 0xFF;
	msg->data[5] = 0xFF;
	msg->data[6] = 0xFF;
	msg->data[7] = 0xFC;
	//WriteAll();
}
/////////////////////////////////////////////////////////////////


/*�˳��������ģʽ*/	
//��ע�������can�����Ƕ�Ӧ����can������ָ���
void ExitMotorMode(CANMessage * msg)
{
	msg->data[0] = 0xFF;
	msg->data[1] = 0xFF;
	msg->data[2] = 0xFF;
	msg->data[3] = 0xFF;
	msg->data[4] = 0xFF;
	msg->data[5] = 0xFF;
	msg->data[6] = 0xFF;
	msg->data[7] = 0xFD;
	//WriteAll();
}
/////////////////////////////////////////////////////////////////
	
	
/// ���ڴ������ն˼������� ///	
void serial_isr()
{
	 while(pc.readable()){
			char c = pc.getc();
			//led = !led;
			switch(c){
					case(27):
							//loop.detach();
							printf("\n\r exiting motor mode \n\r");
							ExitMotorMode(&a1_can);
							ExitMotorMode(&a2_can);
							ExitMotorMode(&h1_can);
							ExitMotorMode(&h2_can);
							ExitMotorMode(&k1_can);
							ExitMotorMode(&k2_can);
							enabled = 0;
							break;
					case('m'):
							printf("\n\r entering motor mode \n\r");
							EnterMotorMode(&a1_can);
							EnterMotorMode(&a2_can);
							EnterMotorMode(&h1_can);
							EnterMotorMode(&h2_can);
							EnterMotorMode(&k1_can);
							EnterMotorMode(&k2_can);
							wait(.5);
							enabled = 1;
							//loop.attach(&sendCMD, .001);
							break;
					case('s'):
							printf("\n\r standing \n\r");
							counter2 = 0;
							is_standing = 1;
							//stand();
							break;
					case('z'):
							printf("\n\r zeroing \n\r");
							Zero(&a1_can);
							Zero(&a2_can);
							Zero(&h1_can);
							Zero(&h2_can);
							Zero(&k1_can);
							Zero(&k2_can);
							break;
					}
			}
			WriteAll();
	}
/////////////////////////////////////////////////////////////////

		

//SPI���У���		
uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;
}
/////////////////////////////////////////////////////////////////



/*SPI�жϷ�����*/
//�������� UPBORD������������ȡ�󣬱����ȥSPI������ 
//ͬʱ���͵ײ�ؽ����ݸ�upboard
void spi_isr(void)
{
	GPIOC->ODR |= (1 << 8);
	GPIOC->ODR &= ~(1 << 8);
	int bytecount = 0;
	SPI1->DR = tx_buff[0];
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	while(cs == 0) 													//upboardʹ��cs���ź�SPI���ݽ��н���
		{
		if(SPI1->SR&0x1) 
			{
				rx_buff[bytecount] = SPI1->DR;    //�������� UPBORD�����������
				bytecount++;
				if(bytecount<TX_LEN) 
					{
						SPI1->DR = tx_buff[bytecount];//��׼���ϱ��Ĺؽ������ϴ���UPBORD ���ݽ���
					}
			}
		} 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
	


  /////////////////////////////////////////////////////////////////////////////////////////////////////////		
    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32); //������������Ӧ���ȼ��У��ͣ� 
    for(int i = 0; i < CMD_LEN; i++)			 //��ȡ�󣬱����ȥSPI������ 
    {
     ((uint16_t*)(&spi_command))[i] = rx_buff[i];
    }   
    //���пؼ���Ϊ��һ�ε������tx_buff
    if(calc_checksum != spi_command.checksum)
			{
        spi_data.flags[1] = 0xdead;
			}     
    //test_control();
    //spi_data.q_abad[0] = 12.0f;
    control();
    PackAll();		//��UPBORD ���������Ϣ��װ��ÿ������
		WriteAll();		//����CAN ���͸�ÿ���ؽ�
}
/////////////////////////////////////////////////////////////////


/*�ؽ�λ���޷�*/
int softstop_joint(joint_state state, joint_control * control, float limit_p, float limit_n)
{
    if((state.p)>=limit_p)//�ؽ�λ�ù����޷�
			{
        //control->p_des = limit_p;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP*(limit_p - state.p);
        return 1;
      }
    else if((state.p)<=limit_n)//�ؽ�λ�ù�С�޷�
			{
        //control->p_des = limit_n;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP*(limit_n - state.p);
        return 1;
      }
    return 0;
}
/////////////////////////////////////////////////////////////////

		
/*����SPI/CAN˫�����ݽ���������*/
//˫��ģʽ
//ֹͣģʽ
void control()
{  
///////////////////////////////////���ݵ��״̬enabled��upboard������spi_command.flags�����Ƿ������ģʽ////////////////////////////////////////////
	if(((spi_command.flags[0]&0x1)==1)  && (enabled==0))						//����CAN�ĵ��ģʽ�������������������enabled=0����upboard�ģ�spi_command.flags[0]=0
		{
			enabled = 1;
			EnterMotorMode(&a1_can);
			can1.write(a1_can);
			EnterMotorMode(&a2_can);
			can2.write(a2_can);
			EnterMotorMode(&k1_can);
			can1.write(k1_can);
			EnterMotorMode(&k2_can);
			can2.write(k2_can);
			EnterMotorMode(&h1_can);
			can1.write(h1_can);
			EnterMotorMode(&h2_can);
			can2.write(h2_can);
			printf("e\n\r");
			return;
		}
		else if((((spi_command.flags[0]&0x1))==0)  && (enabled==1))	  //�˳�CAN�ĵ��ģʽ��������enabled=1��spi_command.flags[0]=1
		{
			enabled = 0;
			ExitMotorMode(&a1_can);
			can1.write(a1_can);
			ExitMotorMode(&a2_can);
			can2.write(a2_can);
			ExitMotorMode(&h1_can);
			can1.write(h1_can);
			ExitMotorMode(&h2_can);
			can2.write(h2_can);
			ExitMotorMode(&k1_can);
			can1.write(k1_can);
			ExitMotorMode(&k2_can);
			can2.write(k2_can);
			printf("x\n\r");
			return;
		 }
		 
///////////////////////////��CAN �ж� ÿ���ؽڻ�ȡ����Ϣ ȡ����׼������SPI ��Ӧ��UPBORAD /////////////////////////////////
		//��MCU1���can������λ����Ϣת�浽SPI���ݻ�����
		spi_data.q_abad[0] = l1_state.a.p;
		spi_data.q_hip[0] = l1_state.h.p;
		spi_data.q_knee[0] = l1_state.k.p;
		//��MCU1���can�������ٶ���Ϣת�浽SPI���ݻ�����	
		spi_data.qd_abad[0] = l1_state.a.v;
		spi_data.qd_hip[0] = l1_state.h.v;
		spi_data.qd_knee[0] = l1_state.k.v; 
		//��MCU2���can������λ����Ϣת�浽SPI���ݻ�����
		spi_data.q_abad[1] = l2_state.a.p;
		spi_data.q_hip[1] = l2_state.h.p;
		spi_data.q_knee[1] = l2_state.k.p;
		//��MCU2���can�������ٶ���Ϣת�浽SPI���ݻ�����			
		spi_data.qd_abad[1] = l2_state.a.v;
		spi_data.qd_hip[1] = l2_state.h.v;
		spi_data.qd_knee[1] = l2_state.k.v;
		
		if(estop==0)	//ֹͣ����ͨ��ģʽ
			{
				//printf("estopped!!!!\n\r");
				memset(&l1_control, 0, sizeof(l1_control));//���ڴ棬��С��l1_controlһ������0�������
				memset(&l2_control, 0, sizeof(l2_control));
				spi_data.flags[0] = 0xdead;
				spi_data.flags[1] = 0xdead;
				led = 1;
			}
	

			
		else			//��upboard���������������CAN���ƣ�ͬʱ�ѵ��CAN�����ݷ��͸�upboard,˫��ͨѶģʽ
			{
				led = 0; 
				memset(&l1_control, 0, sizeof(l1_control));//���ڴ棬��С��l1_controlһ������0�������
				memset(&l2_control, 0, sizeof(l2_control));
				/*CAN1*/
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN1��ؽڵ��������
				l1_control.a.p_des = spi_command.q_des_abad[0];
				l1_control.a.v_des  = spi_command.qd_des_abad[0];
				l1_control.a.kp = spi_command.kp_abad[0];
				l1_control.a.kd = spi_command.kd_abad[0];
				l1_control.a.t_ff = spi_command.tau_abad_ff[0];
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN1�Źؽڵ��������
				l1_control.h.p_des = spi_command.q_des_hip[0];
				l1_control.h.v_des  = spi_command.qd_des_hip[0];
				l1_control.h.kp = spi_command.kp_hip[0];
				l1_control.h.kd = spi_command.kd_hip[0];
				l1_control.h.t_ff = spi_command.tau_hip_ff[0];
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN1ϥ�ؽڵ��������
				l1_control.k.p_des = spi_command.q_des_knee[0];
				l1_control.k.v_des  = spi_command.qd_des_knee[0];
				l1_control.k.kp = spi_command.kp_knee[0];
				l1_control.k.kd = spi_command.kd_knee[0];
				l1_control.k.t_ff = spi_command.tau_knee_ff[0];
				
				
				/*CAN2*/
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN2��ؽڵ��������
				l2_control.a.p_des = spi_command.q_des_abad[1];
				l2_control.a.v_des  = spi_command.qd_des_abad[1];
				l2_control.a.kp = spi_command.kp_abad[1];
				l2_control.a.kd = spi_command.kd_abad[1];
				l2_control.a.t_ff = spi_command.tau_abad_ff[1];
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN2�Źؽڵ��������
				l2_control.h.p_des = spi_command.q_des_hip[1];
				l2_control.h.v_des  = spi_command.qd_des_hip[1];
				l2_control.h.kp = spi_command.kp_hip[1];
				l2_control.h.kd = spi_command.kd_hip[1];
				l2_control.h.t_ff = spi_command.tau_hip_ff[1];
				//��upboard SPI���͹����ĵ�ָ�����MCU CAN2ϥ�ؽڵ��������
				l2_control.k.p_des = spi_command.q_des_knee[1];
				l2_control.k.v_des  = spi_command.qd_des_knee[1];
				l2_control.k.kp = spi_command.kp_knee[1];
				l2_control.k.kd = spi_command.kd_knee[1];
				l2_control.k.t_ff = spi_command.tau_knee_ff[1];	//�õ�����UPBORAD �ϵ����ȡ��׼������CAN�·�����Ӧ�Ĺؽ�
					
				spi_data.flags[0] = 0;    //��λ
				spi_data.flags[1] = 0;    //��λ
				spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
				spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N))<<1;
			
				spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
				spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N))<<1;
			}
			spi_data.checksum = xor_checksum((uint32_t*)&spi_data,14);
			for(int i = 0; i < DATA_LEN; i++)
			{
			tx_buff[i] = ((uint16_t*)(&spi_data))[i]; 			//��CAN ��ȡ�Ĺؽ���Ϣ ׼����SPI ���ͻ�����	
			}     
}
/////////////////////////////////////////////////////////////////    


//SPI�������ݸ�upboard���Ժ���
//��ע��SPI�����´�upboard�Ͻ��յģ����պ����SPI��������
void test_control()
{
	for(int i = 0; i < 2; i++)//ִ�����Σ���Ϊ�ĸ���������Ƭ����ֵ��0����MCU1,1����MCU2
	{
		//λ����
		spi_data.q_abad[i] = spi_command.q_des_abad[i] + 1.f;			//�ѿ�ؽڵ�λ������+1�󣬸�ֵ��SPI����
		spi_data.q_knee[i] = spi_command.q_des_knee[i] + 1.f;			//���Źؽڵ�λ������+1�󣬸�ֵ��SPI����
		spi_data.q_hip[i]  = spi_command.q_des_hip[i]  + 1.f;			//��ϥ�ؽڵ�λ������+1�󣬸�ֵ��SPI����
		//�ٶ���
		spi_data.qd_abad[i] = spi_command.qd_des_abad[i] + 1.f;		//�ѿ�ؽڵ��ٶ�����+1�󣬸�ֵ��SPI����
		spi_data.qd_knee[i] = spi_command.qd_des_knee[i] + 1.f;		//���Źؽڵ��ٶ�����+1�󣬸�ֵ��SPI����
		spi_data.qd_hip[i]  = spi_command.qd_des_hip[i]  + 1.f;		//��ϥ�ؽڵ��ٶ�����+1�󣬸�ֵ��SPI����
	}  
	spi_data.flags[0] = 0xdead;
	//ֻ����Ϣ��ǰ56���ֽڿ�ʼ 
	spi_data.checksum = xor_checksum((uint32_t*)&spi_data,14);	//SPI���У���
	
	for(int i = 0; i < DATA_LEN; i++)														//��������µ�SPI���ݷ���SSPI���ͻ���������
	{
		tx_buff[i] = ((uint16_t*)(&spi_data))[i];
	}       
}



/////////////////////////////////////////////////////////////////


/*
*��ʼ��SPI���ţ�����SPIƵ��Ϊ12M
*/
void init_spi(void)
{
	SPISlave *spi = new SPISlave(PA_7, PA_6, PA_5, PA_4);		//��ʼ��SPI����
	spi->format(16, 0);
	spi->frequency(12000000);																//����SPIƵ��Ϊ12M
	spi->reply(0x0);
	cs.fall(&spi_isr);
	printf("done\n\r");
}
/////////////////////////////////////////////////////////////////


int main() 
{
	///////////////////////////////////////////*Ӳ����ʼ��*/////////////////////////////////////////
	/*��1����ʼ������*/
	pc.baud(921600);																			//���ô��ڵĲ����ʣ����̣�
	pc.attach(&serial_isr); 															//���Ӵ����жϷ����������̣�
	
	/*��2����ʼ��ͨѶģʽ*/
	estop.mode(PullUp);																		//����ͨѶģʽ����������ֹͣCAN/SPIͨѶ��������������CAN/SPI˫��ͨѶ
	
	/*��3����ʼ��CANͨѶ*/
	can1.frequency(1000000);      												//����CAN1ͨѶ������Ϊ1Mbps
	//can1.attach(&rxISR1);                 						 	  //����'CAN receive-complete'���жϴ������
	can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);  //����CAN1�Ĺ�����
	can2.frequency(1000000);                    				  //����CAN2ͨѶ������Ϊ1Mbps
//	can2.attach(&rxISR2);                   							//����'CAN receive-complete'���жϴ������
	can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);  //����CAN2�Ĺ�����
	memset(&tx_buff, 0, TX_LEN * sizeof(uint16_t));				//���仺�����ڴ�
	memset(&spi_data, 0, sizeof(spi_data_t));
	memset(&spi_command,0,sizeof(spi_command_t));
	NVIC_SetPriority(TIM5_IRQn, 1);
  NVIC_SetPriority(CAN1_RX0_IRQn, 3);
  NVIC_SetPriority(CAN2_RX0_IRQn, 3);
	printf("\n\r SPIne\n\r");
	/*����CANͨѶ���ݸ�ʽ*/
	a1_can.len = 8;     // ����8���ֽ�
	a2_can.len = 8;     // ����8���ֽ�
	h1_can.len = 8;
	h2_can.len = 8;
	k1_can.len = 8;
	k2_can.len = 8;
	rxMsg1.len = 6;     //����6���ֽ�
	rxMsg2.len = 6;     //����6���ֽ�
  /*����CAN ID*/
	a1_can.id = 0x1;                        
	a2_can.id = 0x1;                 
	h1_can.id = 0x2;
	h2_can.id = 0x2;
	k1_can.id = 0x3;
	k2_can.id = 0x3;     
	/*CAN����*/	
	pack_cmd(&a1_can, l1_control.a); // CAN ������ṹ������a1_can�����ݣ�l1_control.a�ǵ�һ����ؽ�״̬����
	pack_cmd(&a2_can, l2_control.a); 
	pack_cmd(&h1_can, l1_control.h); 
	pack_cmd(&h2_can, l2_control.h); 
	pack_cmd(&k1_can, l1_control.k); 
	pack_cmd(&k2_can, l2_control.k); 
	WriteAll();

	/*��4����ʼ��SPI*/
	if(!spi_enabled)																								//�ȴ�upboard��CS�������ߣ�Ȼ������SPIͨѶ�������CS��������ʱ��SPI������ 
		{   
			while((spi_enabled==0) && (cs.read() ==0)){wait_us(10);}		//�Ȳ���SPI��CS�������߾�һֱ��
			init_spi();																									//��ʼ��SPI����cs����ʹ�ܺ�ų�ʼ��SPI
			spi_enabled = 1;
		}  

	/*��5����ѭ��*/	
	while(1) 
		{
			counter++;
			can2.read(rxMsg2);  									//�����������can1�źŴ����ջ�����
			unpack_reply(rxMsg2, &l2_state);			//rxMsg2����Ϣ,��can�յ������λ�á��ٶȡ�������Ϣ��λ��ѹ����������������ת���ɸ����ͣ�������ID�Ű�������Ϣ���з���
			can1.read(rxMsg1);                    //�����������can1�źŴ����ջ�����
			unpack_reply(rxMsg1, &l1_state);			//rxMsg2����Ϣ,��can�յ������λ�á��ٶȡ�������Ϣ��λ��ѹ����������������ת���ɸ����ͣ�������ID�Ű�������Ϣ���з���
			wait_us(10);
		} 
	}
    