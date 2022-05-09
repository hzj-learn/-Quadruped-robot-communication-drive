#include "mbed.h"
#include "math_ops.h"
#include <cstring>
#include "leg_message.h"

//接收/发送缓冲器的长度
#define RX_LEN 66
#define TX_LEN 66

//传入/传出消息的长度
#define DATA_LEN 30
#define CMD_LEN  66

//主机的CAN ID
#define CAN_ID 0x0

/// 数值限制 ///
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
 
 /// 关节软停止 ///
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

spi_data_t spi_data;       //从通讯板到upboard的数据 
spi_command_t spi_command; //从upboard到通讯板的数据

// spi 缓冲区
uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

DigitalOut led(PC_5);

Serial       pc(PA_2, PA_3);
CAN          can1(PB_12, PB_13);  // CAN Rx引脚定义, CAN Tx引脚定义
CAN          can2(PB_8, PB_9); 	  // CAN Rx引脚定义, CAN Tx引脚定义

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

void test_control();//从SPI命令生成伪SPI数据 
void control();

/// CAN 命令包结构 ///
/// 16 bit 位置命令 	单位范围在 -4*pi and 4*pi
/// 12 bit 速度命令 	单位范围在 -30 and + 30 rad/s
/// 12 bit kp 				单位范围在 0 and 500 N-m/rad
/// 12 bit kd 				单位范围在0 and 100 N-m*s/rad
/// 12 bit 反馈力矩		单位范围在 -18 and 18 N-m
/// CAN 命令包是8个字节的，格式如下（满足任何频率）
/// bit 0 是 LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
/// CAN 命令包结构，发送MCU命令用的///
void pack_cmd(CANMessage * msg, joint_control joint)
{
     
     /// 将数据限制在范围内 ///
     float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);                    
     float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
     float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
     float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
     float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
     /// 将浮点数转换位无符号整数 ///
     uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     ///把ints装入can缓冲区  ///
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


	 
/// CAN应答包结构 ///
/// 16 bit位置数据 			单位范围在 -4*pi and 4*pi
/// 12 bit速度数据 			单位范围在 -30 and + 30 rad/s
/// 12 bit电流数据      单位范围在 -40A and 40A;
/// CAN 数据包时58个字节，格式如下（满足任何频率）
/// bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]	 
/// CAN应答包结构 ,应答电机信号用的 ///	
//功能：把can收到电机的位置、速度、力矩信息按位解压出来，并数据类型转换成浮点型，最后根据ID号把三个信息进行分配
//备注：发送的时候转换成整型式因为数据两较小
void unpack_reply(CANMessage msg, leg_state * leg)
{
	/// 从CAN缓冲区解压ints ///
	uint16_t id = msg.data[0];
	uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
	uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
	uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
	/// 将uint（无字符整形） 转换到float（浮点型）数据///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
	
	if(id==1)					//ID1电机（胯关节）
		{
			leg->a.p = p;
			leg->a.v = v;
			leg->a.t = t;
		}
	else if(id==2)	  //ID2电机（大腿关节）
		{
			leg->h.p = p;
			leg->h.v = v;
			leg->h.t = t;
		}
	else if(id==3)		//ID3电机（小腿关节）
		{
			leg->k.p = p;
			leg->k.v = v;
			leg->k.t = t;
		}
	} 
/////////////////////////////////////////////////////////////////
	
	
	
/*CAN1数据中断接收服务函数*/
//功能：	（1）将CAN数据读写RX消息储存器
//				（2）应答电机，表示收到信息
//没有用到，功能再main得死循环里面实现了
void rxISR1() 
{
	can1.read(rxMsg1);                    //将CAN数据读写RX消息储存器
	unpack_reply(rxMsg1, &l1_state);
}
/////////////////////////////////////////////////////////////////
	
	
/*CAN2数据中断接收服务函数*/
//功能：	（1）将CAN数据读写RX消息储存器
//				（2）应答电机，表示收到信息
//没有用到，功能再main得死循环里面实现了
void rxISR2()
{
	can2.read(rxMsg2);									//将CAN数据读写RX消息储存器
	unpack_reply(rxMsg2, &l2_state);
}
/////////////////////////////////////////////////////////////////
	
	
	
/*压缩所有CAN数据*/
void PackAll()
{
	//解压胯关节电机can信号
	pack_cmd(&a1_can, l1_control.a); 
	pack_cmd(&a2_can, l2_control.a); 
	//解压大腿关节电机can信号
	pack_cmd(&h1_can, l1_control.h); 
	pack_cmd(&h2_can, l2_control.h); 
	//解压小腿关节电机can信号
	pack_cmd(&k1_can, l1_control.k); 
	pack_cmd(&k2_can, l2_control.k); 
}
/////////////////////////////////////////////////////////////////


	
/*把四路CAN数据写入对应的缓存区，CAN收发器回自动发送的*/
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
	
	
	
/*发送命令函数*/
//功能（1）解压所有CAN数据
//		（2）把四路CAN数据写入对应的缓存区，CAN收发器回自动发送的
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


/*把can数据清零*/   
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
	
	
	
/*进入电机力矩模式*/	
//备注：这里的can数据是对应驱动can的特殊指令的	
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


/*退出电机力矩模式*/	
//备注：这里的can数据是对应驱动can的特殊指令的
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
	
	
/// 串口处理串行终端键盘命令 ///	
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

		

//SPI异或校验和		
uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;
}
/////////////////////////////////////////////////////////////////



/*SPI中断服务函数*/
//接收来自 UPBORD板的数据命令，读取后，保存进去SPI命令中 
//同时发送底层关节数据给upboard
void spi_isr(void)
{
	GPIOC->ODR |= (1 << 8);
	GPIOC->ODR &= ~(1 << 8);
	int bytecount = 0;
	SPI1->DR = tx_buff[0];
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	while(cs == 0) 													//upboard使能cs引脚后，SPI数据进行交换
		{
		if(SPI1->SR&0x1) 
			{
				rx_buff[bytecount] = SPI1->DR;    //接收来自 UPBORD板的数据命令
				bytecount++;
				if(bytecount<TX_LEN) 
					{
						SPI1->DR = tx_buff[bytecount];//把准备上报的关节数据上传给UPBORD 数据交换
					}
			}
		} 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
	


  /////////////////////////////////////////////////////////////////////////////////////////////////////////		
    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32); //读回来的数据应该先检查校验和！ 
    for(int i = 0; i < CMD_LEN; i++)			 //读取后，保存进去SPI命令中 
    {
     ((uint16_t*)(&spi_command))[i] = rx_buff[i];
    }   
    //运行控件，为下一次迭代填充tx_buff
    if(calc_checksum != spi_command.checksum)
			{
        spi_data.flags[1] = 0xdead;
			}     
    //test_control();
    //spi_data.q_abad[0] = 12.0f;
    control();
    PackAll();		//把UPBORD 板过来的信息封装到每条腿上
		WriteAll();		//经过CAN 发送给每个关节
}
/////////////////////////////////////////////////////////////////


/*关节位置限幅*/
int softstop_joint(joint_state state, joint_control * control, float limit_p, float limit_n)
{
    if((state.p)>=limit_p)//关节位置过大限幅
			{
        //control->p_des = limit_p;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP*(limit_p - state.p);
        return 1;
      }
    else if((state.p)<=limit_n)//关节位置过小限幅
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

		
/*控制SPI/CAN双向数据交换并发送*/
//双向模式
//停止模式
void control()
{  
///////////////////////////////////根据电机状态enabled和upboard的命令spi_command.flags设置是否进入电机模式////////////////////////////////////////////
	if(((spi_command.flags[0]&0x1)==1)  && (enabled==0))						//进入CAN的电机模式，条件：（电机启动）enabled=0，（upboard的）spi_command.flags[0]=0
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
		else if((((spi_command.flags[0]&0x1))==0)  && (enabled==1))	  //退出CAN的电机模式，条件：enabled=1，spi_command.flags[0]=1
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
		 
///////////////////////////将CAN 中断 每个关节获取的信息 取出来准备经过SPI 回应给UPBORAD /////////////////////////////////
		//把MCU1电机can反馈的位置信息转存到SPI数据缓存区
		spi_data.q_abad[0] = l1_state.a.p;
		spi_data.q_hip[0] = l1_state.h.p;
		spi_data.q_knee[0] = l1_state.k.p;
		//把MCU1电机can反馈的速度信息转存到SPI数据缓存区	
		spi_data.qd_abad[0] = l1_state.a.v;
		spi_data.qd_hip[0] = l1_state.h.v;
		spi_data.qd_knee[0] = l1_state.k.v; 
		//把MCU2电机can反馈的位置信息转存到SPI数据缓存区
		spi_data.q_abad[1] = l2_state.a.p;
		spi_data.q_hip[1] = l2_state.h.p;
		spi_data.q_knee[1] = l2_state.k.p;
		//把MCU2电机can反馈的速度信息转存到SPI数据缓存区			
		spi_data.qd_abad[1] = l2_state.a.v;
		spi_data.qd_hip[1] = l2_state.h.v;
		spi_data.qd_knee[1] = l2_state.k.v;
		
		if(estop==0)	//停止所有通信模式
			{
				//printf("estopped!!!!\n\r");
				memset(&l1_control, 0, sizeof(l1_control));//开内存，大小和l1_control一样，用0填充数据
				memset(&l2_control, 0, sizeof(l2_control));
				spi_data.flags[0] = 0xdead;
				spi_data.flags[1] = 0xdead;
				led = 1;
			}
	

			
		else			//用upboard发过来的数据填充CAN控制，同时把电机CAN的数据发送给upboard,双向通讯模式
			{
				led = 0; 
				memset(&l1_control, 0, sizeof(l1_control));//开内存，大小和l1_control一样，用0填充数据
				memset(&l2_control, 0, sizeof(l2_control));
				/*CAN1*/
				//用upboard SPI发送过来的的指令填充MCU CAN1跨关节的五个数据
				l1_control.a.p_des = spi_command.q_des_abad[0];
				l1_control.a.v_des  = spi_command.qd_des_abad[0];
				l1_control.a.kp = spi_command.kp_abad[0];
				l1_control.a.kd = spi_command.kd_abad[0];
				l1_control.a.t_ff = spi_command.tau_abad_ff[0];
				//用upboard SPI发送过来的的指令填充MCU CAN1髋关节的五个数据
				l1_control.h.p_des = spi_command.q_des_hip[0];
				l1_control.h.v_des  = spi_command.qd_des_hip[0];
				l1_control.h.kp = spi_command.kp_hip[0];
				l1_control.h.kd = spi_command.kd_hip[0];
				l1_control.h.t_ff = spi_command.tau_hip_ff[0];
				//用upboard SPI发送过来的的指令填充MCU CAN1膝关节的五个数据
				l1_control.k.p_des = spi_command.q_des_knee[0];
				l1_control.k.v_des  = spi_command.qd_des_knee[0];
				l1_control.k.kp = spi_command.kp_knee[0];
				l1_control.k.kd = spi_command.kd_knee[0];
				l1_control.k.t_ff = spi_command.tau_knee_ff[0];
				
				
				/*CAN2*/
				//用upboard SPI发送过来的的指令填充MCU CAN2跨关节的五个数据
				l2_control.a.p_des = spi_command.q_des_abad[1];
				l2_control.a.v_des  = spi_command.qd_des_abad[1];
				l2_control.a.kp = spi_command.kp_abad[1];
				l2_control.a.kd = spi_command.kd_abad[1];
				l2_control.a.t_ff = spi_command.tau_abad_ff[1];
				//用upboard SPI发送过来的的指令填充MCU CAN2髋关节的五个数据
				l2_control.h.p_des = spi_command.q_des_hip[1];
				l2_control.h.v_des  = spi_command.qd_des_hip[1];
				l2_control.h.kp = spi_command.kp_hip[1];
				l2_control.h.kd = spi_command.kd_hip[1];
				l2_control.h.t_ff = spi_command.tau_hip_ff[1];
				//用upboard SPI发送过来的的指令填充MCU CAN2膝关节的五个数据
				l2_control.k.p_des = spi_command.q_des_knee[1];
				l2_control.k.v_des  = spi_command.qd_des_knee[1];
				l2_control.k.kp = spi_command.kp_knee[1];
				l2_control.k.kd = spi_command.kd_knee[1];
				l2_control.k.t_ff = spi_command.tau_knee_ff[1];	//得到来自UPBORAD 上的命令，取出准备经过CAN下发给对应的关节
					
				spi_data.flags[0] = 0;    //复位
				spi_data.flags[1] = 0;    //复位
				spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
				spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N))<<1;
			
				spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
				spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N))<<1;
			}
			spi_data.checksum = xor_checksum((uint32_t*)&spi_data,14);
			for(int i = 0; i < DATA_LEN; i++)
			{
			tx_buff[i] = ((uint16_t*)(&spi_data))[i]; 			//将CAN 获取的关节信息 准备在SPI 发送缓冲区	
			}     
}
/////////////////////////////////////////////////////////////////    


//SPI发送数据给upboard测试函数
//备注：SPI命令事从upboard上接收的，接收后存在SPI数据里面
void test_control()
{
	for(int i = 0; i < 2; i++)//执行两次，因为改更新两个单片机的值，0代表MCU1,1代表MCU2
	{
		//位置项
		spi_data.q_abad[i] = spi_command.q_des_abad[i] + 1.f;			//把胯关节的位置命令+1后，赋值给SPI数据
		spi_data.q_knee[i] = spi_command.q_des_knee[i] + 1.f;			//把髋关节的位置命令+1后，赋值给SPI数据
		spi_data.q_hip[i]  = spi_command.q_des_hip[i]  + 1.f;			//把膝关节的位置命令+1后，赋值给SPI数据
		//速度项
		spi_data.qd_abad[i] = spi_command.qd_des_abad[i] + 1.f;		//把胯关节的速度命令+1后，赋值给SPI数据
		spi_data.qd_knee[i] = spi_command.qd_des_knee[i] + 1.f;		//把髋关节的速度命令+1后，赋值给SPI数据
		spi_data.qd_hip[i]  = spi_command.qd_des_hip[i]  + 1.f;		//把膝关节的速度命令+1后，赋值给SPI数据
	}  
	spi_data.flags[0] = 0xdead;
	//只从消息的前56个字节开始 
	spi_data.checksum = xor_checksum((uint32_t*)&spi_data,14);	//SPI异或校验和
	
	for(int i = 0; i < DATA_LEN; i++)														//把上面更新的SPI数据放入SPI发送缓冲区里面
	{
		tx_buff[i] = ((uint16_t*)(&spi_data))[i];
	}       
}



/////////////////////////////////////////////////////////////////


/*
*初始化SPI引脚，设置SPI频率为12M
*/
void init_spi(void)
{
	SPISlave *spi = new SPISlave(PA_7, PA_6, PA_5, PA_4);		//初始化SPI引脚
	spi->format(16, 0);
	spi->frequency(12000000);																//设置SPI频率为12M
	spi->reply(0x0);
	cs.fall(&spi_isr);
	printf("done\n\r");
}
/////////////////////////////////////////////////////////////////


int main() 
{
	///////////////////////////////////////////*硬件初始化*/////////////////////////////////////////
	/*（1）初始化串口*/
	pc.baud(921600);																			//设置串口的波特率（键盘）
	pc.attach(&serial_isr); 															//连接串口中断服务函数（键盘）
	
	/*（2）初始化通讯模式*/
	estop.mode(PullUp);																		//设置通讯模式，上拉就是停止CAN/SPI通讯，下拉就是启动CAN/SPI双向通讯
	
	/*（3）初始化CAN通讯*/
	can1.frequency(1000000);      												//设置CAN1通讯波特率为1Mbps
	//can1.attach(&rxISR1);                 						 	  //连接'CAN receive-complete'到中断处理程序
	can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);  //设置CAN1的过滤器
	can2.frequency(1000000);                    				  //设置CAN2通讯波特率为1Mbps
//	can2.attach(&rxISR2);                   							//连接'CAN receive-complete'到中断处理程序
	can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);  //设置CAN2的过滤器
	memset(&tx_buff, 0, TX_LEN * sizeof(uint16_t));				//分配缓冲区内存
	memset(&spi_data, 0, sizeof(spi_data_t));
	memset(&spi_command,0,sizeof(spi_command_t));
	NVIC_SetPriority(TIM5_IRQn, 1);
  NVIC_SetPriority(CAN1_RX0_IRQn, 3);
  NVIC_SetPriority(CAN2_RX0_IRQn, 3);
	printf("\n\r SPIne\n\r");
	/*设置CAN通讯数据格式*/
	a1_can.len = 8;     // 传输8个字节
	a2_can.len = 8;     // 传输8个字节
	h1_can.len = 8;
	h2_can.len = 8;
	k1_can.len = 8;
	k2_can.len = 8;
	rxMsg1.len = 6;     //接收6个字节
	rxMsg2.len = 6;     //接收6个字节
  /*设置CAN ID*/
	a1_can.id = 0x1;                        
	a2_can.id = 0x1;                 
	h1_can.id = 0x2;
	h2_can.id = 0x2;
	k1_can.id = 0x3;
	k2_can.id = 0x3;     
	/*CAN控制*/	
	pack_cmd(&a1_can, l1_control.a); // CAN 命令包结构，发送a1_can的数据，l1_control.a是第一个胯关节状态限制
	pack_cmd(&a2_can, l2_control.a); 
	pack_cmd(&h1_can, l1_control.h); 
	pack_cmd(&h2_can, l2_control.h); 
	pack_cmd(&k1_can, l1_control.k); 
	pack_cmd(&k2_can, l2_control.k); 
	WriteAll();

	/*（4）初始化SPI*/
	if(!spi_enabled)																								//等待upboard把CS引脚拉高，然后启动SPI通讯，如果在CS引脚拉低时，SPI不工作 
		{   
			while((spi_enabled==0) && (cs.read() ==0)){wait_us(10);}		//等不到SPI的CS引脚拉高就一直等
			init_spi();																									//初始化SPI，等cs引脚使能后才初始化SPI
			spi_enabled = 1;
		}  

	/*（5）死循环*/	
	while(1) 
		{
			counter++;
			can2.read(rxMsg2);  									//读电机反馈的can1信号带接收缓存器
			unpack_reply(rxMsg2, &l2_state);			//rxMsg2的信息,把can收到电机的位置、速度、力矩信息按位解压出来，并数据类型转换成浮点型，最后根据ID号把三个信息进行分配
			can1.read(rxMsg1);                    //读电机反馈的can1信号带接收缓存器
			unpack_reply(rxMsg1, &l1_state);			//rxMsg2的信息,把can收到电机的位置、速度、力矩信息按位解压出来，并数据类型转换成浮点型，最后根据ID号把三个信息进行分配
			wait_us(10);
		} 
	}
    