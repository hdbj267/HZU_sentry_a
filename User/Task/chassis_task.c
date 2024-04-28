/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate��
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   ���ڵ��̵Ŀ���
 * @Note:       
 * @Others: 
**/
#include "chassis_task.h"
#include "pid.h"
#include "can1_app.h"
#include "can2_app.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "math.h"
#include "rule.h"
#include "connect_task.h"
#include "main.h"
#include "oled.h"
#include "stdio.h"
#include "stdlib.h"
//#include "time.h"
#include "judge.h"
#include "ULTRASONIC_task.h"

chassis_control_data_t chassis_control_data;
chassis_pid_t chassis_pid;

float last_in=0,ch2,ch3;
float T=0.001,rotate_speed=0.07;
float out,chassis_value,delta_init_value,last_delta,delta,yaw_set,yaw_fdb;
int yaw_rotate_set=500,yaw_raw1,k=1,delta_value;
uint8_t rotate_init_flag=1,last_mode,delta_start_flag;



volatile uint16_t T1;
volatile int tim;

car_typedef car=
{
	.position_a[0] = 0,
	.position_a[1] = 0,
	.init_width = (uint8_t)(60/map_divide),
	.target[0] = 144,
	.target[1] = 108
};

 pid_t cmd_forward_pid =
{	
	.kp = 22.0,  //6.2  //10stable  20
	.ki = 0.5,  //0.3   //0
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
};
 pid_t cmd_zuoyou_pid =
{	
	.kp = 6.0,  //6.2  //10stable  20
	.ki = 0.0,  //0.3   //0
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
};

//uint32_t RNG_Get_RandomRange(int min,int max)

//{

//return HAL_RNG_GetRandomNumber(&RNG_Handler)%(max-min+1) +min;

//}

float forwardfeed(float in)
{

	if(in<17)
		out=0;
	else if(in>=17&&in<103)
		out=(double)0.0778*in+562.3;
	else if(in>=103&&in<157)
		out=(double)0.1319*in+559.8;
	else if(in>=157&&in<193)
		out=(double)0.0804*in+567.7;
	else if(in>=193&&in<232)
		out=(double)0.0553*in+572.5;
	else if(in>=232&&in<275)
		out=(double)0.0602*in+571.5;
	else if(in>=275)
		out=(double)0.0399*in+575.5;

	return out;
}


int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief        	����pid��ʼ��
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}


/**
  * @brief          ���̳�ʼ��
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_init(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	chassis->connect = get_connect_data_point();
	chassis->cm1_msg = get_cm1_msg_point();
	chassis->cm2_msg = get_cm2_msg_point();
	chassis->cm3_msg = get_cm3_msg_point();
	chassis->cm4_msg = get_cm4_msg_point();
	chassis->yaw_motor_msg = get_yaw_motor_msg_point();
	
	chassis_pid_init(&chassis_pid->cm1_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm2_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm3_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm4_pid, &cali_chassis_pid.cm_pid);
	
	chassis_pid_init(&chassis_pid->rotate_pid, &cali_chassis_pid.rotate_pid);
}
/**
  * @brief        С�����µ��˶����� 
  * @author         
  * @param[in]      
  * @retval			
  * @note        //ǰ�����ָ��ţ�����Ϊ6020�Ƿ���װ�ģ�Ҳ�ɸ���ʵ�ʵ��Եõ� 
  */
void rotate_motion_mode_process(chassis_control_data_t *chassis)
{
	chassis->rotate_motion.yaw_current_ecd = chassis->yaw_motor_msg->encoder.raw_value;
	if(chassis->chassis_control_mode_flag)
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
	}
	else
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
	}
	//�õ�����ֵ����ʱ�뷽��0��360�ȱ仯�ĽǶ�
	if(chassis->rotate_motion.yaw_current_ecd <= chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.											   \
		chassis_gimbal_angle = (float)(chassis->rotate_motion.yaw_init_ecd \
							    - chassis->rotate_motion.yaw_current_ecd)  \
								* GAMBAL_ENCODE_TO_ANGLE;
	}
	else if(chassis->rotate_motion.yaw_current_ecd > chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.   													   \
		chassis_gimbal_angle = 360.0f - (float)(chassis->rotate_motion.yaw_current_ecd \
										 - chassis->rotate_motion.yaw_init_ecd)        \
										 * GAMBAL_ENCODE_TO_ANGLE;
	}
	if(rc_ctrl_data.rc.s1==3&&rc_ctrl_data.rc.s2==2)	
	{
	cmd_forward_pid.set = (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								(-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	
	cmd_zuoyou_pid.set =   (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));	
	}
	else
	{
	chassis->forward_back_set = (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								(-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	
	chassis->left_right_set =   (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
	}
	
}
/**
  * @brief        ��ȡ�ƶ�������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */

uint16_t time1,time2;
uint16_t speed = 0,NORMAL_SPPED=700,SPORT_ROTATE_DECREASE=100;
float num1=0.7,speed1,num2=1.2;
uint8_t close_flag,last_close_flag,move_close_flag;
float speed_factor1,speed_factor2,speed_factor3;

void get_forward_back_value(chassis_control_data_t *chassis)
{
	
//	int16_t speed = 0;
	
	if(chassis->connect->can2_rc_ctrl.control_mode == REMOTE_MODE)      
	{
		if ( RC_abs(chassis->connect->can2_rc_ctrl.rc.ch3) < 500 || RC_abs(chassis->connect->can2_rc_ctrl.rc.ch2) < 500)
		{
			if(chassis->connect->can2_rc_ctrl.rc.ch3 < 0 && chassis->connect->can2_rc_ctrl.rc.ch2 < 0)
			{	
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
										CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if (chassis->connect->can2_rc_ctrl.rc.ch3 < 0 )
			{
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if ( chassis->connect->can2_rc_ctrl.rc.ch2 < 0 )
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else 
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
		}
		else 
		{
			chassis->forward_back = chassis->connect->can2_rc_ctrl.rc.ch3 *    \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			chassis->left_right = chassis->connect->can2_rc_ctrl.rc.ch2 *      \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
		}
		
	}
	else if(rc_ctrl_data.rc.s1==3&&rc_ctrl_data.rc.s2==2)  //С���ݵ���������ֵ��������������С���ݽ���
	{
		chassis->forward_back = -(chassis->connect->forward_speed)*100.0;
		chassis->left_right   = (chassis->connect->zuoyou_speed)*100.0;
	}	
	else if(rc_ctrl_data.rc.s1==1&&rc_ctrl_data.rc.s2==2)  //��ͨ������
	{
		cmd_forward_pid.set = -(chassis->connect->forward_speed)*100.0;
		cmd_zuoyou_pid.set = (chassis->connect->zuoyou_speed)*100.0;
	}	
}
/**
  * @brief         ��ȡ������תֵ  ��̨��ת��ʱ�����ֵ���  ��5000 ��3000
  * @author         
  * @param[in]      
  * @retval			
  * @note          ���̸���pidĿǰ���ȶ� ����ĳЩ����ת�������з����д���
  */
float rotate_abs(float val)
{
	if(val < 0)
	{
		val = -val;
	}
	return val;
}

uint8_t first_rotate=1;
int16_t yaw_raw,delta_yaw=2000,ROTATE_BASE_SPEED=420,rotate_tend=1,rotate_num;

void get_rotate_value(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	yaw_raw1 = chassis->yaw_motor_msg->encoder.raw_value;
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_COMMON_MODE)	//	���̸���pid
	{
		if(chassis->chassis_control_mode_flag)
		{
			chassis_pid->rotate_pid.set =  GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
		}
		else
		{	
			chassis_pid->rotate_pid.set = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
		}
		chassis_pid->rotate_pid.fdb = (float)(chassis->yaw_motor_msg->encoder.raw_value  \
									+ ((chassis->connect->can2_rc_ctrl.gyro.yaw_set \
								    -chassis->connect->can2_rc_ctrl.gyro.yaw_fdb) * GAMBAL_YAW_angle_VALUE+0.5f));//+ chassis->yaw_motor_msg->encoder.round_cnt * 8192
		//chassis_pid->rotate_pid.fdb = (float)chassis->yaw_motor_msg->encoder.raw_value;
//		chassis_value1 = (chassis->connect->can2_rc_ctrl.rc.ch2)*rotate_speed;
//		chassis_value = (chassis->connect->can2_rc_ctrl.gyro.yaw_set \
//								    -chassis->connect->can2_rc_ctrl.gyro.yaw_fdb) * GAMBAL_YAW_angle_VALUE;
		chassis_pid->rotate_pid.Calc(&chassis_pid->rotate_pid);
		
		chassis->rotate = chassis_pid->rotate_pid.output;//�ɸ��޸�Ϊ��   6.25
		//chassis->rotate_buff_flag = 0;

	}
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_MOTION_MODE)   //�����˶�С����    
	{
		
if(RC_abs(chassis->yaw_motor_msg->encoder.raw_value - yaw_raw)>delta_yaw)
		{
		chassis->rotate_buff_flag=1;
		yaw_raw=chassis->yaw_motor_msg->encoder.raw_value;
		}
		if(chassis->rotate_buff_flag|first_rotate==1)      
        {
			 srand(xTaskGetTickCount());
				chassis->rotate = (rand() % (ROTATE_BASE_SPEED+100) + ROTATE_BASE_SPEED);
				chassis->rotate_buff_flag = 0;
				first_rotate=0;
				}
				
	}
else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_STOP_MODE)	//��ֹС����
	{
		rotate_num++;		//��ʱ����ת����
		if(rotate_num>20000)
		{
		rotate_tend=-rotate_tend;
		rotate_num=0;	
		}
		
if(RC_abs(chassis->yaw_motor_msg->encoder.raw_value - yaw_raw)>delta_yaw)
		{
		chassis->rotate_buff_flag=1;
		yaw_raw=chassis->yaw_motor_msg->encoder.raw_value;
		}
		if(chassis->rotate_buff_flag|first_rotate==1)      
        {
			 srand(xTaskGetTickCount());
				chassis->rotate = (rand() % (ROTATE_BASE_SPEED+100) + ROTATE_BASE_SPEED)*rotate_tend;
				chassis->rotate_buff_flag = 0;
				first_rotate=0;
				}
	}
	else 
	{
		chassis->rotate = 0;
	}
}

uint8_t action_flag,sentry_mode,sentry_work_mode,move_speed=200;


/**
  * @brief        ���µ��̵���趨ֵ�ͷ���ֵ
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float speed_factor1,speed_factor2,speed_factor3;
extern game_status_t game_status;;
extern float cmd_yaw;
uint8_t left_move,game_num;
int left_num,left_num1=6000;

void chassis_set_and_fdb_update(chassis_control_data_t *chassis, \
								chassis_pid_t *chassis_pid)
{
	switch(chassis->connect->can2_rc_ctrl.work_mode)
	//switch(sentry_work_mode)
	{
		case ROBOT_CALI_MODE:
		case ROBOT_INIT_MODE:
		case ROBOT_INIT_END_MODE:
		{
			chassis->forward_back = 0; //ģʽת��ʱ����
			chassis->left_right = 0;
			chassis->rotate = 0;

			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
		case ROBOT_COMMON_MODE: //��ͨ���̸���ģʽ
		{

			get_forward_back_value(chassis);
			get_rotate_value(chassis, chassis_pid);
			
			
				if((game_status.game_progress==3)||(game_status.game_progress==4)||(game_status.game_progress==5))/*������ʼ�������ߣ�
																																								��ü�һ�����꿪ɨ�赵�����õĻ��ǵ�ɾ��*/
	{
		switch(left_move)
		{
			case 0:
	chassis->forward_back=-300;
	left_num++;
		if(left_num>left_num1)
		{
			left_move=1;
			left_num=0;
		}
				break;
			case 1:
				left_num=0;
				break;
		}
	}
	//if((game_status.game_progress==3)||(game_status.game_progress==4)||(game_status.game_progress==5))
	//{
		//	switch(sentry_mode)			//Ҳ���������ַ�����������̼Ƽ�״̬�����ߵ�ĳ���ط�
	//{
//	case 0:
//		if(action_flag==1)
//		{
//		sentry_work_mode=ROBOT_COMMON_MODE;
//		sentry_mode=1;
//	}break;
//	case 1:
//	if(car.position_b[0]>=-1*1)//ǰ2��
//		chassis->left_right = -move_speed;
//	else
//		sentry_mode=2;
//	break;
//	case 2:
//	if(car.position_b[1]>=-1*3)//��4��
//		chassis->forward_back = move_speed;
//	else
//		sentry_mode=3;
//	break;
//	case 3:
//	if(car.position_b[0]>=-1*1)//ǰ2��
//		chassis->left_right = -move_speed;
//	else
//		sentry_mode=04;
//	case 4:
//	if(car.position_b[1]<=0)//��4��
//		chassis->forward_back = -move_speed;
//	else
//		sentry_mode=05;
//	case 5:
//	sentry_work_mode=ROBOT_ROTATE_MOTION_MODE;
//	sentry_mode=06;
//	break;
//	default:break;
//	}
	//}
	
	
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_MOTION_MODE: //�˶�С����ģʽ
		{
			if(rc_ctrl_data.rc.s1==3&&rc_ctrl_data.rc.s2==2)	//��ͨ������������ȡ��תֵ�������̲�������̨
			{
			get_forward_back_value(chassis);
			}
			else
			{
			get_forward_back_value(chassis);//��ȡ����ֵ����ʹ�����溯����ת��
			rotate_motion_mode_process(chassis);//�˶�С���ݽ���
			get_rotate_value(chassis, chassis_pid);
			}
			
		}break;
		case ROBOT_ROTATE_STOP_MODE: //��ֹ����С����ģʽ
		{
			if(rc_ctrl_data.rc.s1==1&&rc_ctrl_data.rc.s2==2)	//С����Ѳ����������С���ݽ��㣬������ȡ��תֵ
			{
			get_forward_back_value(chassis);
			rotate_motion_mode_process(chassis);//�˶�С���ݽ���
			}
			else
			{
			get_rotate_value(chassis, chassis_pid);			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = chassis->rotate;
			}
			
		}break;
		default:
		{
			chassis->forward_back = 0;
			chassis->left_right = 0;
			chassis->rotate = 0;
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
	}
	
#if 0
	chassis->rotate_set = 0; //��������ʹ�� ����Ҫ��ת��
#endif	
	



  if((rc_ctrl_data.rc.s1==1||rc_ctrl_data.rc.s1==3)&&rc_ctrl_data.rc.s2==2)//��������ִ�е�������
	{
		cmd_forward_pid.fdb = -car.v[0];
		PID_Calc(&cmd_forward_pid);
		
		//cmd_zuoyou_pid.set = chassis->connect->can2_rc_ctrl.rc.ch3*0.5 ;
		cmd_zuoyou_pid.fdb = car.v[1];
		PID_Calc(&cmd_zuoyou_pid);
		chassis->cm1_set = -cmd_forward_pid.output +cmd_zuoyou_pid.output + chassis->rotate_set;
		chassis->cm2_set = -cmd_forward_pid.output -cmd_zuoyou_pid.output + chassis->rotate_set;
		chassis->cm3_set =  cmd_forward_pid.output -cmd_zuoyou_pid.output + chassis->rotate_set;
		chassis->cm4_set =  cmd_forward_pid.output +cmd_zuoyou_pid.output + chassis->rotate_set;
		}
		else
		{
	chassis->cm1_set = - chassis->forward_back_set + chassis->left_right_set + chassis->rotate_set;
	chassis->cm2_set = chassis->forward_back_set + chassis->left_right_set 	 + chassis->rotate_set;
	chassis->cm3_set = chassis->forward_back_set - chassis->left_right_set   + chassis->rotate_set;
	chassis->cm4_set = - chassis->forward_back_set - chassis->left_right_set + chassis->rotate_set;
		}
			
	chassis->cm1_fdb = chassis->cm1_msg->encoder.filter_rate;
	chassis->cm2_fdb = chassis->cm2_msg->encoder.filter_rate;
	chassis->cm3_fdb = chassis->cm3_msg->encoder.filter_rate;
	chassis->cm4_fdb = chassis->cm4_msg->encoder.filter_rate;
	//�̵�
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_INIT_MODE)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_calculate(chassis_control_data_t *chassis,  \
						   chassis_pid_t *chassis_pid)
{
	chassis_pid->cm1_pid.set = chassis->cm1_set;
	chassis_pid->cm2_pid.set = chassis->cm2_set;
	chassis_pid->cm3_pid.set = chassis->cm3_set;
	chassis_pid->cm4_pid.set = chassis->cm4_set;
	
	chassis_pid->cm1_pid.fdb = chassis->cm1_fdb;
	chassis_pid->cm2_pid.fdb = chassis->cm2_fdb;
	chassis_pid->cm3_pid.fdb = chassis->cm3_fdb;
	chassis_pid->cm4_pid.fdb = chassis->cm4_fdb;
	
	chassis_pid->cm1_pid.Calc(&chassis_pid->cm1_pid);
	chassis_pid->cm2_pid.Calc(&chassis_pid->cm2_pid);
	chassis_pid->cm3_pid.Calc(&chassis_pid->cm3_pid);
	chassis_pid->cm4_pid.Calc(&chassis_pid->cm4_pid);
}


void chassis_forwardfeed(chassis_control_data_t *chassis)
{
 chassis->cm1_ff=forwardfeed(RC_abs(chassis->cm1_msg->encoder.filter_rate));
 chassis->cm2_ff=forwardfeed(RC_abs(chassis->cm2_msg->encoder.filter_rate));
	chassis->cm3_ff=forwardfeed(RC_abs(chassis->cm3_msg->encoder.filter_rate));
	chassis->cm4_ff=forwardfeed(RC_abs(chassis->cm4_msg->encoder.filter_rate));
	if(chassis->cm1_msg->encoder.filter_rate>0)
	{
	chassis->cm2_ff=-chassis->cm2_ff;
	chassis->cm3_ff=-chassis->cm3_ff;		
	}
	else
	{
	chassis->cm1_ff=-chassis->cm1_ff;
	chassis->cm4_ff=-chassis->cm4_ff;
	}
		
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */

void chassis_control_loop(chassis_control_data_t *chassis, \
						  chassis_pid_t *chassis_pid)
{
	chassis->given_current.cm1 = chassis_pid->cm1_pid.output ;
	chassis->given_current.cm2 = chassis_pid->cm2_pid.output ;
	chassis->given_current.cm3 = chassis_pid->cm3_pid.output ;
	chassis->given_current.cm4 = chassis_pid->cm4_pid.output ;
	
	if(rc_ctrl_data.rc.s1==2&&rc_ctrl_data.rc.s2==2)
	{
		set_chassis_stop();
	}
	else 
	{
		set_chassis_behaviour(chassis->given_current.cm1,
							  chassis->given_current.cm2,
							  chassis->given_current.cm3,
							  chassis->given_current.cm4);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           void chassis_task(void * pvParameters)
  */
 
void chassis_task(void *argument)
{
	TickType_t current_time = 0;

	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_control_data, &chassis_pid);
	
	while(1)
	{
		current_time = xTaskGetTickCount();                         //��ǰϵͳʱ��       *hyj
		chassis_set_and_fdb_update(&chassis_control_data, &chassis_pid);
		srand(xTaskGetTickCount());
		chassis_pid_calculate(&chassis_control_data, &chassis_pid);
		chassis_forwardfeed(&chassis_control_data);
		chassis_control_loop(&chassis_control_data, &chassis_pid);
		vTaskDelayUntil(&current_time, 1);       //1msһ��         *hyj
	}
}

