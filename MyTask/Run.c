#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "PID.h"
#include "infrared_host.h"
extern uint8_t ready;

Joint_t Joint[5];
int16_t can_buf[4] = {0};

uint8_t enable_Joint[5] = {1,1,1,1,1};//1,1,1,1,1

TaskHandle_t Motor_Drive_Handle;

RobStride_t rs03={.hcan=&hcan2,.motor_id=0x02,.type= RobStride_03 };


float rs03_torque=-0.0f,rs03_rad=-0.2f,rs03_omega=0.2f;
float rs03_kp=350.0f,rs03_kd=38.0f;
PID rs03_pos_pid;  // λ�û�PID



void rs03_PID_Init() {

	rs03_pos_pid.Kp = 7.0f;
	rs03_pos_pid.Ki = 0.1f;
	rs03_pos_pid.Kd = 7.0f;
	rs03_pos_pid.limit = 100.0f;
	rs03_pos_pid.output_limit = 50.0f;

}


void Motor_Drive(void *param)
{
  RobStrideEnable(&rs03);
	TickType_t Last_wake_time = xTaskGetTickCount();

	rs03_PID_Init();

	for(;;)
	{
			float curr_rad = rs03.state.rad;    
			float curr_omega = rs03.state.omega;				
		

			PID_Control(curr_rad, rs03_rad, &rs03_pos_pid);
			rs03_torque = rs03_pos_pid.pid_out;
		 
		
		
		// for(uint8_t i = 0; i < 3; i++)
		// {
		// 	PID_Control(Joint[i].Rs_motor.state.rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
		// 	PID_Control(Joint[i].Rs_motor.state.omega, Joint[i].pos_pid.pid_out + Joint[i].exp_omega, &Joint[i].vel_pid);
//		 	RobStrideTorqueControl(&Joint[i].Rs_motor, Joint[i].vel_pid.pid_out * enable_Joint[i]);
//		 }
		// vTaskDelay(1);
		// PID_Control(Joint[3].Rs_motor.state.rad, 	Joint[3].exp_rad + Joint[3].pos_offset, &Joint[3].pos_pid);
		// PID_Control(Joint[3].Rs_motor.state.omega, Joint[3].pos_pid.pid_out + Joint[3].exp_omega, &Joint[3].vel_pid);
//		  RobStrideTorqueControl(&rs03,rs03_torque);
    RobStrideMotionControl(&rs03, 0x02, rs03_torque, rs03_rad, rs03_omega, rs03_kp, rs03_kd);

		// PID_Control(Joint[4].RM_motor.actual_pos, Joint[4].exp_rad - Joint[4].pos_offset, &Joint[4].RM_motor.pos_pid);
		// PID_Control(Joint[4].RM_motor.motor.Speed, Joint[4].RM_motor.pos_pid.pid_out, &Joint[4].RM_motor.vel_pid);
		// can_buf[0] = Joint[4].RM_motor.vel_pid.pid_out * enable_Joint[4];
		// MotorSend(&hcan2 ,0x200, can_buf);
		
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}

Arm_t arm_t;
TaskHandle_t MotorSendTask_Handle;

SemaphoreHandle_t cdc_recv_semphr;
Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	if(!ready)
		return;
	cur_recv_size=size;
	if(((Arm_t *)src )->pack_type == 0x01)
	{
		memcpy(&arm_Rec_t, src, sizeof(arm_Rec_t));
		
		
		
		
		
			Joint[0].exp_rad = arm_Rec_t.joints[0].rad;
			Joint[0].exp_omega = arm_Rec_t.joints[0].omega;
			Joint[0].exp_torque = arm_Rec_t.joints[0].torque;
				
			Joint[1].exp_rad = arm_Rec_t.joints[1].rad;
			Joint[1].exp_omega = arm_Rec_t.joints[1].omega;
			Joint[1].exp_torque = arm_Rec_t.joints[1].torque;
			
			Joint[2].exp_rad = arm_Rec_t.joints[2].rad * Joint[2].inv_motor;
			Joint[2].exp_omega = arm_Rec_t.joints[2].omega * Joint[2].inv_motor;
			Joint[2].exp_torque = arm_Rec_t.joints[2].torque * Joint[2].inv_motor;
			
			Joint[3].exp_rad = arm_Rec_t.joints[3].rad * Joint[3].inv_motor;
			Joint[3].exp_omega = arm_Rec_t.joints[3].omega * Joint[3].inv_motor;
			Joint[3].exp_torque = arm_Rec_t.joints[3].torque * Joint[3].inv_motor;
			
			Joint[4].exp_rad =( arm_Rec_t.joints[4].rad / 6.28319f * 36.0f * 8192.0f);
			Joint[4].exp_omega = arm_Rec_t.joints[4].omega;
			Joint[4].exp_torque = arm_Rec_t.joints[4].torque;
	}
}

void MotorSendTask(void *param)// 向PC发送数据
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	arm_t.pack_type = 1;
	
	for(;;)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			arm_t.joints[i].rad = (Joint[i].Rs_motor.state.rad -  Joint[i].pos_offset) * Joint[i].inv_motor;
			arm_t.joints[i].omega = Joint[i].Rs_motor.state.omega;
			arm_t.joints[i].torque = Joint[i].Rs_motor.state.torque;
		}
		
		arm_t.joints[4].rad =(((Joint[4].RM_motor.actual_pos + Joint[4].pos_offset)/ 8192.0f / 36.0f) * 2.0f * 3.1415926f) * Joint[4].inv_motor;
		arm_t.joints[4].omega = Joint[4].RM_motor.motor.Speed / 36.0f * 3.1415926f /30.0f;
		
		CDC_Transmit_FS((uint8_t*)&arm_t, sizeof(arm_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(20));
	}
}

uint8_t count = 0; 
TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param)// 从PC接收数据
{
	TickType_t Last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();  // 已在freertos.c中创建，此处保留备用
  xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		if(xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			count ++;
			
		}
	}
} 
uint8_t buf[8];
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//if (hcan->Instance == CAN1)
	{
		
		uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
		
		RobStrideRecv_Handle(&rs03, hcan, ID, buf);
//		RobStrideRecv_Handle(&Joint[1].Rs_motor, &hcan1, ID, buf);
//		RobStrideRecv_Handle(&Joint[2].Rs_motor, &hcan1, ID, buf);
//		RobStrideRecv_Handle(&Joint[3].Rs_motor, &hcan1, ID, buf);
	}
}
		uint16_t ID;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		uint8_t rx_data[8];
		CAN_RxHeaderTypeDef rx_header;
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
			IR_Host_ProcessRxFrame(&rx_header, rx_data);
		}
	}
	else if (hcan->Instance == CAN2)
	{
		uint8_t buf[8];
		ID = CAN_Receive_DataFrame(&hcan2, buf);
//		Motor2006Recv(&Joint[4].RM_motor, &hcan2, ID, buf);
	}
}


