#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "infrared_host.h"
#include "math.h"
#include "stdbool.h"

extern Joint_t Joint[5];
float Motor_Init[4] = {0};

extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t Motor_RM_Handle;

void MotorInit(void);
void Motor_RM(void *param);

bool Float_S(float a, float b)
{
		return fabsf(a - b) < 0.03f;
}
uint8_t F_buf[4] = {0};
bool Joint_FinInit()
{
		F_buf[0] = Float_S(Joint[0].Rs_motor.state.rad, 0 + Joint[0].pos_offset);
		F_buf[1] = Float_S(Joint[1].Rs_motor.state.rad, 0 + Joint[1].pos_offset);
		F_buf[2] = Float_S(Joint[2].Rs_motor.state.rad, -1.57 + Joint[2].pos_offset);
		F_buf[3] = Float_S(Joint[3].Rs_motor.state.rad, 0 + Joint[3].pos_offset);

		if(F_buf[0] && F_buf[1]&& F_buf[2]&& F_buf[3])
			return true;
		else
			return false;
}

extern RobStride_t rs03;
extern RobStride_t rs02;
extern float rs03_torque;
extern float rs02_torque;

void Task_Init(void)
{
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
	  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);

//	  vTaskDelay(2000);
//	  RobStrideInit(&rs03, &hcan2, 0x02, RobStride_03);
//	  RobStrideInit(&rs02, &hcan2, 0x03, RobStride_02);
//	  RobStrideSetMode(&rs03, RobStride_Torque);
//	  RobStrideSetMode(&rs02, RobStride_Torque);
//		vTaskDelay(100);
//  	RobStrideEnable(&rs03);
//	  RobStrideEnable(&rs02);

//	xTaskCreate(Motor_Drive, "Motor_Drive", 256, NULL, 4, &Motor_Drive_Handle);
//	xTaskCreate(Motor_RM, "Motor_RM", 256, NULL, 4, &Motor_RM_Handle);

    IR_Test_StartTask();
}

void RampToTarget(float *val, float target, float step)//斜坡
{
    float diff = target - *val;

    if (fabsf(diff) < step)
    {
        *val = target;
    }
    else
    {
        *val += (diff > 0 ? step : -step);
    }
}

uint8_t ready=0;


void PID_Init_Pos(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->pos_pid.Kp = kp;
	Joint->pos_pid.Ki = ki;
	Joint->pos_pid.Kd = kd;
	Joint->pos_pid.limit = limit;
	Joint->pos_pid.output_limit = pid_out;
}

void PID_Init_Vel(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->vel_pid.Kp = kp;
	Joint->vel_pid.Ki = ki;
	Joint->vel_pid.Kd = kd;
	Joint->vel_pid.limit = limit;
	Joint->vel_pid.output_limit = pid_out;
}

void RS_Offest_inv(Joint_t *Joint, int8_t inv_motor, float pos_offset)
{
	Joint->inv_motor = inv_motor;
	Joint->pos_offset = pos_offset;
}

