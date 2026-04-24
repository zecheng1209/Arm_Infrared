//在本文件中包含infrared.c中的函数没有使用，仅作为上位机开发的参考

//红外模块实现  具有解析can数据帧、ack确认、crc校验、超时重传等功能
#include "infrared.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern TIM_HandleTypeDef htim1;  // 用于 38KHz PWM 载波 (PA8 - TIM1_CH1)
extern TIM_HandleTypeDef htim2;  // 用于接收输入捕获 (PA15 - TIM2_CH1)，该芯片不支持上下沿同时捕获，程序中设置的为上升沿捕获。
extern TIM_HandleTypeDef htim3;  // 用于发送时序控制

uint8_t received_data[9];
uint8_t bit_index = 0;
uint32_t last_capture_time = 0;
uint8_t receiving = 0;
uint32_t rx_last_activity_time = 0;

static IR_TX_Context_t tx_context = {0};

static void IR_TX_SetNextTimer(uint16_t delay_us)//设置红外发送定时器
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, delay_us);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
}
 
void IR_Init(void)//初始化红外模块
{
    HAL_TIM_IC_Start_IT(&htim2, IC_CHANNEL);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
    tx_context.state = IR_TX_IDLE;
    tx_context.busy = false;
    rx_last_activity_time = HAL_GetTick();
}

bool IR_SendData(uint8_t *data, uint8_t length)//发送红外数据帧
{
    if (length > 8) return false;
    if (tx_context.busy) return false;
    
    uint32_t time_since_last_tx = HAL_GetTick() - tx_context.last_tx_time;
    if (time_since_last_tx < IR_TX_FRAME_INTERVAL_MS) {
        return false;
    }

    uint8_t crc = IR_CRC8(data, length);
    memcpy(tx_context.frame, data, length);
    tx_context.frame[length] = crc;
    tx_context.total_bytes = length + 1;
    tx_context.byte_index = 0;
    tx_context.bit_index = 7;
    tx_context.busy = true;

    tx_context.state = IR_TX_START_PULSE;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    IR_TX_SetNextTimer(START_PULSE_LEN);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_SET);
    return true;
}

bool IR_IsTXBusy(void)//判断红外发送是否繁忙
{
    return tx_context.busy;
}

void IR_TX_TimerCallback(TIM_HandleTypeDef *htim)//红外发送定时器中断回调函数
{
    if (htim->Instance != TIM3) return;
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    
    switch (tx_context.state) {
        case IR_TX_START_PULSE:
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.state = IR_TX_START_SPACE;
            IR_TX_SetNextTimer(START_SPACE_LEN);
            break;

        case IR_TX_START_SPACE:
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            tx_context.state = IR_TX_DATA_HIGH;
            IR_TX_SetNextTimer(BIT_ONE_HIGH);
            break;

        case IR_TX_DATA_HIGH:
            {
                uint8_t current_bit = (tx_context.frame[tx_context.byte_index] >> tx_context.bit_index) & 0x01;
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
                tx_context.state = IR_TX_DATA_LOW;
                IR_TX_SetNextTimer(current_bit ? BIT_ONE_LOW : BIT_ZERO_LOW);
            }
            break;

        case IR_TX_DATA_LOW:
            if (tx_context.bit_index > 0) {
                tx_context.bit_index--;
            } else {
                tx_context.bit_index = 7;
                tx_context.byte_index++;
            }

            if (tx_context.byte_index >= tx_context.total_bytes) {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                tx_context.state = IR_TX_STOP_PULSE;
                IR_TX_SetNextTimer(BIT_ONE_HIGH);
            } else {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                tx_context.state = IR_TX_DATA_HIGH;
                IR_TX_SetNextTimer(BIT_ONE_HIGH);
            }
            break;

        case IR_TX_STOP_PULSE://停止发送脉冲
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.state = IR_TX_IDLE;
            tx_context.busy = false;
            tx_context.last_tx_time = HAL_GetTick();
            HAL_TIM_Base_Stop_IT(&htim3);
            break;

        default://默认状态
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.busy = false;
            tx_context.state = IR_TX_IDLE;
            HAL_TIM_Base_Stop_IT(&htim3);
            break;
    }
}

uint8_t IR_CRC8(uint8_t *data, uint8_t length)//计算CRC8校验和
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//红外接收捕获中断回调函数
{
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, IC_CHANNEL);
    rx_last_activity_time = HAL_GetTick();

    if (last_capture_time != 0) {
        uint32_t pulse_duration = (capture_value < last_capture_time) ?
                         (0xFFFF - last_capture_time + capture_value) :
                         (capture_value - last_capture_time);

        if (!receiving) {
            // 检测起始信号：4.5ms低电平(载波) + 2.25ms高电平(空闲)
            // 总共约6.75ms (6750us)
            uint32_t start_total = START_PULSE_LEN + START_SPACE_LEN; // 6750us
            uint32_t tolerance = IR_PULSE_TOLERANCE_US * 3; // 起始信号容差稍大
            if (pulse_duration >= (start_total - tolerance) &&
                pulse_duration <= (start_total + tolerance)) {
                receiving = 1;
                bit_index = 0;
                memset(received_data, 0, 9);
            }
        } else {
            // 正在接收中
            if (bit_index >= 72) {
                last_capture_time = capture_value;
                return;
            }

            // 数据位判断：位0=280us高+280us低 总计560us，位1=280us高+840us低 总计1120us
            // 红外接收模块是接收到38kHZ的红外信息，就会将那个口设置为高电平，低电平则设置为低电平。
            // 本模块通过捕获这个口的高低变化，判断是位0还是位1。
            // 这里检测的是整个位周期（高电平+低电平）。这里是检测两个上升沿（也就是红外灯由低变高）的时间，
            // 若使用程序在 几百微秒内翻转检测规则，即能够检测上升沿以及下降沿，容易出现问题，故未采用
            // 位0: 约560us (280+280) 最好情况，全是0位，起始信号6.75ms 72 × 560 µs 40.5 ms  合计约为47ms
            // 位1: 约1120us (280+840)最坏情况，全是1位，起始信号6.75ms 72 × 1120 µs 80ms以及结束位 ~87.67 ms
            uint32_t bit0_total = BIT_ZERO_HIGH + BIT_ZERO_LOW;  // 560us
            uint32_t bit1_total = BIT_ONE_HIGH + BIT_ONE_LOW;     // 1120us
            uint32_t tolerance = IR_PULSE_TOLERANCE_US;

            if (pulse_duration >= (bit0_total - tolerance) &&
                pulse_duration <= (bit0_total + tolerance)) {
                // 位0不需要设置，缓冲区已初始化为0
                bit_index++;
            }
            else if (pulse_duration >= (bit1_total - tolerance) &&
                     pulse_duration <= (bit1_total + tolerance)) {
                received_data[bit_index / 8] |= (1 << (7 - (bit_index % 8)));
                bit_index++;
            }
            else if (pulse_duration > IR_MAX_PULSE_US) {
                receiving = 0;
                bit_index = 0;
                last_capture_time = capture_value;
                return;
            }

            // 接收完成
            if (bit_index >= 72) {
                receiving = 0;
                IR_ReceiveData();
                last_capture_time = 0;
                return;
            }
        }
    }
    last_capture_time = capture_value;
}

// 红外接收完成标志（供主循环查询）
volatile uint8_t ir_rx_complete_flag = 0;
volatile uint8_t ir_ack_received_flag = 0; // ACK接收标志
volatile uint8_t ir_ack_status = 0;        // ACK状态: 0=无, 1=ACK, 2=NACK

void IR_ReceiveData(void) {
    // 检查是否是ACK帧
    if (received_data[0] == IR_ACK_MAGIC && received_data[1] == IR_ACK_MAGIC) {
        ir_ack_status = 1;  // 收到ACK
        ir_ack_received_flag = 1;
        return;
    }
    // 检查是否是NACK帧
    if (received_data[0] == IR_NACK_MAGIC && received_data[1] == IR_NACK_MAGIC) {
        ir_ack_status = 2;  // 收到NACK
        ir_ack_received_flag = 1;
        return;
    }
    // 普通数据帧，设置接收完成标志
    ir_rx_complete_flag = 1;
}

void IR_ResetBuffer(void)
{
    memset(received_data, 0, 9);
    bit_index = 0;
    receiving = 0;
}

bool IR_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry)
{
    if (length > 8) return false;
    
    for (uint8_t retry = 0; retry < max_retry; retry++) {
        while (IR_IsTXBusy()) {
            HAL_Delay(1);
        }
        
        if (IR_SendData(data, length)) {
            while (IR_IsTXBusy()) {
                HAL_Delay(1);
            }
            HAL_Delay(10);
            return true;
        }
        
        HAL_Delay(5);
    }
    
    return false;
}

// 发送ACK帧
void IR_SendAck(uint8_t status)
{
    uint8_t ack_frame[2];
    if (status == 1) {
        ack_frame[0] = IR_ACK_MAGIC;
        ack_frame[1] = IR_ACK_MAGIC;
    } else {
        ack_frame[0] = IR_NACK_MAGIC;
        ack_frame[1] = IR_NACK_MAGIC;
    }
    IR_SendDataWithRetry(ack_frame, 2, 2);
}

// 带ACK确认的数据发送（推荐用于双向通信）
bool IR_SendDataAndWaitAck(uint8_t *data, uint8_t length, uint8_t max_retry)
{
    if (length > 8) return false;
    
    for (uint8_t retry = 0; retry < max_retry; retry++) {
        // 清除ACK标志
        ir_ack_received_flag = 0;
        ir_ack_status = 0;
        
        // 等待发送空闲
        while (IR_IsTXBusy()) {
            HAL_Delay(1);
        }
        
        // 发送数据
        if (!IR_SendData(data, length)) {
            HAL_Delay(5);
            continue;
        }
        
        // 等待发送完成
        while (IR_IsTXBusy()) {
            HAL_Delay(1);
        }
        
        // 等待ACK（超时时间100ms）
        uint32_t start_time = HAL_GetTick();
        while ((HAL_GetTick() - start_time) < IR_ACK_TIMEOUT_MS) {
            if (ir_ack_received_flag) {
                if (ir_ack_status == 1) {
                    return true;  // 收到ACK，发送成功
                } else {
                    break;  // 收到NACK，需要重传
                }
            }
            HAL_Delay(1);
        }
        
        // 超时或未收到ACK，等待后重试
        HAL_Delay(10);
    }
    
    return false;  // 重传次数用完，发送失败
}

// 处理接收到的数据帧（在main循环中调用）
void IR_ProcessReceivedFrame(uint8_t *data, uint8_t length)
{
    // 计算并验证CRC
    uint8_t received_crc = data[length - 1];
    uint8_t calculated_crc = IR_CRC8(data, length - 1);
    
    if (received_crc == calculated_crc) {
        // CRC正确，发送ACK
        IR_SendAck(1);
        // TODO: 在这里处理接收到的数据
    } else {
        // CRC错误，发送NACK
        IR_SendAck(2);
    }
}

void IR_CheckRxTimeout(void)
{
    if (receiving && (HAL_GetTick() - rx_last_activity_time > IR_RX_TIMEOUT_MS)) {
        receiving = 0;
        bit_index = 0;
        last_capture_time = 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        IR_TX_TimerCallback(htim);
    }
}
