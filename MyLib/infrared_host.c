#include "infrared_host.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

IR_Host_Context_t ir_host_context = {0};

typedef struct {
    uint8_t tx_data[7];
    uint8_t rx_data[7];
    uint8_t rx_data_len;
    uint32_t update_timestamp;
    uint32_t test_cycle;
    uint8_t task_state;
    uint8_t online_count;
    uint8_t total_modules;

    struct {
        uint8_t discovered;
        uint8_t online;
        uint8_t busy;
        uint8_t consecutive_errors;
        uint32_t total_rx_count;
        uint32_t last_rx_time;
    } mod1, mod2;

    struct {
        uint32_t tx_success_count;
        uint32_t tx_fail_count;
        uint32_t rx_valid_count;
        uint32_t rx_invalid_count;
        uint8_t data_match;
        uint8_t data_check_result;
    } stats;

    uint8_t data_changed_flag;
} IR_Debug_Data_t;

IR_Debug_Data_t ir_debug = {0};

static TaskHandle_t ir_host_task_handle = NULL;

void IR_Host_Init(void)//初始化红外主机
{
    ir_host_context.rx_queue = xQueueCreate(IR_HOST_RX_QUEUE_SIZE, sizeof(IR_Host_RxFrame_t));
    ir_host_context.module_list.head = NULL;
    ir_host_context.module_list.count = 0;
    ir_host_context.module_list.mutex = xSemaphoreCreateMutex();
    ir_host_context.initialized = true;
    ir_host_context.task_state = IR_HOST_TASK_STATE_INIT;
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;
    ir_host_context.discovery_start_time = 0;

    IR_Host_AddModule(0x01);

    IR_Module_Node_t *module = IR_Host_FindModule(0x01);
    if (module != NULL) {
        module->last_tx_time = HAL_GetTick() - IR_HOST_FRAME_INTERVAL_MS - 1;
    }
}

void IR_Host_StartTask(void)//启动红外主机任务
{
    if (ir_host_task_handle == NULL) {
        xTaskCreate(IR_Host_Task, "IRHostTask", 1024, NULL, 3, &ir_host_task_handle);
    }
}

bool IR_Host_AddModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;
    if (ir_host_context.module_list.count >= IR_HOST_MAX_MODULES) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return false;
        }
        current = current->next;
    }

    IR_Module_Node_t *new_node = (IR_Module_Node_t *)pvPortMalloc(sizeof(IR_Module_Node_t));
    if (new_node == NULL) {
        xSemaphoreGive(ir_host_context.module_list.mutex);
        return false;
    }

    new_node->module_id = module_id;
    new_node->status = IR_HOST_STATUS_IDLE;
    new_node->online = false;
    new_node->busy = false;
    new_node->discovered = false;
    new_node->last_rx_time = 0;
    new_node->last_tx_time = 0;
    new_node->poll_fail_count = 0;
    new_node->consecutive_errors = 0;
    memset(&new_node->last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    memset(&new_node->data_cache, 0, sizeof(IR_Module_DataCache_t));
    new_node->next = ir_host_context.module_list.head;
    ir_host_context.module_list.head = new_node;
    ir_host_context.module_list.count++;

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return true;
}

bool IR_Host_RemoveModule(uint8_t module_id)//移除红外主机模块
{
    if (!ir_host_context.initialized) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t **current = &ir_host_context.module_list.head;
    while (*current != NULL) {
        if ((*current)->module_id == module_id) {
            IR_Module_Node_t *to_remove = *current;
            *current = (*current)->next;
            vPortFree(to_remove);
            ir_host_context.module_list.count--;
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return true;
        }
        current = &(*current)->next;
    }

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return false;
}

IR_Module_Node_t* IR_Host_FindModule(uint8_t module_id)//查找红外主机模块
{
    if (!ir_host_context.initialized) return NULL;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

IR_Module_Node_t* IR_Host_GetModuleByIndex(uint8_t index)//根据索引获取红外主机模块
{
    if (!ir_host_context.initialized || index >= ir_host_context.module_list.count) return NULL;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    for (uint8_t i = 0; i < index && current != NULL; i++) {
        current = current->next;
    }
    return current;
}

uint8_t IR_Host_GetModuleCount(void)//获取红外主机模块数量
{
    return ir_host_context.module_list.count;
}

bool IR_Host_IsModuleOnline(uint8_t module_id)//判断红外主机模块是否在线
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    return module->online;
}

uint8_t IR_Host_GetOnlineCount(void)//获取红外主机在线模块数量
{
    uint8_t count = 0;
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->online && current->discovered) {
            count++;
        }
        current = current->next;
    }
    return count;
}

IR_Host_TaskState_t IR_Host_GetTaskState(void)
{
    return ir_host_context.task_state;
}

void IR_Host_ForceRediscover(void)//强制重新发现红外主机模块
{
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        current->discovered = false;
        current->online = false;
        current->poll_fail_count = 0;
        current->consecutive_errors = 0;
        memset(&current->data_cache, 0, sizeof(IR_Module_DataCache_t));
        current = current->next;
    }
    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = HAL_GetTick();
}

uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length)//计算红外主机CRC8校验和
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

void IR_Host_ConfigCanFilter(void)//配置CAN过滤波
{
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
}

void IR_Host_StartCan(void)
{
    IR_Host_ConfigCanFilter();

    HAL_CAN_Start(&hcan1);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY |
                                              CAN_IT_RX_FIFO0_MSG_PENDING |
                                              CAN_IT_RX_FIFO0_FULL |
                                              CAN_IT_RX_FIFO0_OVERRUN);
}

void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan)//发送完成回调
{
    if (hcan->Instance == CAN1) {

    }
}

void IR_Host_Receive_DataFrame_Ocan(uint32_t can_id, uint8_t module_id, uint8_t *data, uint8_t dlc)//接收红外主机数据帧
{
    IR_Host_RxFrame_t rx_frame;

    rx_frame.can_id = can_id;
    rx_frame.module_id = module_id;
    rx_frame.dlc = dlc > 8 ? 8 : dlc;
    memcpy(rx_frame.data, data, rx_frame.dlc);
    rx_frame.timestamp = HAL_GetTick();

    if (ir_host_context.rx_queue != NULL) {
        xQueueSendFromISR(ir_host_context.rx_queue, &rx_frame, NULL);
    }
}

void IR_Host_Handle(uint8_t module_id, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc)//处理红外主机数据帧
{
    if (id == IR_HOST_CAN_ID_ACK) {
        if (dlc >= 2 && data[0] == IR_ACK_MAGIC && data[1] == IR_ACK_MAGIC) {
            IR_Module_Node_t *module = IR_Host_FindModule(module_id);
            if (module != NULL) {
                module->status = IR_HOST_STATUS_SUCCESS;
                module->busy = false;
                module->consecutive_errors = 0;
            }
        }
    }
}

void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)//处理红外主机数据帧
{
    uint8_t module_id = rx_data[0];

    IR_Host_Receive_DataFrame_Ocan(rx_header->StdId, module_id, rx_data, rx_header->DLC);
    IR_Host_Handle(module_id, &hcan1, rx_header->StdId, rx_data, rx_header->DLC);
}

bool IR_Host_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length)//发送红外主机命令
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    uint8_t total_len;

    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;
    if (length > 6) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    memset(tx_data, 0, 8);
    tx_data[0] = module_id;
    tx_data[1] = cmd;
    if (data != NULL && length > 0) {
        memcpy(&tx_data[2], data, length);
    }
    total_len = 2 + length;

    tx_header.StdId = IR_HOST_CAN_ID_COMMAND;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = total_len;
    tx_header.TransmitGlobalTime = DISABLE;

    module->busy = true;
    module->status = IR_HOST_STATUS_SENDING;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        module->busy = false;
        module->status = IR_HOST_STATUS_ERROR;
        return false;
    }

    module->last_tx_time = HAL_GetTick();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    return true;
}

bool IR_Host_SendDataWithRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t max_retry)//发送红外主机数据，重试次数
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    uint8_t total_len;

    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    if (length > 7) return false;

    for (uint8_t retry = 0; retry < max_retry; retry++) {
        while (module->busy) {
            vTaskDelay(pdMS_TO_TICKS(10));//等待10ms
        }

        module->status = IR_HOST_STATUS_IDLE;//设置状态为空闲

        memset(tx_data, 0, 8);//清空发送数据
        tx_data[0] = module_id;//设置模块ID
        memcpy(&tx_data[1], data, length);//复制数据
        total_len = length + 1;//设置总长度

        tx_header.StdId = IR_HOST_CAN_ID_DATA;//设置CAN ID
        tx_header.ExtId = 0;//设置扩展ID
        tx_header.IDE = CAN_ID_STD;//设置标准ID
        tx_header.RTR = CAN_RTR_DATA;//设置数据帧
        tx_header.DLC = total_len;//设置数据长度
        tx_header.TransmitGlobalTime = DISABLE;//禁用全局时间

        module->busy = true;//设置为忙状态
        module->status = IR_HOST_STATUS_SENDING;//设置状态为发送中

        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) == HAL_OK) {
            module->last_tx_time = HAL_GetTick();//设置上次发送时间

            uint32_t start_time = HAL_GetTick();//设置开始时间
            while ((HAL_GetTick() - start_time) < IR_HOST_CAN_TIMEOUT_MS) {
                if (!module->busy) {
                    if (module->status == IR_HOST_STATUS_SUCCESS) {
                        return true;
                    } else if (module->status == IR_HOST_STATUS_NACK) {
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            module->busy = false;
            module->status = IR_HOST_STATUS_ERROR;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

bool IR_Host_Ping(uint8_t module_id, uint32_t timeout_ms)//发送红外主机Ping命令
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_PING, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            if (module->status == IR_HOST_STATUS_SUCCESS) {
                return true;
            }
            if (module->status == IR_HOST_STATUS_ERROR ||
                module->status == IR_HOST_STATUS_NACK ||
                module->status == IR_HOST_STATUS_TIMEOUT) {
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

bool IR_Host_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms)//读取红外主机状态
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_READ_STATUS, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();//设置开始时间
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            if (status != NULL && module->last_response.length > 0) {
                *status = module->last_response.data[0];
            }
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));//等待10ms
    }

    module->busy = false;//设置为空闲状态
    module->status = IR_HOST_STATUS_TIMEOUT;//设置状态为超时    
    return false;
}

bool IR_Host_ResetModule(uint8_t module_id, uint32_t timeout_ms)//重置红外主机
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_RESET, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();//设置开始时间
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));//等待10ms
    }

    module->busy = false;//设置为空闲状态
    module->status = IR_HOST_STATUS_TIMEOUT;//设置状态为超时    
    return false;
}

IR_Data_CheckResult_t IR_Host_CheckDataConsistency(uint8_t module_id)//检查红外主机数据一致性
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);

    if (module == NULL) {
        return IR_DATA_CHECK_NO_MODULE;
    }

    if (!module->online || !module->discovered) {
        return IR_DATA_CHECK_OFFLINE;
    }

    if (!module->data_cache.valid) {
        return IR_DATA_CHECK_CRC_ERR;
    }

    uint32_t time_since_update = HAL_GetTick() - module->data_cache.update_timestamp;
    if (time_since_update > IR_HOST_DATA_STALE_MS) {
        return IR_DATA_CHECK_STALE;
    }

    if (module->data_cache.consistent_count < IR_HOST_CONSISTENT_COUNT) {
        return IR_DATA_CHECK_INCONSISTENT;
    }

    return IR_DATA_CHECK_OK;
}

IR_Data_CheckResult_t IR_Host_CheckAllModules(void)//检查所有红外主机数据一致性
{
    IR_Data_CheckResult_t worst_result = IR_DATA_CHECK_OK;
    bool any_module_found = false;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        any_module_found = true;
        IR_Data_CheckResult_t result = IR_Host_CheckDataConsistency(current->module_id);
        if (result > worst_result) {
            worst_result = result;
        }
        current = current->next;
    }

    if (!any_module_found) {
        return IR_DATA_CHECK_NO_MODULE;
    }

    return worst_result;
}

bool IR_Host_GetModuleData(uint8_t module_id, uint8_t *data, uint8_t *length)//获取红外主机数据
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);

    if (module == NULL || !module->data_cache.valid) {
        return false;
    }

    if (data != NULL) {
        memcpy(data, module->data_cache.raw_data, 7);
    }
    if (length != NULL) {
        *length = module->last_response.length > 7 ? 7 : module->last_response.length;
    }

    return true;
}

static void IR_Host_DiscoveryPhase(void)//红外主机发现阶段
{
    static uint8_t discover_index = 0;
    uint32_t now = HAL_GetTick();//设置当前时间

    if (now - ir_host_context.discovery_start_time > 5000) {
        ir_host_context.task_state = IR_HOST_TASK_STATE_RUNNING;
        discover_index = 0;
        return;
    }

    IR_Module_Node_t *module = IR_Host_GetModuleByIndex(discover_index);
    if (module != NULL && !module->discovered) {
        if (!module->busy && (now - module->last_tx_time) > IR_HOST_POLL_INTERVAL_MS) {
            if (IR_Host_Ping(module->module_id, 200)) {
                module->discovered = true;//设置发现状态为已发现
                module->online = true;//设置在线状态为在线
            } else {
                module->poll_fail_count++;
            }
        }
    }

    discover_index++;
    if (discover_index >= ir_host_context.module_list.count) {
        discover_index = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

static void IR_Host_PollModule(IR_Module_Node_t *module)//轮询红外主机
{
    if (module == NULL) return;

    if (!module->busy) {
        uint32_t now = HAL_GetTick();//设置当前时间

        if (!module->online || !module->discovered) {
            if (now - module->last_tx_time > IR_HOST_POLL_INTERVAL_MS * 2) {
                if (IR_Host_Ping(module->module_id, 300)) {
                    module->discovered = true;//设置发现状态为已发现
                    module->online = true;//设置在线状态为在线
                    } else {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 10) {
                        module->online = false;//模块离线
                        module->discovered = false;//模块未发现
                        module->data_cache.valid = false;//数据缓存无效
                    }
                }
            }
        } else {
            if (now - module->last_tx_time >= IR_HOST_POLL_INTERVAL_MS) {
                bool read_ok = IR_Host_ReadStatus(module->module_id, NULL, 300);
                if (read_ok) {
                    module->consecutive_errors = 0;
                } else {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 5) {
                        module->online = false;
                        module->data_cache.valid = false;
                    }
                }
            }
        }
    }
}

static void IR_Host_UpdateOnlineStatus(void)//更新红外主机在线状态
{
    uint32_t now = HAL_GetTick();//当前时间
    IR_Module_Node_t *current = ir_host_context.module_list.head;//当前模块节点

    while (current != NULL) {
        if (current->online &&
            (now - current->last_rx_time) > IR_HOST_ONLINE_TIMEOUT_MS) {
            current->online = false;//模块离线
            current->data_cache.valid = false;//数据缓存无效
        }

        if (current->data_cache.valid &&
            (now - current->data_cache.update_timestamp) > IR_HOST_DATA_STALE_MS * 3) {
            current->data_cache.valid = false;
        }

        current = current->next;//下一个模块节点
    }
}

void IR_Host_Task(void *argument)//红外主机任务
{
    IR_Host_RxFrame_t rx_frame;
    (void)argument;

    // 初始化红外主机任务状态
    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = HAL_GetTick();//设置当前时间
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;

    
    for (;;) {
        while (xQueueReceive(ir_host_context.rx_queue, &rx_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (rx_frame.can_id == IR_HOST_CAN_ID_DATA) {
                IR_Module_Node_t *module = IR_Host_FindModule(rx_frame.module_id);//查找模块


                if (module != NULL && rx_frame.dlc >= 2) {
                    uint8_t data_len = rx_frame.dlc - 1;//数据长度（去掉module_id）

                    module->data_cache.total_rx_count++;//增加总接收数据包数

                    if (data_len > 0) {
                        bool data_changed = false;//数据是否改变

                        module->data_cache.total_rx_count++;//增加总接收数据包数

                        if (module->data_cache.valid &&
                            data_len <= 7 &&
                            memcmp(module->data_cache.raw_data,
                                   &rx_frame.data[1],
                                   data_len) != 0) {
                            data_changed = true;//数据改变
                        }

                        memcpy(module->data_cache.raw_data,
                               &rx_frame.data[1],
                               data_len > 7 ? 7 : data_len);//复制数据
                        module->data_cache.update_timestamp = rx_frame.timestamp;//更新时间戳
                        module->data_cache.valid = true;//数据缓存有效

                        if (!data_changed) {
                            if (module->data_cache.consistent_count < 255) {
                                module->data_cache.consistent_count++;//增加一致计数
                            }
                        } else {
                            module->data_cache.consistent_count = 1;
                        }

                        module->last_response.module_id = rx_frame.data[0];//设置模块ID
                        module->last_response.status = IR_HOST_STATUS_SUCCESS;//设置状态为成功
                        module->last_response.length = data_len;//设置长度为数据长度
                        module->last_response.timestamp = rx_frame.timestamp;//设置时间戳
                        module->last_response.valid = true;//设置有效为有效

                        if (data_len > 0) {
                            memcpy(module->last_response.data,
                                   &rx_frame.data[1],
                                   data_len);
                        }

                        module->last_rx_time = rx_frame.timestamp;//设置最后接收时间
                        module->online = true;//模块在线
                        module->busy = false;//模块不忙
                        module->discovered = true;//模块已发现
                        module->poll_fail_count = 0;//轮询失败次数重置
                    }
                }
            }
        }

        switch (ir_host_context.task_state) {
            case IR_HOST_TASK_STATE_DISCOVERY:
                IR_Host_DiscoveryPhase();
                break;

            case IR_HOST_TASK_STATE_RUNNING:
            {
                uint32_t now = HAL_GetTick();//当前时间

                if (now - ir_host_context.last_poll_time >= IR_HOST_POLL_INTERVAL_MS) {
                    ir_host_context.last_poll_time = now;//最后轮询时间

                    IR_Module_Node_t *module = IR_Host_GetModuleByIndex(
                        ir_host_context.current_poll_index);//当前轮询模块

                    if (module != NULL) {
                        IR_Host_PollModule(module);//轮询模块
                    }

                    ir_host_context.current_poll_index++;
                    if (ir_host_context.current_poll_index >=
                        ir_host_context.module_list.count) {
                        ir_host_context.current_poll_index = 0;//当前轮询模块索引重置
                        IR_Host_UpdateOnlineStatus();//更新在线状态
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                break;
            }

            case IR_HOST_TASK_STATE_ERROR:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            default:
                ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;//任务状态重置
                ir_host_context.discovery_start_time = HAL_GetTick();//发现开始时间
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static TaskHandle_t ir_test_task_handle = NULL;

void IR_Test_Task(void *argument)
{
    (void)argument;
    uint32_t last_send_time = 0;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    IR_Host_StartCan();
    vTaskDelay(100);

    IR_Host_Init();
    IR_Host_AddModule(0x01);
    vTaskDelay(100);

    IR_Host_StartTask();

    tx_header.StdId = IR_HOST_CAN_ID_DATA;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;

    for (;;) {
        ir_debug.test_cycle++;

        ir_debug.task_state = (uint8_t)IR_Host_GetTaskState();
        ir_debug.online_count = IR_Host_GetOnlineCount();
        ir_debug.total_modules = IR_Host_GetModuleCount();

        IR_Module_Node_t *mod1 = IR_Host_FindModule(0x01);
        if (mod1 != NULL) {
            ir_debug.mod1.discovered = mod1->discovered ? 1 : 0;
            ir_debug.mod1.online = mod1->online ? 1 : 0;
            ir_debug.mod1.busy = mod1->busy ? 1 : 0;
            ir_debug.mod1.consecutive_errors = mod1->consecutive_errors;
            ir_debug.mod1.total_rx_count = mod1->data_cache.total_rx_count;
            ir_debug.mod1.last_rx_time = mod1->last_rx_time;
        }

        if (ir_debug.task_state == IR_HOST_TASK_STATE_RUNNING && ir_debug.online_count >= 1) {
            uint32_t now = HAL_GetTick();
            if (now - last_send_time >= 5000) {
                last_send_time = now;

                ir_debug.tx_data[0] = (uint8_t)(ir_debug.test_cycle & 0xFF);
                ir_debug.tx_data[1] = (uint8_t)((ir_debug.test_cycle >> 8) & 0xFF);
                ir_debug.tx_data[2] = 0xCC;
                ir_debug.tx_data[3] = 0xDD;
                ir_debug.tx_data[4] = 0xEE;
                ir_debug.tx_data[5] = 0xFF;
                ir_debug.tx_data[6] = 0x11;

                uint8_t can_tx_data[8];
                can_tx_data[0] = 0x01;
                memcpy(&can_tx_data[1], ir_debug.tx_data, 7);
                tx_header.DLC = 8;

                if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, can_tx_data, &tx_mailbox) == HAL_OK) {
                    ir_debug.stats.tx_success_count++;
                    ir_debug.data_changed_flag = 1;
                    ir_debug.update_timestamp = HAL_GetTick();
                } else {
                    ir_debug.stats.tx_fail_count++;
                    ir_debug.data_changed_flag = 0;
                }
            }
        }

        if (ir_debug.test_cycle % 100 == 0 && ir_debug.task_state == IR_HOST_TASK_STATE_DISCOVERY) {
            IR_Host_ForceRediscover();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void IR_Test_StartTask(void)
{
    if (ir_test_task_handle == NULL) {
        xTaskCreate(IR_Test_Task, "IR_Test_Task", 512, NULL, 4, &ir_test_task_handle);
    }
}
