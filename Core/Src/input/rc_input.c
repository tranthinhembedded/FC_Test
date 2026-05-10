#include "main.h"
#include "input/rc_input.h"
#include "sensor/sensor_common.h"
#include "platform/usart.h"

/* iBUS protocol definitions */
#define IBUS_DMA_BUFFER_SIZE 512U
#define IBUS_FRAME_SIZE 32U
#define IBUS_FRAME_LEN 0x20U
#define IBUS_COMMAND_SERVO 0x40U
#define RC_LINK_TIMEOUT_MS 500U
#define RC_CHANNEL_MIN_VALID 900U
#define RC_CHANNEL_MAX_VALID 2100U
#define RC_SWITCH_LOW_US 1300U
#define RC_SWITCH_HIGH_US 1700U
#define RC_SWITCH_DEBOUNCE_FRAMES 5U
#define RC_SMOOTHING_CUTOFF_HZ 15.0f

typedef struct {
    float state;
    float rc;
} pt1Filter_t;

static void pt1FilterInit(pt1Filter_t *filter, float f_cut) {
    filter->rc = 1.0f / (2.0f * 3.1415926535f * f_cut);
}

static float pt1FilterApply(pt1Filter_t *filter, float input, float dt) {
    float k = dt / (filter->rc + dt);
    filter->state += k * (input - filter->state);
    return filter->state;
}

typedef struct {
    uint32_t stable_value;
    uint32_t candidate_value;
    uint8_t candidate_count;
} RcSwitchDebounce_t;

static pt1Filter_t rc_filter_roll;
static pt1Filter_t rc_filter_pitch;
static pt1Filter_t rc_filter_throttle;
static pt1Filter_t rc_filter_yaw;

static uint8_t ibus_dma_buf[IBUS_DMA_BUFFER_SIZE];
static uint16_t ibus_old_pos = 0;

static uint8_t ibus_state_idx = 0;
static uint8_t ibus_frame_buf[32];

volatile uint32_t rc_last_frame_tick_ms = 0U;
volatile uint32_t rc_valid_frame_count = 0U;
volatile uint8_t rc_link_ok = 0U;
volatile uint32_t rc_link_drop_count = 0U;
volatile uint32_t rc_uart_error_count = 0U;
volatile uint32_t rc_last_error_tick_ms = 0U;
volatile uint32_t rc_last_timeout_tick_ms = 0U;
volatile uint32_t rc_dma_pos = 0U;
volatile uint32_t rc_byte_count = 0U;
volatile uint32_t rc_frame_sync_count = 0U;
volatile uint32_t rc_checksum_error_count = 0U;
volatile uint32_t rc_bad_header_count = 0U;
volatile uint32_t rc_dma_start_error_count = 0U;
volatile uint32_t rc_channel_range_error_count = 0U;
volatile uint32_t rc_max_pending_bytes = 0U;
volatile uint32_t rc_raw_sw_arm = 1000U;
volatile uint32_t rc_raw_sw_mode = 1000U;
volatile uint32_t rc_raw_sw_poshold = 1000U;
volatile uint32_t rc_switch_glitch_count = 0U;
volatile uint32_t rc_arm_switch_change_count = 0U;
volatile uint32_t rc_mode_switch_change_count = 0U;
volatile uint32_t rc_poshold_switch_change_count = 0U;
volatile uint32_t rc_failsafe_apply_count = 0U;
volatile uint8_t rc_arm_switch_candidate_count = 0U;
volatile uint8_t rc_mode_switch_candidate_count = 0U;
volatile uint8_t rc_poshold_switch_candidate_count = 0U;
volatile uint8_t rc_dma_running = 0U;

static RcSwitchDebounce_t s_arm_switch = {1000U, 1000U, 0U};
static RcSwitchDebounce_t s_mode_switch = {1000U, 1000U, 0U};
static RcSwitchDebounce_t s_poshold_switch = {1000U, 1000U, 0U};

static void RcReceiver_ResetSwitchDebounce(uint32_t arm_value, uint32_t mode_value, uint32_t poshold_value)
{
    s_arm_switch.stable_value = arm_value;
    s_arm_switch.candidate_value = arm_value;
    s_arm_switch.candidate_count = 0U;
    s_mode_switch.stable_value = mode_value;
    s_mode_switch.candidate_value = mode_value;
    s_mode_switch.candidate_count = 0U;
    s_poshold_switch.stable_value = poshold_value;
    s_poshold_switch.candidate_value = poshold_value;
    s_poshold_switch.candidate_count = 0U;
    rc_arm_switch_candidate_count = 0U;
    rc_mode_switch_candidate_count = 0U;
    rc_poshold_switch_candidate_count = 0U;
}

static void RcReceiver_ApplyFailsafeValues(void)
{
    RC_Raw_Roll = 1500U;
    RC_Raw_Pitch = 1500U;
    RC_Raw_Yaw = 1500U;
    RC_Raw_Throttle = 1000U;
    RC_Raw_SW_Arm = 1000U;
    RC_Raw_SW_Mode = 1000U;
    RC_Raw_SW_PosHold = 1000U;
    rc_raw_sw_arm = 1000U;
    rc_raw_sw_mode = 1000U;
    rc_raw_sw_poshold = 1000U;
    rc_filter_roll.state = 1500.0f;
    rc_filter_pitch.state = 1500.0f;
    rc_filter_yaw.state = 1500.0f;
    rc_filter_throttle.state = 1000.0f;
    RcReceiver_ResetSwitchDebounce(1000U, 1000U, 1000U);
}

static void RcReceiver_ApplyLinkLossFailsafeValues(void)
{
    RC_Raw_Roll = 1500U;
    RC_Raw_Pitch = 1500U;
    RC_Raw_Yaw = 1500U;
    RC_Raw_Throttle = 1000U;
    RC_Raw_SW_Arm = 1000U;
    RC_Raw_SW_Mode = 1000U;
    RC_Raw_SW_PosHold = 1000U;
    rc_filter_roll.state = 1500.0f;
    rc_filter_pitch.state = 1500.0f;
    rc_filter_yaw.state = 1500.0f;
    rc_filter_throttle.state = 1000.0f;
    rc_failsafe_apply_count++;
}

static uint8_t RcReceiver_ChannelInRange(uint32_t value)
{
    return (uint8_t)((value >= RC_CHANNEL_MIN_VALID) && (value <= RC_CHANNEL_MAX_VALID));
}

static uint32_t RcReceiver_NormalizeSwitch(uint32_t value, uint32_t previous)
{
    if (value >= RC_SWITCH_HIGH_US) {
        return 2000U;
    }
    if (value <= RC_SWITCH_LOW_US) {
        return 1000U;
    }
    return previous;
}

static uint32_t RcReceiver_DebounceSwitch(RcSwitchDebounce_t *sw,
                                          uint32_t raw_value,
                                          volatile uint32_t *change_count)
{
    uint32_t normalized = RcReceiver_NormalizeSwitch(raw_value, sw->stable_value);

    if (normalized == sw->stable_value) {
        sw->candidate_value = normalized;
        sw->candidate_count = 0U;
        return sw->stable_value;
    }

    if (normalized != sw->candidate_value) {
        sw->candidate_value = normalized;
        sw->candidate_count = 1U;
        rc_switch_glitch_count++;
        return sw->stable_value;
    }

    if (sw->candidate_count < RC_SWITCH_DEBOUNCE_FRAMES) {
        sw->candidate_count++;
    }

    if (sw->candidate_count < RC_SWITCH_DEBOUNCE_FRAMES) {
        rc_switch_glitch_count++;
        return sw->stable_value;
    }

    sw->stable_value = normalized;
    sw->candidate_count = 0U;
    (*change_count)++;
    return sw->stable_value;
}

void RcReceiver_Init(void)
{
    HAL_StatusTypeDef dma_status;

    pt1FilterInit(&rc_filter_roll, RC_SMOOTHING_CUTOFF_HZ);
    pt1FilterInit(&rc_filter_pitch, RC_SMOOTHING_CUTOFF_HZ);
    pt1FilterInit(&rc_filter_throttle, RC_SMOOTHING_CUTOFF_HZ);
    pt1FilterInit(&rc_filter_yaw, RC_SMOOTHING_CUTOFF_HZ);

    RcReceiver_ApplyFailsafeValues();

    /* Request DMA circular reception */
    ibus_old_pos = 0;
    ibus_state_idx = 0U;
    rc_last_frame_tick_ms = 0U;
    rc_valid_frame_count = 0U;
    rc_link_ok = 0U;
    rc_link_drop_count = 0U;
    rc_uart_error_count = 0U;
    rc_last_error_tick_ms = 0U;
    rc_last_timeout_tick_ms = 0U;
    rc_dma_pos = 0U;
    rc_byte_count = 0U;
    rc_frame_sync_count = 0U;
    rc_checksum_error_count = 0U;
    rc_bad_header_count = 0U;
    rc_dma_start_error_count = 0U;
    rc_channel_range_error_count = 0U;
    rc_max_pending_bytes = 0U;
    rc_raw_sw_arm = 1000U;
    rc_raw_sw_mode = 1000U;
    rc_raw_sw_poshold = 1000U;
    rc_switch_glitch_count = 0U;
    rc_arm_switch_change_count = 0U;
    rc_mode_switch_change_count = 0U;
    rc_poshold_switch_change_count = 0U;
    rc_failsafe_apply_count = 0U;
    rc_arm_switch_candidate_count = 0U;
    rc_mode_switch_candidate_count = 0U;
    rc_poshold_switch_candidate_count = 0U;
    RcReceiver_ResetSwitchDebounce(1000U, 1000U, 1000U);
    rc_dma_running = 0U;

    dma_status = HAL_UART_Receive_DMA(&huart2, ibus_dma_buf, IBUS_DMA_BUFFER_SIZE);
    if (dma_status == HAL_OK) {
        if (huart2.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);
        }
        rc_dma_running = 1U;
    } else {
        rc_dma_start_error_count++;
    }
}

/* Parse a single byte through the iBUS state machine */
static void Process_iBUS_Byte(uint8_t byte)
{
    if (ibus_state_idx == 0) {
        if (byte == IBUS_FRAME_LEN) {
            ibus_frame_buf[ibus_state_idx++] = byte;
        } else {
            rc_bad_header_count++;
        }
    } 
    else if (ibus_state_idx == 1) {
        if (byte == IBUS_COMMAND_SERVO) {
            ibus_frame_buf[ibus_state_idx++] = byte;
        } else {
            rc_bad_header_count++;
            if (byte == IBUS_FRAME_LEN) {
                ibus_frame_buf[0] = byte;
                ibus_state_idx = 1U;
            } else {
                ibus_state_idx = 0U;
            }
        }
    } 
    else {
        ibus_frame_buf[ibus_state_idx++] = byte;
        
        if (ibus_state_idx == IBUS_FRAME_SIZE) {
            /* Checksum validation */
            uint16_t checksum = 0xFFFF;
            for (int i = 0; i < 30; i++) {
                checksum -= ibus_frame_buf[i];
            }
            
            uint16_t received_checksum = (uint16_t)ibus_frame_buf[30] | ((uint16_t)ibus_frame_buf[31] << 8);

            rc_frame_sync_count++;
            
            if (checksum == received_checksum) {
                uint32_t ch_roll = (uint32_t)((uint16_t)ibus_frame_buf[2]  | ((uint16_t)ibus_frame_buf[3]  << 8));
                uint32_t ch_pitch = 3000 - (uint32_t)((uint16_t)ibus_frame_buf[4]  | ((uint16_t)ibus_frame_buf[5]  << 8));
                uint32_t ch_throttle = (uint32_t)((uint16_t)ibus_frame_buf[6]  | ((uint16_t)ibus_frame_buf[7]  << 8));
                uint32_t ch_yaw = (uint32_t)((uint16_t)ibus_frame_buf[8]  | ((uint16_t)ibus_frame_buf[9]  << 8));
                uint32_t ch_arm = (uint32_t)((uint16_t)ibus_frame_buf[10] | ((uint16_t)ibus_frame_buf[11] << 8));
                uint32_t ch_mode = (uint32_t)((uint16_t)ibus_frame_buf[12] | ((uint16_t)ibus_frame_buf[13] << 8));
                uint32_t ch_poshold = (uint32_t)((uint16_t)ibus_frame_buf[16] | ((uint16_t)ibus_frame_buf[17] << 8));

                if ((RcReceiver_ChannelInRange(ch_roll) == 0U)
                    || (RcReceiver_ChannelInRange(ch_pitch) == 0U)
                    || (RcReceiver_ChannelInRange(ch_throttle) == 0U)
                    || (RcReceiver_ChannelInRange(ch_yaw) == 0U)
                    || (RcReceiver_ChannelInRange(ch_arm) == 0U)
                    || (RcReceiver_ChannelInRange(ch_mode) == 0U)
                    || (RcReceiver_ChannelInRange(ch_poshold) == 0U)) {
                    rc_channel_range_error_count++;
                    ibus_state_idx = 0U;
                    return;
                }

                /* Valid frame received, decode channels */
                uint32_t now = HAL_GetTick();
                float dt = 0.014f;
                if (rc_last_frame_tick_ms != 0U) {
                    dt = (float)(now - rc_last_frame_tick_ms) * 0.001f;
                    if (dt <= 0.0f || dt > 0.1f) {
                        dt = 0.014f;
                    }
                }

                pt1FilterApply(&rc_filter_roll, (float)ch_roll, dt);
                pt1FilterApply(&rc_filter_pitch, (float)ch_pitch, dt);
                pt1FilterApply(&rc_filter_throttle, (float)ch_throttle, dt);
                pt1FilterApply(&rc_filter_yaw, (float)ch_yaw, dt);

                RC_Raw_Roll     = (uint32_t)rc_filter_roll.state;
                RC_Raw_Pitch    = (uint32_t)rc_filter_pitch.state;
                RC_Raw_Throttle = (uint32_t)rc_filter_throttle.state;
                RC_Raw_Yaw      = (uint32_t)rc_filter_yaw.state;
                
                rc_raw_sw_arm = ch_arm;
                rc_raw_sw_mode = ch_mode;
                rc_raw_sw_poshold = ch_poshold;
                RC_Raw_SW_Arm = RcReceiver_DebounceSwitch(&s_arm_switch, ch_arm, &rc_arm_switch_change_count);
                RC_Raw_SW_Mode = RcReceiver_DebounceSwitch(&s_mode_switch, ch_mode, &rc_mode_switch_change_count);
                RC_Raw_SW_PosHold = RcReceiver_DebounceSwitch(&s_poshold_switch, ch_poshold, &rc_poshold_switch_change_count);
                rc_arm_switch_candidate_count = s_arm_switch.candidate_count;
                rc_mode_switch_candidate_count = s_mode_switch.candidate_count;
                rc_poshold_switch_candidate_count = s_poshold_switch.candidate_count;
                rc_last_frame_tick_ms = HAL_GetTick();
                rc_valid_frame_count++;
                rc_link_ok = 1U;
            } else {
                rc_checksum_error_count++;
            }
            
            ibus_state_idx = 0; /* Reset for next frame */
        }
    }
}

/* Calculate available bytes from DMA buffer and process */
void RcReceiver_Process_DMA_Ring_Buffer(void)
{
    uint16_t pos;
    uint16_t pending;

    if (huart2.hdmarx == NULL) {
        rc_dma_running = 0U;
        return;
    }

    pos = IBUS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (pos >= IBUS_DMA_BUFFER_SIZE) {
        pos = 0U;
    }
    rc_dma_pos = pos;

    if (pos >= ibus_old_pos) {
        pending = (uint16_t)(pos - ibus_old_pos);
    } else {
        pending = (uint16_t)(IBUS_DMA_BUFFER_SIZE - ibus_old_pos + pos);
    }
    if (pending > rc_max_pending_bytes) {
        rc_max_pending_bytes = pending;
    }
    
    if (pos != ibus_old_pos) {
        if (pos > ibus_old_pos) {
            /* Data hasn't wrapped around the buffer end */
            for (uint16_t i = ibus_old_pos; i < pos; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
        } 
        else {
            /* Data wrapped around to the beginning */
            for (uint16_t i = ibus_old_pos; i < IBUS_DMA_BUFFER_SIZE; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
        }
        ibus_old_pos = pos;
    }
}

void RcReceiver_UpdateLinkStatus(uint32_t now_ms)
{
    if ((rc_last_frame_tick_ms == 0U) || ((uint32_t)(now_ms - rc_last_frame_tick_ms) > RC_LINK_TIMEOUT_MS)) {
        if (rc_link_ok != 0U) {
            rc_link_drop_count++;
            rc_last_timeout_tick_ms = now_ms;
        }
        rc_link_ok = 0U;
        RcReceiver_ApplyLinkLossFailsafeValues();
        return;
    }

    rc_link_ok = 1U;
}

void RcReceiver_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_StatusTypeDef dma_status;

        /* Clear error flags */
        volatile uint32_t temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;

        rc_uart_error_count++;
        rc_last_error_tick_ms = HAL_GetTick();
        ibus_state_idx = 0U;
        ibus_old_pos = 0U;

        /* Restart DMA */
        HAL_UART_DMAStop(&huart2);
        dma_status = HAL_UART_Receive_DMA(&huart2, ibus_dma_buf, IBUS_DMA_BUFFER_SIZE);
        if (dma_status == HAL_OK) {
            if (huart2.hdmarx != NULL) {
                __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
                __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);
            }
            rc_dma_running = 1U;
        } else {
            rc_dma_running = 0U;
            rc_dma_start_error_count++;
        }
    }
}
