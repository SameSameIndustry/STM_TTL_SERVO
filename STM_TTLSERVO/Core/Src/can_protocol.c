#include "can_protocol.h"
#include "feetech_sts.h"
#include "ws2812.h"

/* Fixed indicator color per command, shown on all 6 LEDs. Kept well under
 * 255 to limit the WS2812B chain's peak current draw. */
#define LED_LEVEL         40u
#define LED_COLOR_RESET_R LED_LEVEL
#define LED_COLOR_RESET_G LED_LEVEL
#define LED_COLOR_RESET_B LED_LEVEL   /* white */
#define LED_COLOR_TARGET_R   0u
#define LED_COLOR_TARGET_G   0u
#define LED_COLOR_TARGET_B LED_LEVEL  /* blue */
#define LED_COLOR_OPEN_R      0u
#define LED_COLOR_OPEN_G  LED_LEVEL
#define LED_COLOR_OPEN_B      0u      /* green */
#define LED_COLOR_CLOSE_R LED_LEVEL
#define LED_COLOR_CLOSE_G     0u
#define LED_COLOR_CLOSE_B     0u      /* red */

/* ---- Wire format (see /CAN_PROTOCOL.md) -------------------------------- */
#define CAN_CMD_RESET       0x00u  /* no payload */
#define CAN_CMD_SET_TARGET  0x01u  /* Data[1:2] = uint16 LE, absolute 0-4095 */
#define CAN_CMD_OPEN        0x02u  /* custom: target += +50 deg, no payload */
#define CAN_CMD_CLOSE       0x03u  /* custom: target += -25 deg, no payload */

/* Position domain: 0-4095 counts over one mechanical turn - identical to
 * both the AS5600 encoder this protocol was originally written against and
 * the STS3215's own Present/Goal_Position registers, so no unit conversion
 * is needed here. */
#define CAN_POS_MAX             4095
#define CAN_POS_DEFAULT         2048
#define CAN_OPEN_DELTA_COUNTS   ((int32_t)(50 * 4096 / 360))   /* +50 deg */
#define CAN_CLOSE_DELTA_COUNTS  (-(int32_t)(25 * 4096 / 360))  /* -25 deg */
#define CAN_MOVE_TIME_MS        500u

#define CAN_QUEUE_LEN  8u

typedef struct {
    uint8_t  cmd;
    uint16_t payload;
} CAN_Command_t;

static uint8_t s_servo_id;
static int32_t s_target_counts = CAN_POS_DEFAULT;

/* Single-producer (ISR) / single-consumer (main loop) ring buffer - no
 * extra locking needed since index updates are single-byte and each side
 * only ever writes its own index. */
static volatile CAN_Command_t s_queue[CAN_QUEUE_LEN];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;

static void CAN_Enqueue(uint8_t cmd, uint16_t payload)
{
    uint8_t next_head = (uint8_t)((s_head + 1u) % CAN_QUEUE_LEN);
    if (next_head == s_tail) {
        return; /* queue full: drop rather than clobber an unread command */
    }
    s_queue[s_head].cmd = cmd;
    s_queue[s_head].payload = payload;
    s_head = next_head;
}

static void CAN_ApplyTarget(void)
{
    if (s_target_counts < 0) {
        s_target_counts = 0;
    } else if (s_target_counts > CAN_POS_MAX) {
        s_target_counts = CAN_POS_MAX;
    }
    STS_WritePosition(s_servo_id, (uint16_t)s_target_counts, CAN_MOVE_TIME_MS, 0u);
}

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, uint8_t servo_id)
{
    FDCAN_FilterTypeDef sFilterConfig;
    uint16_t present_pos;

    s_servo_id = servo_id;
    s_head = 0u;
    s_tail = 0u;

    /* Seed the tracked target from wherever the servo actually is right
     * now, so the first OPEN/CLOSE after boot is relative to reality
     * instead of an arbitrary hardcoded default. */
    s_target_counts = STS_ReadPosition(servo_id, &present_pos) ? (int32_t)present_pos : CAN_POS_DEFAULT;

    /* Accept only standard-ID frames matching CAN_NODE_ID exactly. */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_NODE_ID;
    sFilterConfig.FilterID2 = 0x7FFu; /* mask: all 11 ID bits must match */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Reject anything not matched by the filter above (HAL's default is to
     * accept unmatched frames too), and reject remote frames outright. */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void CAN_Process(void)
{
    while (s_tail != s_head) {
        CAN_Command_t cmd = s_queue[s_tail];
        s_tail = (uint8_t)((s_tail + 1u) % CAN_QUEUE_LEN);

        switch (cmd.cmd) {
        case CAN_CMD_RESET: {
            uint16_t pos;
            if (STS_ReadPosition(s_servo_id, &pos)) {
                s_target_counts = (int32_t)pos;
            }
            CAN_ApplyTarget();
            WS2812_SetAll(LED_COLOR_RESET_R, LED_COLOR_RESET_G, LED_COLOR_RESET_B);
            break;
        }
        case CAN_CMD_SET_TARGET:
            s_target_counts = (int32_t)cmd.payload;
            CAN_ApplyTarget();
            WS2812_SetAll(LED_COLOR_TARGET_R, LED_COLOR_TARGET_G, LED_COLOR_TARGET_B);
            break;
        case CAN_CMD_OPEN:
            s_target_counts += CAN_OPEN_DELTA_COUNTS;
            CAN_ApplyTarget();
            WS2812_SetAll(LED_COLOR_OPEN_R, LED_COLOR_OPEN_G, LED_COLOR_OPEN_B);
            break;
        case CAN_CMD_CLOSE:
            s_target_counts += CAN_CLOSE_DELTA_COUNTS;
            CAN_ApplyTarget();
            WS2812_SetAll(LED_COLOR_CLOSE_R, LED_COLOR_CLOSE_G, LED_COLOR_CLOSE_B);
            break;
        default:
            break; /* unknown command: ignore */
        }
    }
}

/**
  * @brief FDCAN RX FIFO0 callback - runs in interrupt context.
  * Only parses and enqueues; never touches USART1 (see file header).
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    uint8_t cmd;
    uint16_t payload = 0u;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        return;
    }
    /* Classic CAN remote frames carry no data; the hardware filter already
     * restricts us to CAN_NODE_ID, but a remote frame could still match it. */
    if (RxHeader.RxFrameType != FDCAN_DATA_FRAME) {
        return;
    }
    if (RxHeader.DataLength == FDCAN_DLC_BYTES_0) {
        return; /* Data[0] (the command byte) is mandatory */
    }

    cmd = RxData[0];
    if (cmd == CAN_CMD_SET_TARGET && RxHeader.DataLength >= FDCAN_DLC_BYTES_3) {
        payload = (uint16_t)(RxData[1] | ((uint16_t)RxData[2] << 8));
    }
    CAN_Enqueue(cmd, payload);
}
