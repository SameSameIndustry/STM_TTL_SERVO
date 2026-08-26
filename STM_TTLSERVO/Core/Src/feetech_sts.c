#include "feetech_sts.h"

#define STS_HEADER          0xFFu

#define STS_INST_PING        0x01u
#define STS_INST_READ        0x02u
#define STS_INST_WRITE       0x03u

/* SMS/STS control table (SRAM area - no EEPROM lock/unlock needed). */
#define STS_ADDR_TORQUE_ENABLE  40u
#define STS_ADDR_GOAL_POSITION  42u
#define STS_ADDR_GOAL_TIME      44u
#define STS_ADDR_GOAL_SPEED     46u
#define STS_ADDR_TORQUE_LIMIT   48u  /* RAM copy; 0-1000 = 0-100% of stall torque */
#define STS_ADDR_MODE           33u  /* EEPROM; 0=position, 1=wheel, 2=PWM open-loop */
#define STS_ADDR_PRESENT_POSITION 56u

static UART_HandleTypeDef *s_huart;
static GPIO_TypeDef *s_de_port;
static uint16_t s_de_pin;

static uint8_t STS_Checksum(const uint8_t *buf, uint8_t len)
{
    /* Sum covers ID, Length, Instruction and Parameters - i.e. everything
     * after the two 0xFF header bytes and before the checksum byte itself. */
    uint16_t sum = 0;
    for (uint8_t i = 2; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}

static void STS_SendPacket(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[16];
    uint8_t body_len = (uint8_t)(5u + param_len); /* header x2, id, length, instruction, params */

    buf[0] = STS_HEADER;
    buf[1] = STS_HEADER;
    buf[2] = id;
    buf[3] = (uint8_t)(param_len + 2u); /* instruction + params + checksum */
    buf[4] = instruction;
    for (uint8_t i = 0; i < param_len; i++) {
        buf[5 + i] = params[i];
    }
    buf[body_len] = STS_Checksum(buf, body_len);

    /* Driver enabled + receiver disabled (DE and ~RE share PA7), so the
     * echo of our own transmission never reaches USART1_RX. */
    HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_SET);
    HAL_UART_Transmit(s_huart, buf, (uint16_t)(body_len + 1u), 20u);
    /* Wait for the shift register to actually finish (incl. stop bit)
     * before releasing the bus, or the last byte gets clipped. */
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET) { }
    HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_RESET);
}

void STS_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin)
{
    s_huart = huart;
    s_de_port = de_port;
    s_de_pin = de_pin;
    HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_RESET); /* idle = listen */
}

uint8_t STS_Ping(uint8_t id)
{
    uint8_t resp[6];

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    STS_SendPacket(id, STS_INST_PING, NULL, 0);

    if (HAL_UART_Receive(s_huart, resp, sizeof(resp), 20u) == HAL_OK) {
        if (resp[0] == STS_HEADER && resp[1] == STS_HEADER && resp[2] == id) {
            return 1u;
        }
    }
    return 0u;
}

void STS_TorqueEnable(uint8_t id, uint8_t enable)
{
    uint8_t params[2];
    params[0] = STS_ADDR_TORQUE_ENABLE;
    params[1] = enable ? 1u : 0u;
    STS_SendPacket(id, STS_INST_WRITE, params, sizeof(params));
}

uint8_t STS_ReadWord(uint8_t id, uint8_t addr, uint16_t *out)
{
    uint8_t params[2];
    uint8_t resp[8]; /* FF FF ID LEN ERROR PARAM_L PARAM_H CHECKSUM */

    params[0] = addr;
    params[1] = 2u; /* read length */

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    STS_SendPacket(id, STS_INST_READ, params, sizeof(params));

    if (HAL_UART_Receive(s_huart, resp, sizeof(resp), 20u) == HAL_OK) {
        if (resp[0] == STS_HEADER && resp[1] == STS_HEADER && resp[2] == id) {
            *out = (uint16_t)(resp[5] | ((uint16_t)resp[6] << 8));
            return 1u;
        }
    }
    return 0u;
}

void STS_SetTorqueLimit(uint8_t id, uint16_t limit_permille)
{
    uint8_t params[3];

    if (limit_permille > 1000u) {
        limit_permille = 1000u;
    }

    params[0] = STS_ADDR_TORQUE_LIMIT;
    params[1] = (uint8_t)(limit_permille & 0xFFu);
    params[2] = (uint8_t)(limit_permille >> 8);
    STS_SendPacket(id, STS_INST_WRITE, params, sizeof(params));
}

uint8_t STS_ReadPosition(uint8_t id, uint16_t *out)
{
    return STS_ReadWord(id, STS_ADDR_PRESENT_POSITION, out);
}

void STS_SetMode(uint8_t id, uint8_t mode)
{
    uint8_t params[2];

    /* The official SCServo SDK writes Mode directly with no EEPROM
     * unlock/lock cycle (see e.g. WheelMode()), so Mode is apparently not
     * gated by the Lock register in practice. */
    STS_TorqueEnable(id, 0u);

    params[0] = STS_ADDR_MODE;
    params[1] = mode;
    STS_SendPacket(id, STS_INST_WRITE, params, sizeof(params));
}

void STS_WritePWM(uint8_t id, int16_t duty_permille)
{
    uint8_t params[3];
    uint16_t raw;

    if (duty_permille > 1000) {
        duty_permille = 1000;
    } else if (duty_permille < -1000) {
        duty_permille = -1000;
    }

    /* In PWM/open-loop mode (Mode 2), the duty command is written to the
     * Goal_Time register (addr 44), NOT Goal_Position - confirmed against
     * the official SCServo SDK's WritePwm(). Direction flag is bit10
     * (0x0400), magnitude in bits0-9, unlike the bit15 convention used
     * elsewhere (e.g. Goal_Speed). */
    raw = (duty_permille < 0) ? ((uint16_t)(-duty_permille) | 0x0400u) : (uint16_t)duty_permille;

    params[0] = STS_ADDR_GOAL_TIME;
    params[1] = (uint8_t)(raw & 0xFFu);
    params[2] = (uint8_t)(raw >> 8);
    STS_SendPacket(id, STS_INST_WRITE, params, sizeof(params));
}

void STS_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    uint8_t params[7];
    params[0] = STS_ADDR_GOAL_POSITION;
    params[1] = (uint8_t)(position & 0xFFu);
    params[2] = (uint8_t)(position >> 8);
    params[3] = (uint8_t)(time_ms & 0xFFu);
    params[4] = (uint8_t)(time_ms >> 8);
    params[5] = (uint8_t)(speed & 0xFFu);
    params[6] = (uint8_t)(speed >> 8);
    STS_SendPacket(id, STS_INST_WRITE, params, sizeof(params));
}
