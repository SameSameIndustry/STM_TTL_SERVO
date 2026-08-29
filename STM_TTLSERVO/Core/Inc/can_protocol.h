/**
 * CAN receive protocol (see /CAN_PROTOCOL.md at the repo root) driving the
 * FEETECH STS servo over feetech_sts.h. Actual servo I/O never happens
 * inside the FDCAN ISR - HAL_FDCAN_RxFifo0Callback only enqueues the
 * command; CAN_Process() (called from the main loop) does the UART work.
 */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include "main.h"

#define CAN_NODE_ID  0x21u

/* Configures the RX filter/notification for CAN_NODE_ID and starts hfdcan.
 * Call once, after MX_FDCAN1_Init() and after the servo driver (feetech_sts)
 * is initialized - this reads the servo's present position to seed the
 * firmware-tracked target used by OPEN/CLOSE. */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, uint8_t servo_id);

/* Call every iteration of the main loop: drains any commands queued by the
 * FDCAN ISR since the last call and talks to the servo over USART1. */
void CAN_Process(void);

#endif /* CAN_PROTOCOL_H */
