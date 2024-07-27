#include "isotp_user.h"
#include "isotp_defines.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"

#include <stdio.h>

CAN_HandleTypeDef *isotp_user_can_handle = NULL;

/* optional, provide to receive debugging log messages */
void isotp_user_debug(const char *message, ...)
{
  // printf("ISOTP_DEBUG: %s", message);
}

/* required, this must send a single CAN message with the given arbitration
  * ID (i.e. the CAN message ID) and data. The size will never be more than 8
  * bytes. Should return ISOTP_RET_OK if frame sent successfully.
  * May return ISOTP_RET_NOSPACE if the frame could not be sent but may be
  * retried later. Should return ISOTP_RET_ERROR in case frame could not be sent.
  */
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint8_t size)
{
  if (isotp_user_can_handle == NULL)
  {
    isotp_user_can_handle = &hcan1;
  }

  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t TxData[8];
  uint8_t i;

  TxHeader.StdId = arbitration_id;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = size;
  TxHeader.TransmitGlobalTime = DISABLE;

  for (i = 0; i < size; i++)
  {
    TxData[i] = data[i];
  }

  if (HAL_CAN_AddTxMessage(isotp_user_can_handle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    return ISOTP_RET_ERROR;
  }

  return ISOTP_RET_OK;
}

/* required, return system tick, unit is micro-second */
uint32_t isotp_user_get_us(void) {
  return HAL_GetTick() * 1000; // idk if this will work
}