/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "isotp.h"
#include "stm32f407x_flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*ptrF)(uint32_t dlyticks);
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FIRMWARE_SIZE 1024 * 50
#define ISOTP_BUFSIZE 4096
#define ISOTP_RECV_CAN_ID 0x700

#define ACK 0x06
#define NACK 0x15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum {
  STATE_IDLE,
  STATE_RECEIVING,
  STATE_RECEIVED
} state = STATE_IDLE;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t file_pos = 0; // Position in firmware file in words
uint32_t firmware_size = 0;
uint8_t isotp_payload_rx[7] = {0};
uint8_t isotp_payload_tx[7] = {0};
int ret;

static IsoTpLink isotp_link;

static uint8_t isotp_rx_buffer[ISOTP_BUFSIZE];
static uint8_t isotp_tx_buffer[ISOTP_BUFSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void goToApp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void goToApp(void)
{
  uint32_t JumpAddress;
  pFunction Jump_to_Application;
  printf("Jumping to Application \n");

  if (((*(uint32_t *)FLASH_APP_ADDR) & 0x2FFC0000) == 0x20000000)
  {
    HAL_Delay(100);
    printf("Valid Stack Pointer...\n");

    JumpAddress = *(uint32_t *)(FLASH_APP_ADDR + 4);
    Jump_to_Application = (pFunction)JumpAddress;

    __set_MSP(*(uint32_t *)FLASH_APP_ADDR);
    Jump_to_Application();
  }
  else
  {
    printf("Failed to Start Application\n");
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    Error_Handler();
  }

  switch (RxHeader.StdId)
  {
  case ISOTP_RECV_CAN_ID:
    isotp_on_can_message(&isotp_link, RxData, RxHeader.DLC);
    break;
  default:
    break;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  // for testing, set boot pin to high
  HAL_GPIO_WritePin(BOOT_GPIO_Port, BOOT_Pin, GPIO_PIN_SET);

  // If BOOT pin is reset, jump to application
  if (HAL_GPIO_ReadPin(BOOT_GPIO_Port, BOOT_Pin) == GPIO_PIN_RESET) {
    goToApp();
  }

  // Activate CAN RX interrupt
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // Initialize ISO-TP
  isotp_init_link(&isotp_link, 0x701, isotp_tx_buffer, ISOTP_BUFSIZE, isotp_rx_buffer, ISOTP_BUFSIZE);

  // Erase flash
  flashEraseAppSectors();

  // Send inital NACK message until we get response from host
  uint16_t copy_size;
  while (isotp_payload_rx[0] != ACK) {
    isotp_payload_tx[0] = NACK;
    ret = isotp_send(&isotp_link, isotp_payload_tx, 1);
    HAL_Delay(5);
    ret = isotp_receive(&isotp_link, isotp_payload_rx, sizeof(isotp_payload_rx), &copy_size);
    HAL_Delay(100);
  }

  // Byte 0: ACK
  // Byte 1-4: Firmware size (little endian)
  memcpy(&firmware_size, &isotp_payload_rx[1], 4);

  memset(isotp_payload_rx, 0, sizeof(isotp_payload_rx));
  memset(isotp_payload_tx, 0, sizeof(isotp_payload_tx));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Poll ISO-TP Link
    isotp_poll(&isotp_link);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Receive firmware at ISOTP_BUFSIZE increments, and write to flash
    if (isotp_link.receive_status == ISOTP_RECEIVE_STATUS_FULL) {
      // Write to flash
      ret = flashWriteApp(file_pos, (uint32_t *) isotp_link.receive_buffer, isotp_link.receive_size / 4);

      file_pos += isotp_link.receive_size;

      if (ret != 0) {
        // Error writing to flash
        // Send error message
        isotp_payload_tx[0] = NACK;
        isotp_send(&isotp_link, isotp_payload_tx, 1);
        Error_Handler();
      }

      // Send success message
      isotp_payload_tx[0] = ACK;
      isotp_send(&isotp_link, isotp_payload_tx, 1);

      isotp_link.receive_status = ISOTP_RECEIVE_STATUS_IDLE;
    } else if (file_pos >= firmware_size) {
      // Firmware received
      // Send success message
      isotp_payload_tx[0] = ACK;
      isotp_send(&isotp_link, isotp_payload_tx, 1);

      // Jump to application
      goToApp();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
