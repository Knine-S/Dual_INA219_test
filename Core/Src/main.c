/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //i2c2 ma connect wires instead of i2c1
#include <stm32f407xx.h>
#include <stm32f4xx.h>
#include "ina219.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int _write(int file, char *data, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
INA219_t ina1, ina2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// void i2c_scan(void);
// void ina219_dual_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

 printf("Scanning I2C bus...\n");
  for(uint8_t addr = 1; addr < 127; addr++)
  {
    if(HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK)
    {
      printf("Device found at 0x%02X\n", addr);
    }
  }

  printf("Starting INA219 test...\n");

  // INA1 0x40
  ina1.address = 0x40;
  if(ina219_init(&ina1)){
    printf("INA219 at 0x40 init success.\n");
  }else{
    printf("INA219 at 0x40 init failed.\n");}

  // INA2 0x41
  ina2.address = 0x41;
  if(ina219_init(&ina2)){
    printf("INA219 at 0x41 init success.\n");
  }else{
    printf("INA219 at 0x41 init failed.\n");
  }


    const float max_current = 2.5; // Amp
  const float r_shunt = 0.1;     // Ohm
  const bool status_cal = ina219_calibrate( &ina1 ,max_current, r_shunt);

  // See ina219_regcal.cpp to compute config_val.
  const uint16_t config_val = 0x199F;
  const bool status_config = ina219_configure(&ina1, config_val);

  if (status_cal && status_config)
  {
    printf("Successful configuration of ina1!\r\n");
  }
  else
  {
    printf("Configuration error of ina1!\r\n");
    while (1);
  }


  const bool status_cal2 = ina219_calibrate( &ina2 ,max_current, r_shunt);

  // See ina219_regcal.cpp to compute config_val.
  const uint16_t config_val2 = 0x199F;
  const bool status_config2 = ina219_configure(&ina2, config_val2);

  if (status_cal2 && status_config2)
  {
    printf("Successful configuration of ina2!\r\n");
  }
  else
  {
    printf("Configuration error of ina2!\r\n");
    while (1);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    float bus_v1 = ina219_get_bus_voltage(&ina1);
    float shunt_v1 = ina219_get_shunt_voltage(&ina1);
    float current1 = ina219_get_current(&ina1);
    float power1 = ina219_get_power(&ina1);

    float bus_v2 = ina219_get_bus_voltage(&ina2);
    float shunt_v2 = ina219_get_shunt_voltage(&ina2);
    float current2 = ina219_get_current(&ina2);
    float power2 = ina219_get_power(&ina2);

    printf("INA1 (0x40): Bus=%iV Shunt=%iV Current=%iA Power=%iW\n", (int)(bus_v1 *1000) ,(int)( shunt_v1 *1000) ,(int)( current1 *1000),(int)( power1 *1000));
    printf("INA2 (0x41): Bus=%iV Shunt=%iV Current=%iA Power=%iW\n", (int)(bus_v2 *1000) ,(int)( shunt_v2 *1000) ,(int)( current2 *1000),(int)( power2 *1000));
    printf("-------------------------------------------\n");

    HAL_Delay(1000);
   
  
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
