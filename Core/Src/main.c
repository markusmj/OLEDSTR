/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * Steering Angle Reader and OLED Screen for HPF017
  *
  * Author: Markus Jahn
  *
  ******************************************************************************

  Can Messages read to OLED screen:

  Kerttu 0x7B6, High to Low, Bytes 0-1  (HPF019 ID=0xB Bytes 0-3)
  Low Voltage Battery 0x527(or 0x520!!!), High to Low, Byte 1
  Max Torque and Drive Mode 0x7C8 Byte 1=Max Torque Byte 0=driveModeFuture

  *****************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1306.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Captured Value */
uint32_t            PWMValue = 0;
uint16_t  			STRAngle = 0;

CAN_FilterTypeDef  sFilterConfig;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t               TxData[2];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint32_t i=0;
uint32_t scrtimer=0;
uint16_t Kerttu=555;
uint16_t LVBatt=203;
uint8_t DriveMode=5;
uint8_t Turku=55;
char txt[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // start the PWM counters
  HAL_Delay(10);
  SSD1306_Init();  // initialise Screen

  // Start Can Bus

  // Defining CAN 1 Filter Banks.

    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // BMS Voltage
    // Can Message (HPF019 ID=0xB Bytes 0-3) 0x7B6, High to Low, Bytes 0-1

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = 0x07B6 << 5;
    sFilterConfig.FilterMaskIdHigh = 0x07B6 << 5;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
      //Filter configuration Error
      Error_Handler();
    }

    // Low Voltage
    // Can Message 0x527, High to Low, Byte 1

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = 0x0527 << 5;
    sFilterConfig.FilterMaskIdHigh = 0x0527 << 5;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        {
          //Filter configuration Error
          Error_Handler();
        }

    // Drive Mode & Max Torque
    // Can Message 0x7C8 Byte 1=Max Torque Byte 0=driveModeFuture

    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = 0x07C8 << 5;
    sFilterConfig.FilterMaskIdHigh = 0x07C8 << 5;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        {
          //Filter configuration Error
          Error_Handler();
        }


  // Start Can 1

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      // Start Error
      Error_Handler();
    }

    scrtimer=HAL_GetTick();

    // Write static stuff to screen only once
                  SSD1306_Clear();
    			  SSD1306_GotoXY (5,0);
   	  		      SSD1306_Puts ("Kerttu", &Font_7x10, 1);
  	  		      SSD1306_GotoXY (65,0);
   	  		      SSD1306_Puts ("LV Batt.", &Font_7x10, 1);
   	  		      SSD1306_GotoXY (10,33);
   	  		      SSD1306_Puts ("Mode", &Font_7x10, 1);
   	  		      SSD1306_GotoXY (65,33);
   	  		      SSD1306_Puts ("Torque", &Font_7x10, 1);
   	  		      SSD1306_UpdateScreen();

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	   // Read Steering Angle sensor
	      PWMValue = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

	  	  if (PWMValue != 0)
	  	  {
	  		STRAngle = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2)*360/PWMValue;
	  	  }
	  	  else
	  	  {
	  		  STRAngle = 0;
	  	  }


	  	// Receive Can 1 message

	  		    if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 1)
	  		      {
	  		        /* Reception Missing */
	  		        Error_Handler();
	  		      }

	  		      if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  		      {
	  		        /* Reception Error */
	  		        Error_Handler();
	  		      }

	  		    switch ( RxHeader.StdId )
	  		            {
	  		    			case 0x7B6 :  // BMS Voltage
	  		                    Kerttu=256*RxData[0]+RxData[1];
	  		                    break;

	  		    			case 0x527 :  // Low Voltage Battery or is it 0x520???
	  		    				LVBatt=RxData[1];
	  		    				break;

	  		    			case 0x07C8 :  // Byte 1=Max Torque Byte 0=driveModeFuture
	  		    				DriveMode=RxData[0];
	  		    				Turku=RxData[0];
	  		    				break;


	  		            }
	  		      // uint32_t to bytes

	  		      union {
	  		    	    uint16_t integer;
	  		      		unsigned char bytearray[2];
	  		      	} data;
	  		      	data.integer = STRAngle;


	  		      // Can 1 Start the Transmission process
	  		        TxHeader.StdId = 69;
	  		        TxHeader.RTR = CAN_RTR_DATA;
	  		        TxHeader.IDE = CAN_ID_STD;
	  		        TxHeader.DLC = 2;
	  		        TxHeader.TransmitGlobalTime = DISABLE;
	  		        TxData[0] = data.bytearray[0];
	  		        TxData[1] = data.bytearray[1];


	  		        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  		        {
	  		          /* Transmission request Error */
	  		          Error_Handler();
	  		        }

	  		        /* Wait transmission complete */
	  		          while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}

	   //Write to the screen


if (HAL_GetTick()-scrtimer > 100) {
	    SSD1306_GotoXY (5,12);
	    sprintf(txt, "%iv", Kerttu);
	    SSD1306_Puts (txt, &Font_11x18, 1);
	    SSD1306_GotoXY (65,12);
	    sprintf(txt, "%u.%01uv", LVBatt/10, LVBatt%10);
	    SSD1306_Puts (txt, &Font_11x18, 1);
	    SSD1306_GotoXY (12,45);
	    sprintf(txt, "#%i", DriveMode);
	    SSD1306_Puts (txt, &Font_11x18, 1);
	    SSD1306_GotoXY (65,45);
	    sprintf(txt, "%iNm", Turku);
	    SSD1306_Puts (txt, &Font_11x18, 1);
	    SSD1306_UpdateScreen();
        scrtimer=HAL_GetTick();
} else { HAL_Delay(4);}

	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);


    /* USER CODE END WHILE */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

    // For better speed to update the screen
	// hi2c1.Init.ClockSpeed = 1600000;

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 1600000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
