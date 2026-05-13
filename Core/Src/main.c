/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Laser.h"
#include "OLED.h"
#include "REG.h"
#include "StepMotor.h"
#include "Key.h"
#include "Motor.h"
#include "menu.h"
#include "laser.h"
#include "Title.h"
#include "gyroscope.h"
#include "wit_c_sdk.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
StepMotor_Driver *StepMotor;

Motor_Driver *PanMotor;

Menu menu_instance;
Menu *menu = &menu_instance;

title_Driver *title;
FloatConvert conv;

GyroData_t gyroData_instance;
GyroData_t *pGyroData=&gyroData_instance;

uint8_t keynum=0;
uint8_t ch=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
//坐标数据



//TODO 串口中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&huart3)
  {
    title->var.uart_flag=1; // 置位串口标志，主循环里会检测到并处理视觉数据
    //TODO 数据处理
    title->fun->Data_receive(title); 
    HAL_UART_Receive_IT(&huart3, &title->var.rx_byte, 1);  // 只在USART2里重开
  }
  else if(huart==&huart4)
  {
    WitSerialDataIn(ch);
    HAL_UART_Receive_IT(&huart4, &ch, 1);  
  }
  
}

//TODO 定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // 检查是否是TIM2的中断
  {
    title->var.tim_flag=1; // 置位定时器标志，主循环里会检测到并进行位置控制计算
  }
}

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
  StepMotor=StepMotor_Create(&huart1,0x01);
  PanMotor=Motor_Create(0x01,2550,1900,2200);
  title=Titile_Create();
  
  if(StepMotor == NULL || PanMotor == NULL || title == NULL) {
    while(1) {
        for(volatile int i=0; i<10000; i++); // 预先加入软延时，避免紧凑死循环卡死SWD仿真器
    }
  }
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /*初始化所有已配置的外围设备*/
  title->fun->Init(title); // 初始化题目驱动，传入title实例地址以供题目驱动访问和修改数据
  StepMotor->fun->Init(StepMotor);
  StepMotor->fun->Stop(StepMotor); // 初始化时先停止电机，确保安全
  PanMotor->fun->Motor_Init(PanMotor);
  PanMotor->fun->Motor_Move(PanMotor,0); // 初始化时先将PanMotor移动到中位位置，确保安全


  gyroscope_Init(pGyroData); // 初始化陀螺仪，传入pGyroData实例地址以供陀螺仪模块访问和更新数据  


  //菜单初始化
  Menu_Init(menu, title); // 初始化菜单，传入title实例本身以供菜单访问和修改
  Menu_Switch(menu,1);

  //激光初始化
  Laser_Init();  // 初始化激光模块，默认关闭激光
  Laser_Off();  // 打开激光，确保激光在系统启动时就处于工作状态 ////////////////////////////////////////////////////////////////


  // 定义按键数组，包含3个按键的GPIO端口和引脚号
  KEY_Driver key[3] = {     
    Key_Create(GPIOD, GPIO_PIN_8),
    Key_Create(GPIOB, GPIO_PIN_15),
    Key_Create(GPIOD, GPIO_PIN_10)
  };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //打开串口中断，准备接收数据
  HAL_Delay(500); // 延时等待视觉那边上电并准备好发送数据，避免刚开串口中断就接收到无效数据导致程序异常
  HAL_UART_Receive_IT(&huart3, &title->var.rx_byte, 1);
  HAL_UART_Receive_IT(&huart4, &ch, 1);  

  //发送0xFF，等待视觉那边准备好接收数据
  OLED_Clear();
  OLED_ShowString(0, 0, "sending 0xFF", OLED_8X16);
  OLED_Update();
  StepMotor->fun->Move(StepMotor,10);
  uint8_t ready_signal = 0xFF;
  do
  {
    HAL_UART_Transmit(&huart3, &ready_signal, 1, 20);
    HAL_Delay(500);
  }while(title->var.mode_next==0); // 等待视觉那边发送数据，通知视觉已经准备好接收数据了
  


  //打开定时器正式开始工作
  //HAL_TIM_Base_Start_IT(&htim2); // 启动定时器中断，定时器会周期性地触发中断，主循环里会检测到并进行位置控制计算

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //TODO while循环

    //TODO PID设置
    //进行pid切换以及发送切换模式数据给视觉
    if(title->var.mode==0&&title->var.mode_next==1)
    {
      title->var.mode=1;
      //框坐标pid
      title->fun->X_PIDSET(title,1.4,0,0.1);
      //陀螺仪pid
      pGyroData->fun->PID_SET(&pGyroData->pid,0,0,0); 
      //云台pid
      PanMotor->fun->PID_SET(PanMotor,0.2,0.001,0.08); 
    }
    else if(title->var.mode==1&&title->var.mode_next==2)
    {
      title->var.mode=2;
      //框坐标pid
      title->fun->X_PIDSET(title,1.4,0.003,0.1);
      //陀螺仪pid
      pGyroData->fun->PID_SET(&pGyroData->pid,0,0,0); 
      //云台pid
      PanMotor->fun->PID_SET(PanMotor,0.3,0.002,0.04); 
    }

    //主程序
    //收到串口的标志位，专门处理数据，对数据进行补偿或者pid拉伸
    if(title->var.uart_flag==1)
    {
      title->var.uart_flag=0;
      //根据不同的mode对x,y进行不同的补偿
      //框坐标+补偿
      if(title->var.mode==1)
      {
        //为了h系数补偿所以要俯仰角
        title->fun->Laser_offset(title); // 进行激光补偿计算，更新title实例中的相关数据，以供后续位置控制计算使用
        title->xy.x=title->xy.frame_x;
        title->xy.y=title->xy.y_offset;

        //判断是否打中框中心，发送标志位给视觉让视觉切换激光打靶
        title->fun->XYRead(title);
        if(title->var.ready==1) 
        {
          title->var.Start_Flag[1]=0x02;
          Laser_On();  // 对准框之后启动激光
          HAL_UART_Transmit(&huart3,title->var.Start_Flag,3,20);  //切换模式数据发送给视觉，通知视觉切换到模式2（激光坐标+补偿）
        }
      }
      //激光坐标+补偿
      else if(title->var.mode==2)
      {
        title->xy.x=-title->xy.laser_x; 
        title->xy.y=-title->xy.laser_y;
      }

      //TODO y坐标环
      //云台追踪+补偿环
      title->fun->X_PIDOUT(title); // 进行位置控制计算，更新title实例中的pid.out的值
      PanMotor->fun->MPID_OUT(PanMotor,title->xy.y); // 进行位置控制计算，并更新PanMotor的输出
      //TODO 电机驱动函数
      StepMotor->fun->Move(StepMotor,title->pid.out); // 根据位置控制计算的输出，发送位置控制指令给StepMotor
      PanMotor->fun->Motor_Move(PanMotor,PanMotor->var.out); // 根据位置控制计算的输出，发送位置控制指令给PanMotor
      
    }
    OLED_Clear();
    OLED_ShowFloatNum(0, 16, title->pid.integral, 5, 5, OLED_8X16);
    OLED_Update();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 167;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 250000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  // __disable_irq(); // 屏蔽掉，防止由于时钟配置失败直接死锁导致ST-Link连不上
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
