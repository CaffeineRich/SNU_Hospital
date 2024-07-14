/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "arm_math.h"
#include "global_variables.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

typedef struct {
  FDCAN_HandleTypeDef   *Module;
  FDCAN_TxHeaderTypeDef *TxHeader;
  FDCAN_RxHeaderTypeDef *RxHeader;
  uint8_t               TxData[8];
  uint8_t               RxData[8];
  uint32_t              RxLocation;
  uint16_t              fault;
  uint16_t              Rx_buf_cnt;
  uint16_t              Tx_buf_cnt;
}Can_HandleTypeDef;

typedef struct {
  uint8_t       rx_buf[RX_BUF_SIZE];
  uint8_t       raw_rx_buf[RAW_RX_BUF_SIZE];
  uint8_t       temp_rx_buf[RX_BUF_SIZE];
  uint8_t       tx_buf[TX_BUF_SIZE];
}Serial_HandleTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_HANDLE_DEFAULTS   {&hfdcan1, &TxHeader, &RxHeader, {0,}, {0,}, FDCAN_RX_FIFO0,0, 0, 0}
#define SER_HANDLE_DEFAULTS   {{0,}, {0,},  {0,}, {0,}}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/*============================================================ 함수 호출 ============================================================*/

unsigned short crc16(unsigned char* buf, unsigned int len);             /// CRC

/// @@@@@@@@@@@@@@@@ Setup
void setup();                                                                                       /// 초기 설정
static void FDCAN_Config();                                                                /// CAN통신 구성
void send_msg_to_receive_from_motor();                                               /// 10ms마다 모터 현재 position 수신
void motor_go_origin();                                                                        /// 모터1,2 원점으로 구동

/// @@@@@@@@@@@@@@@@ While
void Uart_Rx_Dma_Handler1();                                                              /// 모터1 Serial 메시지 수신 필터
void Uart_Rx_Dma_Handler2();                                                              /// 모터2 Serial 메시지 수신 필터
void read_position_now();                                                                    /// 모터1,2 현재위치 Serial 메시지 -> 모터1,2 현재위치
void read_coordinate_now();                                                                /// 모터1,2 현재위치 -> 현재 xy좌표

/// @@@@@@@@@@@@@@@@ HAL_SYSTICK_Callback
void move_motor_by_trajectory();                                                         /// 각 Way Point의 모터1,2 position CAN 메시지 -> 모터1,2 구동
void send_motor_position_to_vs();                                                        /// Visual Studio로 모터1,2 position 송신


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/// @@@@ For Can(PC Communication)
Can_HandleTypeDef can = CAN_HANDLE_DEFAULTS;

/// @@@@@ For UART(T-Motor Control)
Serial_HandleTypeDef Motor[2] = {SER_HANDLE_DEFAULTS, SER_HANDLE_DEFAULTS};

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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  setup();                                                                                      /// 기본설정
  
  ///@@@@@@@@@@@@@@@@@@@@@@@@
  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, Motor[0].raw_rx_buf, RAW_RX_BUF_SIZE);          /// 모터1 Serial 메시지 수신
  HAL_UART_Receive_DMA(&huart2, Motor[1].raw_rx_buf, RAW_RX_BUF_SIZE);          /// 모터2 Serial 메시지 수신
  
  ///@@@@@@@@@@@@@@@@@@@@@@@@
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    
    check_while++;                                                                          /// while문 정상실행 확인
    Uart_Rx_Dma_Handler1();                                                            /// 모터1 Serial 메시지 수신 필터
    Uart_Rx_Dma_Handler2();                                                            /// 모터2 Serial 메시지 수신 필터
    read_position_now();                                                                  /// 모터1,2 현재위치 Serial 메시지 -> 모터1,2 현재위치
    read_coordinate_now();                                                              /// 모터1,2 현재위치 -> 현재 xy좌표
    
    if(can.RxData[0] == 0xF0 && can.RxData[1] == 0xF0){                   /// 신체정보 수신완료한 경우
      motor1_velocity = 5000;                                                             /// 모터1 속도
      motor1_acceleration = 18000;                                                     /// 모터1 가속도
      motor2_velocity = 5000;                                                             /// 모터2 속도
      motor2_acceleration = 18000;                                                     /// 모터2 가속도
      offset_flag = 1;                                                                         /// Offset 실행
    }
    else if(can.RxData[0] == 0xF0 && can.RxData[1] == 0xFF){
      offset_flag = 0;                                                                          /// Offset 종료
      motor1_velocity = can.RxData[2] * 1000;                                     /// 모터1 속도
      motor1_acceleration = can.RxData[3] * 1000;                              /// 모터1 가속도
      motor2_velocity = can.RxData[4] * 1000;                                     /// 모터2 속도
      motor2_acceleration = can.RxData[5] * 1000;                              /// 모터2 가속도      
      trajectory_flag = 1;                                                                    /// Sector Trajectory 실행
    }
    else if(can.RxData[0] == 0xF0 && can.RxData[1] == 0xFE){
      motor1_velocity = can.RxData[2] * 1000;                                     /// 모터1 속도
      motor1_acceleration = can.RxData[3] * 1000;                              /// 모터1 가속도
      motor2_velocity = can.RxData[4] * 1000;                                     /// 모터2 속도
      motor2_acceleration = can.RxData[5] * 1000;                              /// 모터2 가속도      
    }
    else if(can.RxData[0] == 0xF0 && can.RxData[1] == 0xFD){
      motor1_velocity = 30000;                                                            /// 모터1 속도
      motor1_acceleration = 45000;                                                     /// 모터1 가속도
      motor2_velocity = 30000;                                                            /// 모터2 속도
      motor2_acceleration = 45000;                                                     /// 모터2 가속도      
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void UART_IDLECallback(UART_HandleTypeDef *huart){
  /*if(huart->Instance == USART1){
    Motor[0].RcvFlag = true;
  }
  else if(huart->Instance == USART2){
    Motor[1].RcvFlag = true;
  }*/
}


/*============================================================ HAL 함수 선언 ============================================================*/

void HAL_SYSTICK_Callback(){

  ccnt++;                                                                                                        /// 1ms 간격으로 실행
  if(ccnt % 10 == 0){                                                                                        /// 10ms 간격으로 실행    
    HAL_UART_Transmit(&huart1, Motor[0].tx_buf, TX_BUF_SIZE, 5);                    /// 모터1 Serial 메시지 송신
    HAL_UART_Transmit(&huart2, Motor[1].tx_buf, TX_BUF_SIZE, 5);                    /// 모터2 Serial 메시지 송신
    if(offset_flag == 1 || trajectory_flag == 1){
      move_motor_by_trajectory();                                                                     /// 각 Way Point의 모터1,2 Position CAN 메시지 -> 모터1,2 구동
      send_motor_position_to_vs();                                                                    /// Visual Studio로 모터1,2 position 송신
    }
    ccnt = 0;
  }
  
}

/// Visual Studio로부터 새로운 CAN 메시지를 받으면 자동으로 실행되는 함수
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
    if(hfdcan->Instance == FDCAN1){
      if (HAL_FDCAN_GetRxMessage(can.Module, can.RxLocation, can.RxHeader, can.RxData) != HAL_OK){
        Error_Handler();
      }
      if(can.RxData[0] != 0xF0){
        /// 각 Way Point의 모터1,2 Position
        trajectory_buffer[0] = can.RxData[0];
        trajectory_buffer[1] = can.RxData[1];
        trajectory_buffer[2] = can.RxData[2];
        trajectory_buffer[3] = can.RxData[3];
        trajectory_buffer[4] = can.RxData[4];
        trajectory_buffer[5] = can.RxData[5];
        trajectory_buffer[6] = can.RxData[6];
        trajectory_buffer[7] = can.RxData[7];
      }
    }
  }
  
}


/*============================================================ 함수 선언 ============================================================*/

/// CRC
unsigned short crc16(unsigned char* buf, unsigned int len){
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    };
    return cksum;
}

/// @@@@@@@@@@@@@@@@ Setup
/// 기본설정
void setup(){
  
  FDCAN_Config();                                                                                                                            /// CAN통신 구성

  D_def = (x_def * x_def + y_def * y_def - leg_1 * leg_1 - leg_2 * leg_2) / (2 * leg_1 * leg_2);                     /// D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2) 초기값
  ang2_def = -1. * acos(D_def);                                                                                                          /// 모터2 각도 초기값
  ang1_def = atan2(y_def, x_def) + atan2(leg_2 * sin(-1. * ang2_def), leg_1 + leg_2 * cos(ang2_def));         /// 모터1 각도 초기값

  send_msg_to_receive_from_motor();                                                                                                /// 10ms마다 모터 현재 Position 수신
  motor_go_origin();                                                                                                                         /// 모터1,2 원점으로 구동

}
/// CAN통신 구성
static void FDCAN_Config(){
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;              /// @@@@@@@@@@@@@@@ Id of receive part
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(can.Module, &sFilterConfig) != HAL_OK){
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(can.Module, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(can.Module) != HAL_OK){
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(can.Module, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
    Error_Handler();
  }

  /* Prepare Tx Header */               
  can.TxHeader->Identifier = 0x321;             /// @@@@@@@@@@@@@@@ Id of transmit part
  can.TxHeader->IdType = FDCAN_STANDARD_ID;
  can.TxHeader->TxFrameType = FDCAN_DATA_FRAME;
  can.TxHeader->DataLength = FDCAN_DLC_BYTES_8;
  can.TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can.TxHeader->BitRateSwitch = FDCAN_BRS_OFF;
  can.TxHeader->FDFormat = FDCAN_CLASSIC_CAN;
  can.TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  can.TxHeader->MessageMarker = 0;
  
}
/// 10ms마다 모터 현재 Position 수신
void send_msg_to_receive_from_motor(){
  
  Motor[0].tx_buf[0] = 0x02;
  Motor[0].tx_buf[1] = 0x02;
  Motor[0].tx_buf[2] = 0x0B;
  Motor[0].tx_buf[3] = 0x04;
  Motor[0].tx_buf[4] = 0x9C;
  Motor[0].tx_buf[5] = 0x7E;
  Motor[0].tx_buf[6] = 0x03;
  
  Motor[1].tx_buf[0] = 0x02;
  Motor[1].tx_buf[1] = 0x02;
  Motor[1].tx_buf[2] = 0x0B;
  Motor[1].tx_buf[3] = 0x04;
  Motor[1].tx_buf[4] = 0x9C;
  Motor[1].tx_buf[5] = 0x7E;
  Motor[1].tx_buf[6] = 0x03;
  
  for(int i = 7; i < 18; i++){
    Motor[0].tx_buf[i] = 0;
    Motor[1].tx_buf[i] = 0;
  }
  
  HAL_UART_Transmit(&huart1, Motor[0].tx_buf, TX_BUF_SIZE, 5);                  /// 모터1 Serial 메시지 송신
  HAL_UART_Transmit(&huart2, Motor[1].tx_buf, TX_BUF_SIZE, 5);                  /// 모터2 Serial 메시지 송신
  
}
/// 모터1,2 원점으로 구동
void motor_go_origin(){

  /// Motor 1
  Motor[0].tx_buf[0] = 0x02;                                                                         /// sof
  Motor[0].tx_buf[1] = 0x0D;                                                                        /// len
  Motor[0].tx_buf[2] = 0x5B;                                                                        /// frame
  
  Motor[0].tx_buf[3] = 0x00;                                                                        /// pos
  Motor[0].tx_buf[4] = 0x00;
  Motor[0].tx_buf[5] = 0x00;
  Motor[0].tx_buf[6] = 0x00;

  Motor[0].tx_buf[7] = 0x00;                                                                        /// vel (5000)
  Motor[0].tx_buf[8] = 0x00;
  Motor[0].tx_buf[9] = 0x13;
  Motor[0].tx_buf[10] = 0x88;

  Motor[0].tx_buf[11] = 0x00;                                                                      /// acc (30000)
  Motor[0].tx_buf[12] = 0x00;
  Motor[0].tx_buf[13] = 0x75;
  Motor[0].tx_buf[14] = 0x30;

  static unsigned short cksum_1;                                                            /// crc
  uint8_t buf_1[13] = { Motor[0].tx_buf[2], Motor[0].tx_buf[3], Motor[0].tx_buf[4], Motor[0].tx_buf[5], Motor[0].tx_buf[6],
                               Motor[0].tx_buf[7], Motor[0].tx_buf[8], Motor[0].tx_buf[9], Motor[0].tx_buf[10], Motor[0].tx_buf[11],
                               Motor[0].tx_buf[12], Motor[0].tx_buf[13], Motor[0].tx_buf[14]};
  uint8_t crc_1[2] = { 0 };
  cksum_1 = crc16(buf_1, 13);
  crc_1[0] = cksum_1 / 256;
  crc_1[1] = cksum_1 % 256;
  Motor[0].tx_buf[15] = crc_1[0];
  Motor[0].tx_buf[16] = crc_1[1];
  
  Motor[0].tx_buf[17] = 0x03;                                                                       /// eof
  
  /// Motor 2
  Motor[1].tx_buf[0] = 0x02;                                                                        /// sof
  Motor[1].tx_buf[1] = 0x0D;                                                                       /// len
  Motor[1].tx_buf[2] = 0x5B;                                                                       /// frame
  
  Motor[1].tx_buf[3] = 0x00;                                                                       /// pos
  Motor[1].tx_buf[4] = 0x00;
  Motor[1].tx_buf[5] = 0x00;
  Motor[1].tx_buf[6] = 0x00;

  Motor[1].tx_buf[7] = 0x00;                                                                       /// vel (5000)
  Motor[1].tx_buf[8] = 0x00;
  Motor[1].tx_buf[9] = 0x13;
  Motor[1].tx_buf[10] = 0x88;
  
  Motor[1].tx_buf[11] = 0x00;                                                                     /// acc (30000)
  Motor[1].tx_buf[12] = 0x00;
  Motor[1].tx_buf[13] = 0x75;
  Motor[1].tx_buf[14] = 0x30;

  static unsigned short cksum_2;                                                           /// crc
  uint8_t buf_2[13] = { Motor[1].tx_buf[2], Motor[1].tx_buf[3], Motor[1].tx_buf[4], Motor[1].tx_buf[5], Motor[1].tx_buf[6],
                               Motor[1].tx_buf[7], Motor[1].tx_buf[8], Motor[1].tx_buf[9], Motor[1].tx_buf[10], Motor[1].tx_buf[11],
                               Motor[1].tx_buf[12], Motor[1].tx_buf[13], Motor[1].tx_buf[14]};
  uint8_t crc_2[2] = { 0 };
  cksum_2 = crc16(buf_2, 13);
  crc_2[0] = cksum_2 / 256;
  crc_2[1] = cksum_2 % 256;
  Motor[1].tx_buf[15] = crc_2[0];
  Motor[1].tx_buf[16] = crc_2[1];

  Motor[1].tx_buf[17] = 0x03;                                                                     /// eof
    
  HAL_UART_Transmit(&huart1, Motor[0].tx_buf, TX_BUF_SIZE, 5);               /// 모터1 Serial 메시지 송신
  HAL_UART_Transmit(&huart2, Motor[1].tx_buf, TX_BUF_SIZE, 5);               /// 모터2 Serial 메시지 송신
  
}
/// 모터1,2 원점 설정
void motor_set_origin(){
  
  /// Motor 1
  Motor[0].tx_buf[0] = 0x02;                                                                      /// sof
  Motor[0].tx_buf[1] = 0x02;                                                                      /// len
  Motor[0].tx_buf[2] = 0x5F;                                                                     /// frame
  Motor[0].tx_buf[3] = 0x01;
  Motor[0].tx_buf[4] = 0x0E;
  Motor[0].tx_buf[5] = 0xA0;
  Motor[0].tx_buf[6] = 0x03;                                                                      /// eof
  
  /// Motor 2
  Motor[1].tx_buf[0] = 0x02;                                                                      /// sof
  Motor[1].tx_buf[1] = 0x02;                                                                      /// len
  Motor[1].tx_buf[2] = 0x5F;                                                                     /// frame
  Motor[1].tx_buf[3] = 0x01;
  Motor[1].tx_buf[4] = 0x0E;
  Motor[1].tx_buf[5] = 0xA0;
  Motor[1].tx_buf[6] = 0x03;                                                                     /// eof
  
  HAL_UART_Transmit(&huart1, Motor[0].tx_buf, TX_BUF_SIZE, 5);             /// 모터1 Serial 메시지 송신
  HAL_UART_Transmit(&huart2, Motor[1].tx_buf, TX_BUF_SIZE, 5);             /// 모터2 Serial 메시지 송신
  
}


/// @@@@@@@@@@@@@@@@ While
/// 모터1 Serial 메시지 수신 필터
void Uart_Rx_Dma_Handler1(){
  
  head_pos[0] = huart1.RxXferSize - huart1.hdmarx->Instance->CNDTR - 1;
  while(tail_pos[0] != head_pos[0]) {
    
    switch (rx_PacketMode[0]) {
      
    case 0:
      if (Motor[0].raw_rx_buf[tail_pos[0]] == 0x02) {                                                                 /// Header check
        Motor[0].temp_rx_buf[rx_checkSize[0]++] = Motor[0].raw_rx_buf[tail_pos[0]++];
        if(Motor[0].raw_rx_buf[tail_pos[0]] == RX_LEN){                                                            /// Len check
          Motor[0].temp_rx_buf[rx_checkSize[0]++] = Motor[0].raw_rx_buf[tail_pos[0]++];
          if(Motor[0].raw_rx_buf[tail_pos[0]] == RX_FRAME){                                                     /// Frame chcek
            Motor[0].temp_rx_buf[rx_checkSize[0]++] = Motor[0].raw_rx_buf[tail_pos[0]];
            rx_PacketMode[0] = 1;
          }
        }
      }
      else{
        rx_checkSize[0] = 0;
      }
      break;
      
    case 1:
      Motor[0].temp_rx_buf[rx_checkSize[0]++] = Motor[0].raw_rx_buf[tail_pos[0]];
      if(rx_checkSize[0]== RX_BUF_SIZE){
        if(Motor[0].temp_rx_buf[RX_BUF_SIZE-1] == 0x03){
          uint8_t buf[6] = {Motor[0].temp_rx_buf[2], Motor[0].temp_rx_buf[3],
                                  Motor[0].temp_rx_buf[4], Motor[0].temp_rx_buf[5],
                                  Motor[0].temp_rx_buf[6]};
          rx_checkSum[0] = crc16(buf, RX_LEN);
          if(Motor[0].temp_rx_buf[7] == rx_checkSum[0] / 256 &&
             Motor[0].temp_rx_buf[8] == rx_checkSum[0] % 256){
            
            Motor[0].rx_buf[0] = Motor[0].temp_rx_buf[0];
            Motor[0].rx_buf[1] = Motor[0].temp_rx_buf[1];
            Motor[0].rx_buf[2] = Motor[0].temp_rx_buf[2];
            Motor[0].rx_buf[3] = Motor[0].temp_rx_buf[3];
            Motor[0].rx_buf[4] = Motor[0].temp_rx_buf[4];
            Motor[0].rx_buf[5] = Motor[0].temp_rx_buf[5];
            Motor[0].rx_buf[6] = Motor[0].temp_rx_buf[6];
            Motor[0].rx_buf[7] = Motor[0].temp_rx_buf[7];
            Motor[0].rx_buf[8] = Motor[0].temp_rx_buf[8];
            Motor[0].rx_buf[9] = Motor[0].temp_rx_buf[9];
          }
          else{
            rx_checkSum[0] =  0;
            rx_checkSize[0] = 0;
            rx_PacketMode[0] = 0;
          }
        }
        else{
          rx_checkSum[0] =  0;
          rx_checkSize[0] = 0;
          rx_PacketMode[0] = 0;
        }
      }
      else if(rx_checkSize[0] > RX_BUF_SIZE){
        rx_checkSum[0] =  0;
        rx_checkSize[0] = 0;
        rx_PacketMode[0] = 0;
      }
      break;
    }
    tail_pos[0]++;
  }
  
}
/// 모터2 Serial 메시지 수신 필터
void Uart_Rx_Dma_Handler2(){
  
  head_pos[1] = huart2.RxXferSize - huart2.hdmarx->Instance->CNDTR - 1;
  while(tail_pos[1] != head_pos[1]) {
    
    switch (rx_PacketMode[1]) {
      
    case 0:
      if (Motor[1].raw_rx_buf[tail_pos[1]] == 0x02) {                                                                 /// Header check
        Motor[1].temp_rx_buf[rx_checkSize[1]++] = Motor[1].raw_rx_buf[tail_pos[1]++];
        if(Motor[1].raw_rx_buf[tail_pos[1]] == RX_LEN){                                                            /// Len check
          Motor[1].temp_rx_buf[rx_checkSize[1]++] = Motor[1].raw_rx_buf[tail_pos[1]++];
          if(Motor[1].raw_rx_buf[tail_pos[1]] == RX_FRAME){                                                     /// Frame chcek
            Motor[1].temp_rx_buf[rx_checkSize[1]++] = Motor[1].raw_rx_buf[tail_pos[1]];
            rx_PacketMode[1] = 1;
          }
        }
      }
      else{
        rx_checkSize[1] = 0;
      }
      break;
      
    case 1:
      Motor[1].temp_rx_buf[rx_checkSize[1]++] = Motor[1].raw_rx_buf[tail_pos[1]];
      if(rx_checkSize[1]== RX_BUF_SIZE){
        if(Motor[1].temp_rx_buf[RX_BUF_SIZE-1] == 0x03){
          uint8_t buf[6] = {Motor[1].temp_rx_buf[2], Motor[1].temp_rx_buf[3],
                                  Motor[1].temp_rx_buf[4], Motor[1].temp_rx_buf[5],
                                  Motor[1].temp_rx_buf[6]};
          rx_checkSum[1] = crc16(buf, RX_LEN);
          if(Motor[1].temp_rx_buf[7] == rx_checkSum[1] / 256 &&
             Motor[1].temp_rx_buf[8] == rx_checkSum[1] % 256){
            Motor[1].rx_buf[0] = Motor[1].temp_rx_buf[0];
            Motor[1].rx_buf[1] = Motor[1].temp_rx_buf[1];
            Motor[1].rx_buf[2] = Motor[1].temp_rx_buf[2];
            Motor[1].rx_buf[3] = Motor[1].temp_rx_buf[3];
            Motor[1].rx_buf[4] = Motor[1].temp_rx_buf[4];
            Motor[1].rx_buf[5] = Motor[1].temp_rx_buf[5];
            Motor[1].rx_buf[6] = Motor[1].temp_rx_buf[6];
            Motor[1].rx_buf[7] = Motor[1].temp_rx_buf[7];
            Motor[1].rx_buf[8] = Motor[1].temp_rx_buf[8];
            Motor[1].rx_buf[9] = Motor[1].temp_rx_buf[9];
          }
          else{
            rx_checkSum[1] =  0;
            rx_checkSize[1] = 0;
            rx_PacketMode[1] = 0;
          }
        }
        else{
          rx_checkSum[1] =  0;
          rx_checkSize[1] = 0;
          rx_PacketMode[1] = 0;
        }
      }
      else if(rx_checkSize[1] > RX_BUF_SIZE){
        rx_checkSum[1] =  0;
        rx_checkSize[1] = 0;
        rx_PacketMode[1] = 0;
      }
    }
    tail_pos[1]++;
  }
  
}
/// 모터1,2 현재위치 Serial 메시지 -> 모터1,2 현재위치
void read_position_now(){

  m1_pos_now_temp = (Motor[0].rx_buf[3] << 24)                                               /// 모터1 Position x 10^4
                                 + (Motor[0].rx_buf[4] << 16)
                                 + (Motor[0].rx_buf[5] << 8)
                                 + Motor[0].rx_buf[6];
  m2_pos_now_temp = (Motor[1].rx_buf[3] << 24)                                               /// 모터2 Position x 10^4
                                 + (Motor[1].rx_buf[4] << 16)
                                 + (Motor[1].rx_buf[5] << 8)
                                 + Motor[1].rx_buf[6];
  
  if(Motor[0].rx_buf[3] == 0xFF){                                                                       /// 모터1 Position이 음수인 경우
    motor1_position_now = (double)(m1_pos_now_temp - 4294967296.) / 10000.;     /// 모터1 현재위치
  }
  else if(Motor[0].rx_buf[3] == 0x00){                                                                /// 모터1 Position이 양수인 경우
    motor1_position_now = (double)m1_pos_now_temp / 10000.;                           /// 모터1 현재위치
  }
  
  if(Motor[1].rx_buf[3] == 0xFF){                                                                       /// 모터2 Position이 음수인 경우
    motor2_position_now = (double)(m2_pos_now_temp - 4294967296.) / 10000.;     /// 모터2 현재위치
  }
  else if(Motor[1].rx_buf[3] == 0x00){                                                                /// 모터2 Position이 양수인 경우
    motor2_position_now = (double)m2_pos_now_temp / 10000.;                           /// 모터2 현재위치
  }

}
/// 모터1,2 현재위치 -> 현재 xy좌표
void read_coordinate_now(){
  
  x_coordinate_now = leg_1 * cos(ang1_def - motor1_position_now * PI / 180.)            /// 현재 x좌표
                               + leg_2 * cos(ang1_def - motor1_position_now * PI / 180.
                               + ang2_def + motor2_position_now * PI / 180.);
  y_coordinate_now = leg_1 * sin(ang1_def - motor1_position_now * PI / 180.)             /// 현재 y좌표
                               + leg_2 * sin(ang1_def - motor1_position_now * PI / 180.
                               + ang2_def + motor2_position_now * PI / 180.);
 
}

/// @@@@@@@@@@@@@@@@ HAL_SYSTICK_Callback
/// 각 Way Point의 모터1,2 Position CAN 메시지 -> 모터1,2 구동
void move_motor_by_trajectory(){
  
  m1_pos_temp = (trajectory_buffer[0] << 24)                                                                                     /// 모터1 Position x 10^3
                          + (trajectory_buffer[1] << 16)
                          + (trajectory_buffer[2] << 8)
                          + trajectory_buffer[3];
  m2_pos_temp = (trajectory_buffer[4] << 24)                                                                                     /// 모터2 Position x 10^3
                          + (trajectory_buffer[5] << 16)
                          + (trajectory_buffer[6] << 8)
                          + trajectory_buffer[7];
  
  if(trajectory_buffer[0] == 0xFF){                                                                                                      /// 모터1 Position이 음수인 경우
    degree1_target = (double)(m1_pos_temp - 4294967296.) / 1000.;                                                      /// 모터1 목표각도
  }
  else if(trajectory_buffer[0] == 0x00){                                                                                               /// 모터1 Position이 양수인 경우
    degree1_target = (double)m1_pos_temp / 1000.;                                                                            /// 모터1 목표각도
  }
  
  if(trajectory_buffer[4] == 0xFF){                                                                                                      /// 모터2 Position이 음수인 경우
    degree2_target = (double)(m2_pos_temp - 4294967296.) / 1000.;                                                      /// 모터2 목표각도
  }
  else if(trajectory_buffer[4] == 0x00){                                                                                               /// 모터2 Position이 양수인 경우
    degree2_target = (double)m2_pos_temp / 1000.;                                                                            /// 모터2 목표각도
  }
    
  /// 모터 P,V,A 값 -> 모터 Serial 코드값

  /// Motor 1                 
  Motor[0].tx_buf[0] = 0x02;                                                                                                             /// SOF
  Motor[0].tx_buf[1] = len_write;                                                                                                       /// Data Length
  Motor[0].tx_buf[2] = frame_write;                                                                                                   /// Data Frame
  
  Motor[0].tx_buf[3] = trajectory_buffer[0];                                                                                        /// Position
  Motor[0].tx_buf[4] = trajectory_buffer[1];
  Motor[0].tx_buf[5] = trajectory_buffer[2];
  Motor[0].tx_buf[6] = trajectory_buffer[3];
  
  Motor[0].tx_buf[7] = (motor1_velocity >> 24) & 0xFF;                                                                       /// Velocity
  Motor[0].tx_buf[8] = (motor1_velocity >> 16) & 0xFF;
  Motor[0].tx_buf[9] = (motor1_velocity >> 8) & 0xFF;
  Motor[0].tx_buf[10] = motor1_velocity & 0xFF;

  Motor[0].tx_buf[11] = (motor1_acceleration >> 24) & 0xFF;                                                               /// Acceleration
  Motor[0].tx_buf[12] = (motor1_acceleration >> 16) & 0xFF;
  Motor[0].tx_buf[13] = (motor1_acceleration >> 8) & 0xFF;
  Motor[0].tx_buf[14] = motor1_acceleration & 0xFF;

  static unsigned short cksum_1;                                                                                                 /// CRC
  uint8_t buf_1[] = { frame_write,Motor[0].tx_buf[3],Motor[0].tx_buf[4],Motor[0].tx_buf[5],
                            Motor[0].tx_buf[6],Motor[0].tx_buf[7], Motor[0].tx_buf[8],
                            Motor[0].tx_buf[9],Motor[0].tx_buf[10],Motor[0].tx_buf[11],
                            Motor[0].tx_buf[12],Motor[0].tx_buf[13],Motor[0].tx_buf[14] };
  uint8_t crc_1[2] = { 0 };
  cksum_1 = crc16(buf_1, len_write);
  crc_1[0] = cksum_1 / 256;
  crc_1[1] = cksum_1 % 256;
  
  Motor[0].tx_buf[15] = crc_1[0];
  Motor[0].tx_buf[16] = crc_1[1];
  Motor[0].tx_buf[17] = 0x03;                                                                                                            /// EOF
  
  /// Motor 2
  Motor[1].tx_buf[0] = 0x02;                                                                                                             /// SOF
  Motor[1].tx_buf[1] = len_write;                                                                                                       /// Data Length
  Motor[1].tx_buf[2] = frame_write;                                                                                                   /// Data Frame
  
  Motor[1].tx_buf[3] = trajectory_buffer[4];                                                                                        /// Position
  Motor[1].tx_buf[4] = trajectory_buffer[5];
  Motor[1].tx_buf[5] = trajectory_buffer[6];
  Motor[1].tx_buf[6] = trajectory_buffer[7];
  
  Motor[1].tx_buf[7] = (motor2_velocity >> 24) & 0xFF;                                                                       /// Velocity
  Motor[1].tx_buf[8] = (motor2_velocity >> 16) & 0xFF;
  Motor[1].tx_buf[9] = (motor2_velocity >> 8) & 0xFF;
  Motor[1].tx_buf[10] = motor2_velocity & 0xFF;

  Motor[1].tx_buf[11] = (motor2_acceleration >> 24) & 0xFF;                                                               /// Acceleration
  Motor[1].tx_buf[12] = (motor2_acceleration >> 16) & 0xFF;
  Motor[1].tx_buf[13] = (motor2_acceleration >> 8) & 0xFF;
  Motor[1].tx_buf[14] = motor2_acceleration & 0xFF;

  static unsigned short cksum_2;                                                                                                 /// CRC
  uint8_t buf_2[] = { frame_write,Motor[1].tx_buf[3],Motor[1].tx_buf[4],Motor[1].tx_buf[5],
                            Motor[1].tx_buf[6],Motor[1].tx_buf[7], Motor[1].tx_buf[8],
                            Motor[1].tx_buf[9],Motor[1].tx_buf[10],Motor[1].tx_buf[11],
                            Motor[1].tx_buf[12],Motor[1].tx_buf[13],Motor[1].tx_buf[14] };
  uint8_t crc_2[2] = { 0 };
  cksum_2 = crc16(buf_2, len_write);
  crc_2[0] = cksum_2 / 256;
  crc_2[1] = cksum_2 % 256;
  
  Motor[1].tx_buf[15] = crc_2[0];
  Motor[1].tx_buf[16] = crc_2[1];
  Motor[1].tx_buf[17] = 0x03;                                                                                                            /// EOF

  degree1_last_target = degree1_target;                                                                                              /// 최신각도1 최신화
  degree2_last_target = degree2_target;                                                                                              /// 최신각도2 최신화

}
/// Visual Studio로 모터1,2 position 송신
void send_motor_position_to_vs(){
  
  can.TxData[0] = Motor[0].rx_buf[3];                                                                   /// 모터1 Position x 10^4
  can.TxData[1] = Motor[0].rx_buf[4];
  can.TxData[2] = Motor[0].rx_buf[5];
  can.TxData[3] = Motor[0].rx_buf[6];
  can.TxData[4] = Motor[1].rx_buf[3];                                                                   /// 모터2 Position x 10^4
  can.TxData[5] = Motor[1].rx_buf[4];
  can.TxData[6] = Motor[1].rx_buf[5];
  can.TxData[7] = Motor[1].rx_buf[6];
  HAL_FDCAN_AddMessageToTxFifoQ(can.Module, can.TxHeader, can.TxData);
  
}


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
