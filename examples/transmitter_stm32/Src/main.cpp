/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cetaguard.h"
#include "lt8920.h"
#include <string.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct status_flags{
	uint32_t btn_intr :1;
	uint32_t spi_intr :1;
	uint32_t got_packet :1;
	uint32_t expecting_rx :1;
	uint32_t waiting_tx :1;
	uint32_t pairing_mode :1;
	uint32_t cold_boot :1;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

LPTIM_HandleTypeDef hlptim1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile struct status_flags status = {0};
uint8_t message_buffer[64] = {0x01, 0x02, 0x03, 0x99, 0x04, 0x03, 0x02, 0x01};
uint8_t recv_buffer[64];
uint8_t last_received;
volatile uint32_t last_error;
volatile uint32_t total_errors;
Cetaguard_transmitter tx;
LT8920 radio(&hspi1);
volatile uint32_t debug_data[64];
volatile uint32_t debug_data_ctr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t measure_battery_voltage();
void set_RTC_timestamp(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void print_to_serial(const char* stuff){
	HAL_UART_Transmit(&huart2, (uint8_t*)stuff, strlen(stuff), 10);
}

#define SERIAL_PRINTF(...) do { char buffer[128]; snprintf(buffer, 128, __VA_ARGS__); print_to_serial(buffer); } while (0)

class My_interpreter2: public Cetaguard_message_interpreter{
  void button_msg(Cetaguard_msg_contents contents, Cetaguard_index idx){
  }
  void pairing_without_secret_msg(Cetaguard_index idx){
  }
  void pairing_with_secret_msg(Cetaguard_index idx){
  }
};

My_interpreter2 interpreter2;

void benchmark(){
  Cetaguard_status ret;

  Cetaguard_transmitter txTest;
  uint32_t cycles_start, cycles;
  cycles_start = DWT->CYCCNT;
  Cetaguard_receiver rxTest(&interpreter2);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Creating rx: %lu\n", cycles);

  cycles_start = DWT->CYCCNT;
  uint32_t id_out;
  ret = rxTest.add_pending_transmitter(&id_out);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("adding pending tx: %lu\n", cycles);
  SERIAL_PRINTF("\t keygen: %lu\n", debug_data[1] - debug_data[0]);
  SERIAL_PRINTF("\t random: %lu\n", debug_data[2] - debug_data[1]);
  SERIAL_PRINTF("\t hash: %lu\n", debug_data[3] - debug_data[2]);
  if (ret != CETAGUARD_OK){
    return;
  }

  cycles_start = DWT->CYCCNT;
  ret = txTest.add_receiver(rxTest.pending[0].pub_key, rxTest.pending[0].pairing_secret);
  if (ret != CETAGUARD_OK){
    return;
  }

  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Adding rx: %lu\n", cycles);

  uint8_t msg[64];
  size_t size;

  cycles_start = DWT->CYCCNT;
  ret = txTest.prepare_pairing_msg(0, msg, &size);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Prepairing pairing msg: %lu\n", cycles);
  if (ret != CETAGUARD_OK){
    return;
  }

  cycles_start = DWT->CYCCNT;
  ret = rxTest.interpret_message(msg, size);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Interpreting pairing msg: %lu\n", cycles);
  if (ret != CETAGUARD_OK){
    return;
  }

  cycles_start = DWT->CYCCNT;
  ret = txTest.finish_pairing(0);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Finish pairing: %lu\n", cycles);
  if (ret != CETAGUARD_OK){
    return;
  }

  Cetaguard_msg_contents c_in;
  cycles_start = DWT->CYCCNT;
  ret = txTest.prepare_button_msg(0, &c_in, msg, &size);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Preparing button msg: %lu\n", cycles);
  SERIAL_PRINTF("\t prepare: %lu\n", debug_data[1] - debug_data[0]);
  SERIAL_PRINTF("\t hash: %lu\n", debug_data[2] - debug_data[1]);
  SERIAL_PRINTF("\t encrypt: %lu\n", debug_data[3] - debug_data[2]);
  if (ret != CETAGUARD_OK){
    return;
  }

  cycles_start = DWT->CYCCNT;
  ret = rxTest.interpret_message(msg, size);
  cycles = DWT->CYCCNT - cycles_start;
  SERIAL_PRINTF("Interpreting button msg: %lu\n", cycles);
  SERIAL_PRINTF("\t decrypt: %lu\n", debug_data[1] - debug_data[0]);
  if (ret != CETAGUARD_OK){
    return;
  }

}

void stress_test_calc(){
	uint8_t pub1[PUB_KEY_SIZE], pub2[PUB_KEY_SIZE], priv1[PRIV_KEY_SIZE], priv2[PRIV_KEY_SIZE], secret[SECRET_SIZE];
	generate_keys(pub1, priv1);
	generate_keys(pub2, priv2);
	calculate_secret(pub1, priv2, secret);
	hash(priv1, PRIV_KEY_SIZE, recv_buffer);
	for (int i = 0; i < 6; i++){
		encrypt(recv_buffer, recv_buffer, secret);
	}
	for (int i = 0; i < 6; i++){
		decrypt(priv1, priv1, secret);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin){
	if (pin & (Btn1_Pin | Btn2_Pin | Btn3_Pin | Btn4_Pin)){
		status.btn_intr = 1;
	}
	if (pin & Radio_PKT_Pin){
		if (READ_PIN_L(Radio_PKT)){
			if (status.expecting_rx){
				status.got_packet = 1;
			}
		}
		else {
			if (status.waiting_tx){
				status.waiting_tx = 0;
			}
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){

}

static const int ADC_resolution = 12;
static const uint16_t ADC_max = (1 << 12) - 1;
volatile int errors = 0;

uint8_t measure_battery_voltage(){
	HAL_ADC_Start(&hadc1);
	HAL_StatusTypeDef retval;
	do {
		retval = HAL_ADC_PollForConversion(&hadc1, 10);
	} while (retval != HAL_OK && retval != HAL_TIMEOUT);
	if (retval == HAL_TIMEOUT){
		return 0;
	}
	uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
	float voltage = ((VREFINT_CAL_VREF / 1000.0f) * (*VREFINT_CAL_ADDR)) / adc_value;
	voltage = (voltage - 2.2) * (256/0.8f);
	voltage = voltage < 0 ? 0 : voltage;
	voltage = voltage > 255 ? 255 : voltage;
	uint8_t result = voltage;
	HAL_ADC_Stop(&hadc1);
	return result;
}

void set_RTC_timestamp(time_t now){
	 RTC_TimeTypeDef sTime;
	 RTC_DateTypeDef sDate;

	 struct tm time_tm;
	 time_tm = *(localtime(&now));

	 sTime.Hours = (uint8_t)time_tm.tm_hour;
	 sTime.Minutes = (uint8_t)time_tm.tm_min;
	 sTime.Seconds = (uint8_t)time_tm.tm_sec;
	 if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK){
		 Error_Handler();
	 }

	 if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; } // the chip goes mon tue wed thu fri sat sun
	 sDate.WeekDay = (uint8_t)time_tm.tm_wday;
	 sDate.Month = (uint8_t)time_tm.tm_mon+1; //month 1- This is why date math is frustrating.
	 sDate.Date = (uint8_t)time_tm.tm_mday;
	 sDate.Year = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

	 /*
	 * update the RTC
	 */
	 if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK){
		 Error_Handler();
	 }

	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2); // lock it in with the backup registers
}

void start_radio(){
	WRITE_PIN_L(Radio_RST, 1);
	HAL_Delay(5);
	radio.begin();
	radio.setChannel(0x20);
}

void shutdown_radio(){
	WRITE_PIN_L(Radio_RST, 0);
}

bool send_message_buffer(size_t msg_size){
	status.waiting_tx = 1;
	radio.sendPacket(message_buffer, msg_size);
	uint32_t timeout = HAL_GetTick() + 5;
	while (READ_PIN_L(Radio_PKT)){
		if (HAL_GetTick() > timeout){
			shutdown_radio();
			HAL_Delay(10);
			start_radio();
			break;
		}
	}
	return true;
}

const uint32_t PAIRING_TIMEOUT_MAX = 5000;
uint32_t pairing_wait_timeout = 0;
void try_pairing(){
	status.pairing_mode = 1;
	message_buffer[3] = 0x18;
	send_message_buffer(4);
	status.expecting_rx = 1;
	radio.startListening();
	pairing_wait_timeout = HAL_GetTick() + PAIRING_TIMEOUT_MAX;
}

void stop_pairing(){
	status.expecting_rx = 0;
	status.pairing_mode = 0;
	radio.stopListening();
}

void enter_standby(){
	/* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
	mainly when using more than one wakeup source this is to not miss any wakeup event.
	 - Disable all used wakeup sources,
	 - Clear all related wakeup flags,
	 - Re-enable all used wakeup sources,
	 - Enter the Standby mode.
	*/

	/*  For power consumption's sake, appropriately configure the GPIO corresponding to
	  the wake-up pin, fill up the pull-down control register and set the APC bit. */
	radio.sleep();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_0;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_0);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_8); //pull the radio reset low to keep it in low-power state
	HAL_PWREx_EnablePullUpPullDownConfig();

	/* Disable used wakeup source: PWR_WAKEUP_PIN1 */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* Enable wakeup pin WKUP1 */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);

	/* Enter the Standby mode */
	HAL_PWR_EnterSTANDBYMode();
}

bool compareCRC(uint8_t* buffer, size_t length, uint32_t toCompare){
	uint32_t result = HAL_CRC_Calculate(&hcrc, (uint32_t*) buffer, length);
	return result == toCompare;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /*ENABLE CYCLE COUNTING:*/ DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
	status.cold_boot = 0;
	/* Clear Standby flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	/* Blink LED3 to indicate that the system was resumed from Standby mode */
	WRITE_PIN_L(LED, 1);
	HAL_Delay(200);
	WRITE_PIN_L(LED, 0);
	HAL_Delay(200);
	HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_8);
  }
  else {
	  status.cold_boot = 1;
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_LPTIM1_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  WRITE_PIN_L(LED, 1);
  benchmark();
  /*
  //benchmark();
  while ((READ_PIN_L(Btn1) | READ_PIN_L(Btn2)) != (Btn2_Pin | Btn1_Pin)){
	  stress_test_calc();

  }
  WRITE_PIN_L(LED, 0);
  */
  for (int i = 8; i < 64; i++){
	  message_buffer[i] = i;
  }

  int last_buttons_pressed = 0;
  const int PAIRING_MAX_RETRIES = 3;
  int pairing_retries = PAIRING_MAX_RETRIES;
  WRITE_PIN_L(Radio_RST, 1);
  HAL_Delay(10);
  start_radio();
  /*
  HAL_Delay(1000);
  while ((READ_PIN_L(Btn1) | READ_PIN_L(Btn3)) != (Btn3_Pin | Btn1_Pin)){
	  radio.sendPacket(recv_buffer, 56);
  }
  */
  uint32_t time_test = get_time();
  radio.sendPacket((uint8_t*)&time_test, 4);
  WRITE_PIN_L(LED, 1);
  HAL_Delay(1000);
  WRITE_PIN_L(LED, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	WRITE_PIN_L(LED, 1);
	if (status.btn_intr){
		HAL_Delay(2); //2ms debounce
		status.btn_intr = 0;
		int buttons_pressed = 0;
		buttons_pressed |= READ_PIN_L(Btn1);
		buttons_pressed |= READ_PIN_L(Btn2);
		buttons_pressed |= READ_PIN_L(Btn3);
		buttons_pressed |= READ_PIN_L(Btn4);

		switch (buttons_pressed != last_buttons_pressed ? buttons_pressed : 0){
		case Btn4_Pin: {
			uint32_t time_test = get_time();
			radio.sendPacket((uint8_t*)&time_test, 4);
		}
		break;
		case Btn4_Pin | Btn1_Pin:{
			pairing_retries = PAIRING_MAX_RETRIES;
			try_pairing();
		}
		break;
		case Btn4_Pin | Btn2_Pin:{
			stop_pairing();
		}
		break;
		case Btn4_Pin | Btn3_Pin:{
			HAL_Delay(1000);
			enter_standby();
		}
		break;
		case Btn1_Pin:
		case Btn2_Pin:
		case Btn3_Pin:{
		  if (!status.pairing_mode && !tx.receivers.empty()){
			  Cetaguard_msg_contents contents;
			  contents.battery = measure_battery_voltage();
			  contents.buttons = buttons_pressed;
			  size_t size_out;
			  Cetaguard_status r = tx.prepare_button_msg(0, &contents, message_buffer, &size_out);
			  if (r != CETAGUARD_OK){
				  last_error = r;
				  Error_Handler();
			  }
			  else {
				  send_message_buffer(size_out);
			  }
		  }
		  else if (status.pairing_mode && !tx.pending.empty()){
			  if (buttons_pressed == Btn1_Pin){
				  size_t size_out;
				  Cetaguard_status r = tx.prepare_pairing_msg(0, message_buffer, &size_out);
				  if (r != CETAGUARD_OK){
					  last_error = r;
					  Error_Handler();
				  }
				  else {
					  send_message_buffer(size_out);
				  }
			  }
			  else if (buttons_pressed == Btn2_Pin){
				  tx.finish_pairing(0);
			  }
		  }
		}
		break;
		}
		last_buttons_pressed = buttons_pressed;
		status.btn_intr = 0;
	}
	if (status.expecting_rx && HAL_GetTick() > pairing_wait_timeout){
		status.expecting_rx = 0;
		radio.stopListening();
	}
	if (status.got_packet){
		status.got_packet = 0;
		int pkt_length = radio.read(recv_buffer, 64);
		const int expected_length = PUB_KEY_SIZE + 8;
		if (pkt_length >= expected_length &&
			recv_buffer[0] == 0x01 &&
			recv_buffer[1] == 0x02 &&
			recv_buffer[2] == 0x03 &&
			recv_buffer[3] == 0x18 &&
			compareCRC(recv_buffer + 8, PUB_KEY_SIZE, ((uint32_t*)recv_buffer)[1]) &&
			tx.add_receiver(recv_buffer + 8) == CETAGUARD_OK){
			status.expecting_rx = 0;
		}
		else {
			if (pairing_retries > 0){
				pairing_retries--;
				HAL_Delay(100);
				try_pairing();
			}
			else {
				stop_pairing();
			}
		}
		char buf[4];
		for (int i = 0; i < pkt_length; i++){
			snprintf(buf,4," %02x", recv_buffer[i]);
			HAL_UART_Transmit(&huart2, (uint8_t*)buf, 3, 10);
		}
		snprintf(buf,4,"\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)buf, 2, 10);
	}

	WRITE_PIN_L(LED, 0);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
  /** Enable the SYSCFG APB clock 
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS 
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (status.cold_boot){
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 2;
  sDate.Year = 1;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Radio_RST_GPIO_Port, Radio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Radio_PKT_Pin */
  GPIO_InitStruct.Pin = Radio_PKT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Radio_PKT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NCS_Pin */
  GPIO_InitStruct.Pin = SPI1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Radio_RST_Pin */
  GPIO_InitStruct.Pin = Radio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Radio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn1_Pin Btn2_Pin Btn3_Pin Btn4_Pin */
  GPIO_InitStruct.Pin = Btn1_Pin|Btn2_Pin|Btn3_Pin|Btn4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
	total_errors++;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
