/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "stm32f4_discovery_accelerometer.h"
#include "fsm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIEMPO_BOTON 500

#define N_LECTURA 20
#define N_EJE 3

#define TH_HIGH 200
#define TH_LOW 100

#define FAULT 90
#define WARNING 60
#define NORMAL 30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Estados FSM ON-OFF
enum start_state {
	OFF,
	ON,
};

//Estados FSM Lectura-Espera
enum lecture_state_x{
	ESPERA_X,
	LECTURA_X,
};

enum lecture_state_y{
	ESPERA_Y,
	LECTURA_Y,
};

enum lecture_state_z{
	ESPERA_Z,
	LECTURA_Z,
};

//Estados FSM LED azul
enum led_state{
	LED_OFF,
	LED_ON
};

static uint32_t cycles;

static int16_t *lectura;
static uint8_t i = 0, j = 0, k = 0;

//entradas
static uint8_t boton = 0;
static int16_t sensorx, sensory, sensorz;
static int16_t *sensorx_v, *sensory_v, *sensorz_v;
static uint8_t timer_boton = 1, timer_lectura = 0, timer_led = 0;

//variables compartidas
static uint8_t activado = 0;

//salidas
static uint8_t faultx=0, warningx=0, normalx=0;
static uint8_t faulty=0, warningy=0, normaly=0;
static uint8_t faultz=0, warningz=0, normalz=0;
static uint8_t led_azul=0;

//funciones de transicion
static int boton_presionado (fsm_t* this) { if (timer_boton && boton) return 1; else return 0; }
static int boton_no_presionado (fsm_t* this) { if (timer_boton && !boton) return 1; else return 0; }

static int activado_on (fsm_t* this) { if (timer_lectura && activado) return 1; else return 0; }
static int activado_off (fsm_t* this) { if (!timer_lectura || !activado) return 1; else return 0; }
static int activado_on_led (fsm_t* this) { if (timer_led && activado) return 1; else return 0; }
static int activado_off_led (fsm_t* this) { if (!timer_led||!activado) return 1; else return 0; }

static int defecto (fsm_t* this)  {return 1;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void activacion (fsm_t* this)
{
  activado = 1;
}

static void desactivacion (fsm_t* this)
{
  activado = 0;

  faultx = 0;
  warningx = 0;
  normalx = 0;

  faulty = 0;
  warningy = 0;
  normaly = 0;

  faultz = 0;
  warningz = 0;
  normalz = 0;
}

static void lectura_x (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(lectura);
  sensorx = lectura[0];
  if(i < N_LECTURA){
	  sensorx_v[i] = sensorx;
	  i++;
  }
  else{
	  faultx = 0;
	  warningx = 0;
	  normalx = 0;
	  i = 0;
	  uint8_t b = 0, c = 0;
	  for(uint8_t a = 0; a < N_LECTURA; a++){
		  if(sensorx_v[a] < sensorx_v[b]) //mínimo
			  b = a;
		  else if(sensorx_v[a] > sensorx_v[c]) //máximo
			  c = a;
	  }

	  int16_t dif_x = sensorx_v[c] - sensorx_v[b];
	  if(dif_x > TH_HIGH)
		  faultx = 1;
	  else if(dif_x > TH_LOW)
		  warningx = 1;
	  else
		  normalx = 1;
  }
}

static void lectura_y (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(lectura);
  sensory = lectura[1];
  if(j < N_LECTURA){
	  sensory_v[j] = sensory;
	  j++;
  }
  else{
	  faulty = 0;
	  warningy = 0;
	  normaly = 0;
	  j = 0;
  	  uint8_t b = 0, c = 0;
  	  for(uint8_t a = 0; a < N_LECTURA; a++){
  		  if(sensory_v[a] < sensory_v[b]) //mínimo
  			  b = a;
  		  else if(sensory_v[a] > sensory_v[c]) //máximo
  			  c = a;
  	  }

  	  int16_t dif_y = sensory_v[c] - sensory_v[b];
  	  if(dif_y > TH_HIGH)
  		  faulty = 1;
  	  else if(dif_y > TH_LOW)
  		  warningy = 1;
  	  else
  		  normaly = 1;
    }
}

static void lectura_z (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(lectura);
  sensorz = lectura[2];
  if(k < N_LECTURA){
	  sensorz_v[k] = sensorz;
	  k++;
  }
  else{
	  faultz = 0;
	  warningz = 0;
	  normalz = 0;
	  k = 0;
  	  uint8_t b = 0, c = 0;
  	  for(uint8_t a = 0; a < N_LECTURA; a++){
  		  if(sensorz_v[a] < sensorz_v[b]) //mínimo
  			  b = a;
  		  else if(sensorz_v[a] > sensorz_v[c]) //máximo
  			  c = a;
  	  }

  	  int16_t dif_z = sensorz_v[c] - sensorz_v[b];
  	  if(dif_z > TH_HIGH)
  		  faultz = 1;
  	  else if(dif_z > TH_LOW)
  		  warningz = 1;
  	  else
  		  normalz = 1;
    }
}

static void lectura_x_fin (fsm_t* this)
{

}

static void lectura_y_fin (fsm_t* this)
{

}

static void lectura_z_fin (fsm_t* this)
{

}

static void led_activado (fsm_t* this)
{
  led_azul = 1;
}

static void led_no_activado (fsm_t* this)
{
  led_azul = 0;
}

static fsm_trans_t inicio[] = {
  { OFF, boton_presionado, ON, activacion},
  { ON, boton_no_presionado, OFF,  desactivacion },
  { ON, defecto, ON,  activacion },
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t lecturax[] = {
  { ESPERA_X, activado_on, LECTURA_X, 0},
  { LECTURA_X, activado_off, ESPERA_X, lectura_x_fin},
  { LECTURA_X, defecto, LECTURA_X, lectura_x},
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t lecturay[] = {
  { ESPERA_Y, activado_on, LECTURA_Y, 0},
  { LECTURA_Y, activado_off, ESPERA_Y, lectura_y_fin},
  { LECTURA_Y, defecto, LECTURA_Y, lectura_y},
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t lecturaz[] = {
  { ESPERA_Z, activado_on, LECTURA_Z, 0},
  { LECTURA_Z, activado_off, ESPERA_Z, lectura_z_fin},
  { LECTURA_Z, defecto, LECTURA_Z, lectura_z},
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t led_activo[] = {
  { LED_OFF, activado_on_led, LED_ON, led_activado},
  { LED_ON, activado_off_led, LED_OFF, led_no_activado},
  { LED_ON, defecto, LED_ON, led_activado},
  {-1, NULL, -1, NULL },
  };

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  KIN1_InitCycleCounter();

  lectura = malloc(N_EJE * sizeof(int16_t));

  sensorx_v = malloc(N_LECTURA * sizeof(int16_t));
  sensory_v = malloc(N_LECTURA * sizeof(int16_t));
  sensorz_v = malloc(N_LECTURA * sizeof(int16_t));

  //Acelerómetro
  if(BSP_ACCELERO_Init() != HAL_OK){
	  Error_Handler();
  }

  //Temporizadores
  HAL_TIM_Base_Start_IT(&htim6); //Temporizador del led azul
  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1); //Temporizador boton
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  //Creación de las FSM
  fsm_t* fsm_inicio = fsm_new (inicio);
  fsm_t* fsm_lectura_x = fsm_new (lecturax);
  fsm_t* fsm_lectura_y = fsm_new (lecturay);
  fsm_t* fsm_lectura_z = fsm_new (lecturaz);
  fsm_t* fsm_led_activo = fsm_new (led_activo);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    KIN1_ResetCycleCounter();
    KIN1_EnableCycleCounter();

    fsm_fire (fsm_inicio);
    fsm_fire (fsm_lectura_x);
    fsm_fire (fsm_lectura_y);
    fsm_fire (fsm_lectura_z);
    fsm_fire (fsm_led_activo);

    //LED SISTEMA ENCENDIDO
    if(led_azul)
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
    else
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);

    //LEDS SENSORES
    if(faultx)
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, FAULT);
    else if(warningx)
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, WARNING);
    else if(normalx)
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, NORMAL);
    else
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

    if(faulty)
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, FAULT);
	else if(warningy)
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, WARNING);
	else if(normaly)
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NORMAL);
	else
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    if(faultz)
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, FAULT);
    else if(warningz)
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, WARNING);
	else if(normalz)
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NORMAL);
	else
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

    cycles = KIN1_GetCycleCounter();

  }

  free(lectura);
  free(sensorx_v);
  free(sensory_v);
  free(sensorz_v);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//Comprobar que viene del timer 2
	if (htim->Instance==TIM2){
		if(boton)
			boton = 0;
		else
			boton = 1;
		if(__HAL_TIM_GetCounter(htim) > TIEMPO_BOTON)
			timer_boton = 1;
		else
			timer_boton = 0;
		__HAL_TIM_SetCounter(htim,0);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM6)
	{
		if(timer_led)
			timer_led = 0;
		else
			timer_led = 1;
	}
	if(htim->Instance==TIM7)
	{
		if(timer_lectura)
			timer_lectura = 0;
		else
			timer_lectura = 1;
	}
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
