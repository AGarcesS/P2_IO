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

#define N_MUESTRAS 20
#define N_EJES 3

#define TH_HIGH 200
#define TH_LOW 100

#define FAULT 90
#define WARNING 60
#define NORMAL 30

#define CANAL_EJE TIM_CHANNEL_1+eje*4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Estados FSM ON-OFF
enum start_state {
	OFF,
	BOTON_PULSADO,
	ON
};

//Estados FSM Lectura-Espera
enum lecture_state{
	STOP,
	MUESTREO,
	MAX_MIN,
	CONTADOR_MUESTRA,
	SALIDA,
	GRADO
};

enum ejes{
	x,
	y,
	z
};

uint32_t ciclos;

//entradas
static int16_t *sensor;

//variables
static int16_t **lectura, *max, *min;
static uint8_t muestra = 0, eje = 0;
volatile static uint8_t timer_boton = 1, led_azul = 0, timer_lectura = 0;

//variables compartidas
static uint8_t activado = 0;


//funciones de transicion
static int boton_presionado_on (fsm_t* this) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) return 1; else return 0; }
static int boton_no_presionado_on (fsm_t* this) { if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) && activado) return 1; else return 0; }
static int boton_presionado_off (fsm_t* this) { if (timer_boton && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) return 1; else return 0; }
static int boton_no_presionado_off (fsm_t* this) { if (timer_boton && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) return 1; else return 0; }


static int activado_on (fsm_t* this) { return activado; }
static int activado_off (fsm_t* this) { return !activado; }

static int muestreo_listo (fsm_t* this)  {if ((muestra < N_MUESTRAS) && timer_lectura && activado ) return 1; else return 0;}
static int muestra_max (fsm_t* this)  {if ((muestra >= N_MUESTRAS) && activado ) return 1; else return 0;}
static int eje_no_max (fsm_t* this)  {if ((eje < N_EJES - 1) && activado ) return 1; else return 0;}
static int eje_max_fin (fsm_t* this)  {if ((eje >= N_EJES - 1) && activado ) return 1; else return 0;}
static int muestra_no_max (fsm_t* this)  {if ((muestra < N_MUESTRAS - 1) && activado ) return 1; else return 0;}
static int eje_no_listo (fsm_t* this)  {if ((muestra >= N_MUESTRAS - 1) && (eje < N_EJES - 1) && activado ) return 1; else return 0;}
static int eje_listo (fsm_t* this)  {if ((muestra >= N_MUESTRAS - 1) && (eje >= N_EJES - 1) && activado ) return 1; else return 0;}
static int max_muestra (fsm_t* this)  {if ((lectura[eje][muestra] >= max[eje]) && activado) return 1; else return 0;}
static int min_muestra (fsm_t* this)  {if ((lectura[eje][muestra] < min[eje]) && activado) return 1; else return 0;}
static int med_muestra (fsm_t* this)  {if ((lectura[eje][muestra] >= min[eje]) && (lectura[eje][muestra] < max[eje]) && activado) return 1; else return 0;}
static int out_fault (fsm_t* this)  {if (((max[eje] - min[eje]) >= TH_HIGH) && activado) return 1; else return 0;}
static int out_warning (fsm_t* this)  {if (((max[eje] - min[eje]) < TH_HIGH) && ((max[eje] - min[eje]) > TH_LOW) && activado) return 1; else return 0;}
static int out_normal (fsm_t* this)  {if (((max[eje] - min[eje]) <= TH_LOW) && activado) return 1; else return 0;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void activacion_inicio (fsm_t* this)
{
  activado = 1;
  timer_boton=0;
  led_azul=0;
  __HAL_TIM_SET_COUNTER(&htim9,0); //Reinicio a cero del temporizador del boton
  __HAL_TIM_SET_COUNTER(&htim6,0); //Reinicio a cero del temporizador del led
  HAL_TIM_Base_Start_IT(&htim9); //Temporizador boton
  HAL_TIM_Base_Start_IT(&htim6); //Temporizador del led azul
}

static void desactivacion_inicio (fsm_t* this)
{
  activado = 0;
  timer_boton = 0;
  led_azul=0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
  HAL_TIM_Base_Stop_IT(&htim6); //Temporizador del led azul stop
}

static void reinicio_inicio (fsm_t* this)
{
  HAL_TIM_Base_Stop_IT(&htim9); //Temporizador boton
  HAL_TIM_Base_Stop_IT(&htim6); //Temporizador del led azul
  timer_boton = 0;
}

static void toggle_led (fsm_t* this)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, led_azul);
}

static void desactivacion_muestreo (fsm_t* this)
{
  muestra = 0;
  eje = 0;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  HAL_TIM_Base_Stop_IT(&htim7);
  free(sensor);
  for (int i = 0; i < N_EJES; ++i) {
	  free(lectura[i]);
  }
  free(lectura);
  free(max);
  free(min);
}

static void preparacion_muestreo (fsm_t* this)
{
  timer_lectura=0;
  __HAL_TIM_SET_COUNTER(&htim7,0); //Reinicio a cero del temporizador de muestreo
  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas

  sensor = malloc(N_EJES * sizeof(uint16_t));
  lectura = malloc(N_EJES * sizeof(int16_t*));
  for (int i = 0; i < N_EJES; ++i) {
      lectura[i] = malloc(N_MUESTRAS * sizeof(int16_t));
  }
  max = malloc(N_EJES * sizeof(int16_t));
  min = malloc(N_EJES * sizeof(int16_t));
}

static void muestreo_ejes (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(sensor);
  lectura[x][muestra] = sensor[x];
  lectura[y][muestra] = sensor[y];
  lectura[z][muestra] = sensor[z];
  muestra++;

  timer_lectura = 0;
  __HAL_TIM_SET_COUNTER(&htim9,0); //Reinicio a cero del temporizador del boton
}

static void cambio_eje (fsm_t* this)
{
  eje++;
  muestra = 0;
}

static void fin_muestreo (fsm_t* this)
{
  eje = 0;
  muestra = 1;
  for(int i = x; i < N_EJES; i++){
	  max[i] = lectura[i][0];
	  min[i] = lectura[i][0];
  }
  HAL_TIM_Base_Stop_IT(&htim7); //Temporizador para hacer las lecturas
  __HAL_TIM_SET_COUNTER(&htim7,0); //Reinicio a cero del temporizador de lecturas
}

static void salida_max (fsm_t* this)
{
  max[eje] = lectura[eje][muestra];
}


static void salida_min (fsm_t* this)
{
  min[eje] = lectura[eje][muestra];
}

static void fin_lectura (fsm_t* this)
{
  eje = 0;
  muestra = 0;
  __HAL_TIM_SET_COUNTER(&htim7,0); //Reinicio a cero del temporizador de lecturas
  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas
}

static void cambio_muestra (fsm_t* this)
{
  muestra++;
}

static void preparacion_salida (fsm_t* this)
{
  eje = 0;
  muestra = 0;
}


static void salida_fault (fsm_t* this)
{
	__HAL_TIM_SET_COMPARE(&htim4, CANAL_EJE, FAULT);
}

static void salida_warning (fsm_t* this)
{
	__HAL_TIM_SET_COMPARE(&htim4, CANAL_EJE, WARNING);
}

static void salida_normal (fsm_t* this)
{
	__HAL_TIM_SET_COMPARE(&htim4, CANAL_EJE, NORMAL);
}

static fsm_trans_t inicio[] = {
  { OFF, boton_presionado_on, BOTON_PULSADO, activacion_inicio},
  { BOTON_PULSADO, boton_no_presionado_on, ON, 0},
  {BOTON_PULSADO,activado_on,BOTON_PULSADO,toggle_led},
  { ON, boton_presionado_off, BOTON_PULSADO,  desactivacion_inicio },
  { ON, activado_on, ON, toggle_led},
  { BOTON_PULSADO, boton_no_presionado_off, OFF, reinicio_inicio },
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t muestreo_acc[] = {
  { STOP, activado_on, MUESTREO, preparacion_muestreo},
  { MUESTREO, activado_off, STOP, desactivacion_muestreo},
  { MUESTREO, muestreo_listo, MUESTREO, muestreo_ejes},
  { MUESTREO, muestra_max, MAX_MIN, fin_muestreo},
  { MAX_MIN, activado_off, STOP, desactivacion_muestreo},
  { MAX_MIN, max_muestra, CONTADOR_MUESTRA, salida_max},
  { MAX_MIN, min_muestra, CONTADOR_MUESTRA, salida_min},
  { MAX_MIN, med_muestra, CONTADOR_MUESTRA, 0},
  { CONTADOR_MUESTRA, activado_off, STOP, desactivacion_muestreo},
  { CONTADOR_MUESTRA, muestra_no_max, MAX_MIN, cambio_muestra},
  { CONTADOR_MUESTRA, eje_no_listo, MAX_MIN, cambio_eje},
  { CONTADOR_MUESTRA, eje_listo, SALIDA, preparacion_salida},
  { SALIDA, activado_off, STOP, desactivacion_muestreo},
  { SALIDA, out_fault, GRADO, salida_fault},
  { SALIDA, out_warning, GRADO, salida_warning},
  { SALIDA, out_normal, GRADO, salida_normal},
  { GRADO, activado_off, STOP, desactivacion_muestreo},
  { GRADO, eje_no_max, SALIDA, cambio_eje},
  { GRADO, eje_max_fin, MUESTREO, fin_lectura},
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
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  KIN1_InitCycleCounter();

  //Acelerómetro
  if(BSP_ACCELERO_Init() != HAL_OK){
	  Error_Handler();
  }

  //Temporizadores

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  //Creación de las FSM
  fsm_t* fsm_inicio = fsm_new (inicio);
  fsm_t* fsm_lectura = fsm_new (muestreo_acc);

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
    fsm_fire (fsm_lectura);
    HAL_Delay(1);

    ciclos = KIN1_GetCycleCounter();
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM6){
		if(led_azul){led_azul=0;}
		else {led_azul=1;}
	}
	if(htim->Instance==TIM7)
		timer_lectura = 1;
	if(htim->Instance==TIM9)
		timer_boton = 1;
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
