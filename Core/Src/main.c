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

#define CICLOS_LED 2

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

enum salida_eje{
	normal,
	warning,
	fault
};

uint32_t ciclos;

//entradas
static int16_t *sensor;

//variables
static int16_t **lectura, *max, *min;
static uint8_t muestra = 0, eje = 0;

//variables compartidas
static uint8_t activado = 0;
static uint8_t timer_boton = 0, temp_led = 0, timer_lectura = 0;

//salidas



//funciones de transicion
static int boton_presionado (fsm_t* this) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) return 1; else return 0; }
static int maquina_ON (fsm_t* this) { if (timer_boton && activado) return 1; else return 0; }
static int maquina_OFF (fsm_t* this) {if(timer_boton&&!activado)return 1; else return0;}

static int led_on (fsm_t* this) { if((temp_led >= CICLOS_LED - 1) && (temp_led < CICLOS_LED)&&activado) return 1; else return 0; }
static int led_off (fsm_t* this) { if((temp_led >= CICLOS_LED)&&activado) return 1; else return 0;}

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
static int fault_out (fsm_t* this)  {if (((max[eje] - min[eje]) >= TH_HIGH) && activado) return 1; else return 0;}
static int warning_out (fsm_t* this)  {if (((max[eje] - min[eje]) < TH_HIGH) && ((max[eje] - min[eje]) > TH_LOW) && activado) return 1; else return 0;}
static int normal_out (fsm_t* this)  {if (((max[eje] - min[eje]) <= TH_LOW) && activado) return 1; else return 0;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void activacion_maquina (fsm_t* this)
{
  activado = 1;
  temp_boton = 0;
  temp_led = 0;
  __HAL_TIM_SET_COUNTER(&htim9,0); //Reinicio a cero del temporizador del boton
  __HAL_TIM_SET_COUNTER(&htim6,0); //Reinicio a cero del temporizador del led
  HAL_TIM_Base_Start_IT(&htim9); //Temporizador boton
  HAL_TIM_Base_Start_IT(&htim6); //Temporizador del led azul
}

static void desactivacion_maquina (fsm_t* this)
{
  activado = 0;
  timer_boton = 0
  __HAL_TIM_SET_COUNTER(&htim9,0); //Reinicio a cero del temporizador del boton
  HAL_TIM_Base_Start_IT(&htim9); //Temporizador boton comienza
  HAL_TIM_Base_Stop_IT(&htim6); //Stop temporizador led
  led_desactivado(this); //Apagar led y reiniciar su cuenta a 0
}

static void temporizacion_boton_off (fsm_t* this)
{
  HAL_TIM_Base_Stop_IT(&htim9); //Stop temporizador boton
  __HAL_TIM_SET_COUNTER(&htim9,0); //Reinicio a cero del temporizador del boton
  timer_boton = 0;
}

static void desactivacion_muestreo (fsm_t* this)
{
  muestra = 0;
  eje = 0;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}

static void led_activado (fsm_t* this)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
}

static void led_desactivado (fsm_t* this)
{
  temp_led = 0;
  __HAL_TIM_SET_COUNTER(&htim6,0); //Reinicio a cero del temporizador del led
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0); //Apagar led
}

static void preparacion_muestreo (fsm_t* this)
{
  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas
}

static void muestreo_ejes (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(sensor);
  lectura[x][muestra] = sensor[x];
  lectura[y][muestra] = sensor[y];
  lectura[z][muestra] = sensor[z];
  muestra++;

  timer_lectura = 0;
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
}

static void fin_lectura (fsm_t* this)
{
  eje = 0;
  muestra = 0;
  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas
}

static void salida_max (fsm_t* this)
{
  max[eje] = lectura[eje][muestra];
}

static void salida_min (fsm_t* this)
{
  min[eje] = lectura[eje][muestra];
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

 switch(eje){
 case 0:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, FAULT);
	faultx = 1;
	warningx = 0;
	normalx = 0;
	break;
 case 1:
 	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, FAULT);
	faulty = 1;
	warningy = 0;
	normaly = 0;
 	break;
 case 2:
 	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, FAULT);
	faultz = 1;
	warningz = 0;
	normalz = 0;
 	break;
 }

}

static void salida_warning (fsm_t* this)
{
  switch(eje){
  case 0:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, WARNING);
	faultx = 0;
	warningx = 1;
	normalx = 0;
	break;
  case 1:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, WARNING);
	faulty = 0;
	warningy = 1;
	normaly = 0;
	break;
  case 2:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, WARNING);
	faultz = 0;
	warningz = 1;
	normalz = 0;
	break;
  }
}

static void salida_normal (fsm_t* this)
{
  switch(eje){
  case 0:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, NORMAL);
	faultx = 0;
	warningx = 0;
	normalx = 1;
	break;
  case 1:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NORMAL);
	faulty = 0;
	warningy = 0;
	normaly = 1;
	break;
  case 2:
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NORMAL);
	faultz = 0;
	warningz = 0;
	normalz = 1;
	break;
  }
}

static fsm_trans_t inicio[] = {
  { OFF, boton_presionado, BOTON_PULSADO, activacion_maquina},
  { BOTON_PULSADO, maquina_ON, ON, temporizacion_boton_off},
  {BOTON_PULSADO,led_on,BOTON_PULSADO,led_activado},
  {BOTON_PULSADO,led_off,BOTON_PULSADO,led_desactivado},
  { ON, boton_presionado, BOTON_PULSADO,  desactivacion_maquina },
  { ON, led_on, ON, led_activado},
  { ON, led_off, ON, led_desactivado},
  { BOTON_PULSADO, maquina_OFF, OFF, temporizacion_boton_off },
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
  { SALIDA, fault, GRADO, salida_fault},
  { SALIDA, warning, GRADO, salida_warning},
  { SALIDA, normal, GRADO, salida_normal},
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

  sensor = malloc(N_EJES * sizeof(int16_t));
  lectura = malloc(N_EJES * sizeof(int16_t*));
  for (uint8_t i = 0; i < N_EJES; i++) {
      lectura[i] = malloc(N_MUESTRAS * sizeof(int16_t));
  }
  max = malloc(N_EJES * sizeof(int16_t));
  min = malloc(N_EJES * sizeof(int16_t));

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

  free(sensor);
  for (uint8_t i = 0; i < N_MUESTRAS; i++) {
      free(lectura[i]);
  }
  free(lectura);
  free(max);
  free(min);

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
	if(htim->Instance==TIM6)
		temp_led++;
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
