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

#define N_MUESTRAS 20
#define N_EJES 3

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
	ON
};

//Estados FSM Lectura-Espera
enum lecture_state_x{
	ESPERA,
	LECTURA,
	EVALUACION_X,
	EVALUACION_Y,
	EVALUACION_Z,
	SALIDA_X,
	SALIDA_Y,
	SALIDA_Z
};

//entradas
static int16_t *sensor;

//variables
static int16_t *lectura_x, *lectura_y, *lectura_z;
static uint8_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, muestra = 0;
static uint8_t timer_boton = 1, timer_led = 0;
static uint8_t end_temp_lectura = 0;

//variables compartidas
static uint8_t activado = 0;

//salidas
/*static uint8_t faultx = 0, warningx = 0, normalx = 0;
static uint8_t faulty = 0, warningy = 0, normaly = 0;
static uint8_t faultz = 0, warningz = 0, normalz = 0;
static uint8_t led_azul = 0;*/

//funciones de transicion
static int boton_presionado (fsm_t* this) { if (timer_boton && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) return 1; else return 0; }
static int boton_no_presionado (fsm_t* this) { if (timer_boton && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) return 1; else return 0; }
static int led_on (fsm_t* this) { if(timer_led == 1) return 1; else return 0; }
static int led_off (fsm_t* this) { if(timer_led == 2) return 1; else return 0;}

static int activado_on (fsm_t* this) { return activado; }
static int activado_off (fsm_t* this) { return !activado; }

static int muestra_acc (fsm_t* this)  {if ((muestra < N_MUESTRAS) && end_temp_lectura && activado ) return 1; else return 0;}
static int muestra_max (fsm_t* this)  {if ((muestra >= N_MUESTRAS) && activado) return 1; else return 0;}
static int max_x (fsm_t* this)  {if ((lectura_x[muestra] >= lectura_x[a]) && activado) return 1; else return 0;}
static int min_x (fsm_t* this)  {if ((lectura_x[muestra] < lectura_x[b]) && activado) return 1; else return 0;}
static int med_x (fsm_t* this)  {if ((lectura_x[muestra] >= lectura_x[b]) && (lectura_x[muestra] < lectura_x[a]) && activado) return 1; else return 0;}
static int max_y (fsm_t* this)  {if ((lectura_y[muestra] >= lectura_y[c]) && activado) return 1; else return 0;}
static int min_y (fsm_t* this)  {if ((lectura_y[muestra] < lectura_y[d]) && activado) return 1; else return 0;}
static int med_y (fsm_t* this)  {if ((lectura_y[muestra] >= lectura_y[d]) && (lectura_y[muestra] < lectura_y[c]) && activado) return 1; else return 0;}
static int max_z_muestra (fsm_t* this)  {if ((lectura_z[muestra] >= lectura_z[e]) && (muestra < N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int min_z_muestra (fsm_t* this)  {if ((lectura_z[muestra] < lectura_z[f]) && (muestra < N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int med_z_muestra (fsm_t* this)  {if ((lectura_z[muestra] >= lectura_z[f]) && (lectura_z[muestra] < lectura_z[e]) && (muestra < N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int max_z_fin (fsm_t* this)  {if ((lectura_z[muestra] >= lectura_z[e]) && (muestra >= N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int min_z_fin (fsm_t* this)  {if ((lectura_z[muestra] < lectura_z[f]) && (muestra >= N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int med_z_fin (fsm_t* this)  {if ((lectura_z[muestra] >= lectura_z[f]) && (lectura_z[muestra] < lectura_z[e]) && (muestra >= N_MUESTRAS - 1) && activado) return 1; else return 0;}
static int resta_x_fault (fsm_t* this)  {if (((lectura_x[a] - lectura_x[b]) >= FAULT) && activado) return 1; else return 0;}
static int resta_x_warning (fsm_t* this)  {if (((lectura_x[a] - lectura_x[b]) < FAULT) && ((lectura_x[a] - lectura_x[b]) > WARNING) && activado) return 1; else return 0;}
static int resta_x_normal (fsm_t* this)  {if (((lectura_x[a] - lectura_x[b]) <= WARNING) && activado) return 1; else return 0;}
static int resta_y_fault (fsm_t* this)  {if (((lectura_y[c] - lectura_y[d]) >= FAULT) && activado) return 1; else return 0;}
static int resta_y_warning (fsm_t* this)  {if (((lectura_y[c] - lectura_y[d]) < FAULT) && ((lectura_y[c] - lectura_y[d]) > WARNING) && activado) return 1; else return 0;}
static int resta_y_normal (fsm_t* this)  {if (((lectura_y[c] - lectura_y[d]) <= WARNING) && activado) return 1; else return 0;}
static int resta_z_fault (fsm_t* this)  {if (((lectura_z[e] - lectura_z[f]) >= FAULT) && activado && end_temp_lectura) return 1; else return 0;}
static int resta_z_warning (fsm_t* this)  {if (((lectura_z[e] - lectura_z[f]) < FAULT) && ((lectura_z[e] - lectura_z[f]) > WARNING) && activado && end_temp_lectura) return 1; else return 0;}
static int resta_z_normal (fsm_t* this)  {if (((lectura_z[e] - lectura_z[f]) <= WARNING) && activado && end_temp_lectura) return 1; else return 0;}

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
  timer_boton = 0;
  HAL_TIM_Base_Start_IT(&htim9); //Temporizador boton
  HAL_TIM_Base_Start_IT(&htim6); //Temporizador del led azul
}

static void desactivacion_inicio (fsm_t* this)
{
  activado = 0;
  timer_boton = 0;
  HAL_TIM_Base_Stop_IT(&htim6); //Temporizador del led azul
  timer_led = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
}

static void desactivacion_lectura (fsm_t* this)
{
  muestra = 0;
  /*faultx = 0;
  warningx = 0;
  normalx = 0;
  faulty = 0;
  warningy = 0;
  normaly = 0;
  faultz = 0;
  warningz = 0;
  normalz = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  e = 0;
  f = 0;
  end_temp_lectura = 0;
}

static void fin_lectura (fsm_t* this)
{
  muestra = 0;
}


static void led_activado (fsm_t* this)
{
  //led_azul = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
}

static void led_desactivado (fsm_t* this)
{
  timer_led = 0;
  //led_azul = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
}

static void lectura_ejes (fsm_t* this)
{
  BSP_ACCELERO_GetXYZ(sensor);

  lectura_x[muestra] = sensor[0];
  lectura_y[muestra] = sensor[1];
  lectura_z[muestra] = sensor[2];
  muestra++;

  end_temp_lectura = 0;
}

static void act_max_x (fsm_t* this)
{
  a = muestra;
}

static void act_min_x (fsm_t* this)
{
  b = muestra;
}

static void act_max_y (fsm_t* this)
{
  c = muestra;
}

static void act_min_y (fsm_t* this)
{
  d = muestra;
}

static void act_max_z_muestra (fsm_t* this)
{
  e = muestra;
  muestra++;
}

static void act_min_z_muestra (fsm_t* this)
{
  f = muestra;
  muestra++;
}

static void act_med_z_muestra (fsm_t* this)
{
  muestra++;
}

static void act_max_z_fin (fsm_t* this)
{
  e = muestra;
}

static void act_min_z_fin (fsm_t* this)
{
  f = muestra;
}

static void fault_x (fsm_t* this)
{
  /*faultx = 1;
  warningx = 0;
  normalx = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, FAULT);
}

static void warning_x (fsm_t* this)
{
  /*faultx = 0;
  warningx = 1;
  normalx = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, WARNING);
}

static void normal_x (fsm_t* this)
{
  /*faultx = 0;
  warningx = 0;
  normalx = 1;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, NORMAL);
}

static void fault_y (fsm_t* this)
{
  /*faulty = 1;
  warningy = 0;
  normaly = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, FAULT);

}

static void warning_y (fsm_t* this)
{
  /*faulty = 0;
  warningy = 1;
  normaly = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, WARNING);
}

static void normal_y (fsm_t* this)
{
  /*faulty = 0;
  warningy = 0;
  normaly = 1;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NORMAL);
}

static void fault_z (fsm_t* this)
{
  /*faultz = 1;
  warningz = 0;
  normalz = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, FAULT);
  muestra = 0;
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  e = 0;
  f = 0;
}

static void warning_z (fsm_t* this)
{
  /*faultz = 0;
  warningz = 1;
  normalz = 0;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, WARNING);
  muestra = 0;
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  e = 0;
  f = 0;
}

static void normal_z (fsm_t* this)
{
  /*faultz = 0;
  warningz = 0;
  normalz = 1;*/
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NORMAL);
  muestra = 0;
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  e = 0;
  f = 0;
}

static fsm_trans_t inicio[] = {
  { OFF, boton_presionado, ON, activacion},
  { ON, boton_no_presionado, OFF,  desactivacion_inicio },
  { ON, led_on, ON,  led_activado },
  { ON, led_off, ON,  led_desactivado },
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t lectura_acc[] = {
  { ESPERA, activado_on, LECTURA, 0},
  { LECTURA, activado_off, ESPERA, desactivacion_lectura},
  { LECTURA, muestra_acc, LECTURA, lectura_ejes},
  { LECTURA, muestra_max, EVALUACION_X, fin_lectura},
  { EVALUACION_X, max_x, EVALUACION_Y, act_max_x},
  { EVALUACION_X, min_x, EVALUACION_Y, act_min_x},
  { EVALUACION_X, med_x, EVALUACION_Y, 0},
  { EVALUACION_X, activado_off, ESPERA, desactivacion_lectura},
  { EVALUACION_Y, max_y, EVALUACION_Z, act_max_y},
  { EVALUACION_Y, min_y, EVALUACION_Z, act_min_y},
  { EVALUACION_Y, med_y, EVALUACION_Z, 0},
  { EVALUACION_Y, activado_off, ESPERA, desactivacion_lectura},
  { EVALUACION_Z, max_z_muestra, EVALUACION_X, act_max_z_muestra},
  { EVALUACION_Z, min_z_muestra, EVALUACION_X, act_min_z_muestra},
  { EVALUACION_Z, med_z_muestra, EVALUACION_X, act_med_z_muestra},
  { EVALUACION_Z, max_z_fin, SALIDA_X, act_max_z_fin},
  { EVALUACION_Z, min_z_fin, SALIDA_X, act_min_z_fin},
  { EVALUACION_Z, med_z_fin, SALIDA_X, 0},
  { EVALUACION_Z, activado_off, ESPERA, desactivacion_lectura},
  { SALIDA_X, resta_x_fault, SALIDA_Y, fault_x},
  { SALIDA_X, resta_x_warning, SALIDA_Y, warning_x},
  { SALIDA_X, resta_x_normal, SALIDA_Y, normal_x},
  { SALIDA_X, activado_off, ESPERA, desactivacion_lectura},
  { SALIDA_Y, resta_y_fault, SALIDA_Z, fault_y},
  { SALIDA_Y, resta_y_warning, SALIDA_Z, warning_y},
  { SALIDA_Y, resta_y_normal, SALIDA_Z, normal_y},
  { SALIDA_Y, activado_off, ESPERA, desactivacion_lectura},
  { SALIDA_Z, resta_z_fault, LECTURA, fault_z},
  { SALIDA_Z, resta_z_warning, LECTURA, warning_z},
  { SALIDA_Z, resta_z_normal, LECTURA, normal_z},
  { SALIDA_Z, activado_off, ESPERA, desactivacion_lectura},
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

  lectura_x = malloc(N_MUESTRAS * sizeof(int16_t));
  lectura_y = malloc(N_MUESTRAS * sizeof(int16_t));
  lectura_z = malloc(N_MUESTRAS * sizeof(int16_t));

  //Acelerómetro
  if(BSP_ACCELERO_Init() != HAL_OK){
	  Error_Handler();
  }

  //Temporizadores

  HAL_TIM_Base_Start_IT(&htim7); //Temporizador para hacer las lecturas
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  //Creación de las FSM
  fsm_t* fsm_inicio = fsm_new (inicio);
  fsm_t* fsm_lectura = fsm_new (lectura_acc);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    fsm_fire (fsm_inicio);
    fsm_fire (fsm_lectura);
  }

  free(sensor);
  free(lectura_x);
  free(lectura_y);
  free(lectura_z);

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
		timer_led++;
	if(htim->Instance==TIM7)
		end_temp_lectura = 1;
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
