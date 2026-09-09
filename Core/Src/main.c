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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
//bisogna distinguere le variabili del sensore tubo e del sensore mano
/* USER CODE BEGIN 0 */
volatile uint8_t time_elapsed = 0;
//volatile uint8_t time_elapsed = 0;
uint16_t counter = 0;

//variabili tubo
uint8_t measure_collected_tubo = 0;
uint32_t measure_tubo = 0;

//per filtro
float alpha = 0.2f;          // peso della nuova lettura (0.2 = molto filtrato, 0.8 = poco filtrato)
float dist_filtrata_tubo = 0.0f;  // distanza pulita


//variabili mano

uint8_t measure_collected_mano = 0;
uint32_t measure_mano = 0;

//per filtro
float dist_filtrata_mano = 0.0f;  // Qui salveremo la distanza pulita

//per seriale
volatile char buff_tx[50];
volatile uint8_t tx_len = 0;
uint8_t tx_len_max = 0;

//per il pid
float Kp = 14.0f;
float Ki = 0.5f;
float Kd = 6.5f;

float pos_pallina_prec = 0.0f;
float integrale = 0.0f;
float no_hand_counter = 0; //da quanto tempo è sparita la mano

// Il valore di PWM in cui la pallina galleggia a metà tubo

#define PWM_BASE 950.0f
#define PWM_MAX 999.0f
#define PWM_MIN 0.0f
#define DT 0.06f

//iniziamo ad impostare i timer


//il timer 6 serve come cronometro per i sensori ad ultrasuoni, mentre il timer 3 ci serve come generatore di PWM
void TIM6_DAC_IRQHandler() {
	if((TIM6->SR >> TIM_SR_UIF_Pos) & 0x01) {
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); // Eliminare richieste di interrupt pendenti
		time_elapsed = 1;
		TIM6->CR1 &= ~(1 << TIM_CR1_CEN_Pos);
		TIM6->SR &= ~(1 << TIM_SR_UIF_Pos);
	}
}



// --- INTERRUPT SENSORI ---
void EXTI1_IRQHandler() { // TUBO
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	if((GPIOC->IDR >> 1) & 0x01) { //Legge il registro dei dati in ingresso (IDR) della porta C, sposta a destra il valore per isolare il bit 1 (PC1) e
		//applica una maschera AND. Questo serve al microcontrollore per capire se l'interrupt è stato generato da un fronte di salita (pin HIGH) o di discesa (pin LOW).
		//se l'impulso echo è andato alto allora :
		TIM6->ARR = 65000; TIM6->CNT = 0; TIM6->CR1 |= (1 << TIM_CR1_CEN_Pos); //accendo il timer
		measure_collected_tubo = 0;
	} else { //il ping acustico è tornato indietro
		TIM6->CR1 &= ~(1 << TIM_CR1_CEN_Pos);
		measure_tubo = TIM6->CNT; measure_collected_tubo = 1; //spengo il timer, salvo il tempo accumulato e measure collected va a 1
	}
	EXTI->PR |= (0x01 << EXTI_PR_PR1_Pos); //azzero il flag di interruzione pendente
}

//questa funzione si comporta esattamente come la precedente con la differenza che in questo caso stiamo gestendo il canale 3 e contiamo la distanza della mano
void EXTI3_IRQHandler() { // MANO
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	if((GPIOC->IDR >> 3) & 0x01) {
		TIM6->ARR = 65000; TIM6->CNT = 0; TIM6->CR1 |= (1 << TIM_CR1_CEN_Pos);
		measure_collected_mano = 0;
	} else {
		TIM6->CR1 &= ~(1 << TIM_CR1_CEN_Pos);
		measure_mano = TIM6->CNT; measure_collected_mano = 1;
	}
	EXTI->PR |= (0x01 << EXTI_PR_PR3_Pos);
}



void TIM6_basic_setup() {

	RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM6EN_Pos); 		// Clock to TIM6

	TIM6->CR1 &= ~(1 << TIM_CR1_UDIS_Pos); 		// Update event enable
	TIM6->CR1 |= (1 << TIM_CR1_URS_Pos);		// Update event generated only by overflow/underflow
	//TIM6->PSC = 90 - 1; 						// CNT incremented every nanoseconds perchè il codice lavora solo se è il clock è 90Mhz quindi 90/90 = 1hz -> frequenza che ci serve per i microsecondi
	TIM6->PSC = 84 - 1;

	TIM6->DIER |= (1 << TIM_DIER_UIE_Pos); 		// Trigger an interrupt every update event
	NVIC_EnableIRQ(TIM6_DAC_IRQn);				// Enable interrupt for TIM6
	NVIC_SetPriority(TIM6_DAC_IRQn, 0); 		// Set Priority
}

void delay_micro_s(uint16_t delay) {
	TIM6->ARR = delay - 1;
	TIM6->CNT = 0;
	TIM6->CR1 |= (1 << TIM_CR1_CEN_Pos);
	while(!time_elapsed) {};
	time_elapsed = 0;
}


//IMPOSTAZIONE SENSORI il primo utilizza GPIO PC1 e PC0 per echo e trig mentre il secondo PC3 e PC2

void hc_sr04_init_tubo() {
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN_Pos);
	RCC->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN_Pos);

	/** Trigger pin configuration (PC0) **/
	GPIOC->MODER |= (0x01 << 0);
	GPIOC->OTYPER &= ~(0x1 << 0);

	/** Echo pin configuration (PC1) **/
	GPIOC->MODER  &= ~(0x03 << 2); // configuriamo il pin come input 00, lo facciamo modificando solo i bit di interesse senza intaccare il resto
	/* SYStem ConFiGuration (Multiplexer) */
	SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PC);

	/* EXTernal Interrupt configuration (EXTI1) */
	EXTI->IMR  |= (EXTI_IMR_IM1);
	EXTI->FTSR |= (0x01 << EXTI_FTSR_TR1_Pos);
	EXTI->RTSR |= (0x01 << EXTI_RTSR_TR1_Pos);

	/* NVIC Configuration */
	NVIC_EnableIRQ(EXTI1_IRQn); //abilitiamo l'interrupt
	NVIC_SetPriority(EXTI1_IRQn, 0); //priorità a 0

	TIM6_basic_setup();
}


void set_trigger_tubo(){
	GPIOC->ODR |= (1 << 0);
}

void clear_trigger_tubo(){
	GPIOC->ODR &= ~(1 << 0);
}

//il secondo sensore sarà destinato alla distanza della mano, in questo caso le gpio coinvolte sono PC2 e PC3

void hc_sr04_init_mano() {
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN_Pos);
	RCC->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN_Pos);

	/** Trigger pin configuration (PC2) **/
	GPIOC->MODER |= (0x01 << 4);   // MODER2 occupa i bit 4 e 5
	GPIOC->OTYPER &= ~(0x1 << 2);

	/** Echo pin configuration (PC3) **/
	GPIOC->MODER  &= ~(0x03 << 6); // MODER3 occupa i bit 6 e 7

	/* SYStem ConFiGuration (Multiplexer) */
	SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PC);

	/* EXTernal Interrupt configuration (EXTI3) */
	EXTI->IMR  |= (EXTI_IMR_IM3);
	EXTI->FTSR |= (0x01 << EXTI_FTSR_TR3_Pos);
	EXTI->RTSR |= (0x01 << EXTI_RTSR_TR3_Pos);

	/* NVIC Configuration */
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_SetPriority(EXTI3_IRQn, 0);

	TIM6_basic_setup();
}



void set_trigger_mano(){
	GPIOC->ODR |= (1 << 2);
}

void clear_trigger_mano() {
	GPIOC->ODR &= ~(1 << 2);
}


volatile float measure_f;

float meas_dist_tubo() {
    measure_collected_tubo = 0; //lo azzero perchè altrimenti potrebbe tener conto dell'ultima misurazione quando tolgo la mano
    clear_trigger_tubo();
    delay_micro_s(3);
    set_trigger_tubo();
    delay_micro_s(10);
    clear_trigger_tubo();

    uint32_t inizio = HAL_GetTick();
    while(!measure_collected_tubo) {
        if((HAL_GetTick() - inizio) > 50) {
            TIM6->CR1 &= ~(1 << TIM_CR1_CEN_Pos);
            return -1.0f;
        }
    }
    return 0.5f * 0.0343f * measure_tubo;
}

float meas_dist_mano() {
    measure_collected_mano = 0;
    clear_trigger_mano();
    delay_micro_s(3);
    set_trigger_mano();
    delay_micro_s(10);
    clear_trigger_mano();

    uint32_t inizio = HAL_GetTick();
    while(!measure_collected_mano) {
        if((HAL_GetTick() - inizio) > 50) {
            TIM6->CR1 &= ~(1 << TIM_CR1_CEN_Pos);
            return -1.0f;
        }
    }
    return 0.5f * 0.0343f * measure_mano;
}

void TIM3_PWM_setup() {
	/* GPIO SetUp (PA6 -> TIM3_CH1) */
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);		// Enable GPIO6
	/*GPIOA->MODER |= (2 << 12); 							// Alternate function mode
	GPIOA->AFR[0] = (2 << 24);
	*/					// Alternate function n2 for PA6 (TIM3 CH1)
	GPIOA->MODER &= ~(3 << 12);
	GPIOA->MODER |=  (2 << 12);

	GPIOA->AFR[0] &= ~(0xF << 24);
	GPIOA->AFR[0] |=  (2 << 24);
	GPIOA->OSPEEDR 	|= (3 << 12);  						// High Speed for PIN PA6

	/* TIM3 SetUp */
	RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM3EN_Pos); 		// Clock to TIM3

	/* CR1 Configuration */
	TIM3->CR1 &= ~(1 << TIM_CR1_UDIS_Pos); 		// Update event enabled
	TIM3->CR1 &= ~(1 << TIM_CR1_URS_Pos);		// Update event generated by any source
	TIM3->CR1 &= ~(1 << TIM_CR1_DIR_Pos); 		// Upcounting direction check
	TIM3->CR1 &= ~(0x03 << TIM_CR1_CMS_Pos); 	// (00) Edge aligned mode check
	TIM3->CR1 |= (1 << TIM_CR1_ARPE_Pos); 		// Auto reload preload enabled

	TIM3->CCMR1 &= ~(0x03 << TIM_CCMR1_CC1S_Pos); 	// TIM3_CH1 configured as output
	TIM3->CCMR1 |= (1 << TIM_CCMR1_OC1PE_Pos); 		// Preload CCR1 register aswell
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); 	// PWM Mode 1

	TIM3->CCER |= (1 << TIM_CCER_CC1E_Pos); 				// Enable output channel 1
	//TIM3->CCER |= (1 << TIM_CCER_CC1P_Pos); 				// Active low



	TIM3->PSC = 42 - 1; 						// CNT incremented every ms
	TIM3->ARR = 1000 - 1; 						// Update event every seconds
	TIM3->CCR1 = 500; //partiamo dalla ventola spenta e poi gli diamo potenza volta per volta



	TIM3->EGR |= (1 << TIM_EGR_UG_Pos); 		// Fire an update event to update shadow registers
}


//usart
void USART2_IRQHandler() {
    NVIC_ClearPendingIRQ(USART2_IRQn);
    if(tx_len < tx_len_max && (USART2->SR >> USART_SR_TXE_Pos) & 0x01) {
        USART2->DR = buff_tx[++tx_len];
    } else if((USART2->SR >> USART_SR_TC_Pos) & 0x01) {
        tx_len = 0;
        USART2->CR1 &= ~(0x01 << USART_CR1_TXEIE_Pos);
    }
}

void send_str_it(volatile char *buff, uint8_t len) {
    tx_len = 0; tx_len_max = len - 1;
    USART2->DR = buff[0];
    USART2->CR1 |= (0x01 << USART_CR1_TXEIE_Pos);
}

//funzioone del pid
float calcola_PID(float setpoint, float posizione) {
    float errore = posizione - setpoint; // > 0 significa pallina troppo bassa
    float P = Kp * errore;

    // Anti-windup con il calcolo del Tempo (DT)
    float out_parziale = PWM_BASE + P + (Ki * integrale);
    if ((out_parziale > PWM_MIN && out_parziale < PWM_MAX) ||
        (out_parziale >= PWM_MAX && errore < 0) ||
        (out_parziale <= PWM_MIN && errore > 0)) {
        integrale += errore * DT;
    }

    float I = Ki * integrale;


    float derivata_pos = (posizione - pos_pallina_prec) / DT;
    float D = Kd * derivata_pos;

    // Calcolo finale
    float out_finale = PWM_BASE + P + I + D;

    if (out_finale > PWM_MAX) out_finale = PWM_MAX;
    if (out_finale < PWM_MIN) out_finale = PWM_MIN;

    pos_pallina_prec = posizione;
    return out_finale;
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
	  MX_USART2_UART_Init();
	  /* USER CODE BEGIN 2 */
	  hc_sr04_init_mano();   // Inizializza i pin del sensore, l'EXTI e avvia anche il TIM6!
	  hc_sr04_init_tubo();
	  TIM3_PWM_setup(); // Inizializza il PWM e il pin del motore
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */



	  /* GPIO PA5, PA7, PA8 */

	  RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);

	  GPIOA->MODER &= ~((3 << 8) |
	                       (3 << 14) |
	                       (3 << 16));

	  GPIOA->MODER |= ((1 << 8) |
	                      (1 << 14) |
	                      (1 << 16));

	  /* TB6612 ENABLE */

	  GPIOA->ODR |= (1 << 4);     // STBY = HIGH

	  /* Direzione */

	  GPIOA->ODR |= (1 << 7);     // AIN1 = HIGH
	  GPIOA->ODR &= ~(1 << 8);     // AIN2 = LOW

	  /* Start timers */


	  TIM3->CR1 |= (1 << TIM_CR1_CEN_Pos);
	  NVIC_EnableIRQ(USART2_IRQn);


	  while (1)
	  	  {


		  //MISURA MANO gestisce anche mano assente
		            float raw_mano = meas_dist_mano();
		            if (raw_mano > 2.0f && raw_mano < 60.0f) {
		                no_hand_counter = 0; // La mano c'è, azzera l'allarme
		                if (dist_filtrata_mano == 0.0f) {
		                    dist_filtrata_mano = raw_mano;
		                } else {
		                    dist_filtrata_mano = (alpha * raw_mano) + ((1.0f - alpha) * dist_filtrata_mano);
		                }
		            } else {
		                no_hand_counter++; // Se legge fuori range, incrementa l'allarme
		            }
		            HAL_Delay(30);

		            // MISURA TUBO
		            float raw_tubo = meas_dist_tubo();
		            if (raw_tubo > 2.0f && raw_tubo < 60.0f) {
		                if (dist_filtrata_tubo == 0.0f) {
		                    dist_filtrata_tubo = raw_tubo;
		                } else {
		                    dist_filtrata_tubo = (alpha * raw_tubo) + ((1.0f - alpha) * dist_filtrata_tubo);
		                }
		            }
		            HAL_Delay(30);

		            // LOGICA DI CONTROLLO A STATI
		            uint32_t pwm_out = 0;

		            if (no_hand_counter > 5) {

		                pwm_out = 0;
		                dist_filtrata_mano = 0.0f; // Reset per il prossimo avvio
		                integrale = 0.0f;
		            }
		            else if (dist_filtrata_tubo > 45.0f) {

		                pwm_out = 999;
		            }
		            else {

		                float altezza_tubo = 50.0f;
		                float setpoint_reale = altezza_tubo - dist_filtrata_mano;

		                // Protezione: impedisce bersagli impossibili
		                if(setpoint_reale < 5.0f) setpoint_reale = 5.0f;

		                pwm_out = (uint32_t)calcola_PID(setpoint_reale, dist_filtrata_tubo);
		            }


		            TIM3->CCR1 = pwm_out;


		            int len = sprintf((char*)buff_tx, "%.2f,%.2f,%lu\r\n", dist_filtrata_mano, dist_filtrata_tubo, pwm_out);
		            send_str_it(buff_tx, len);

		            HAL_Delay(30);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
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
