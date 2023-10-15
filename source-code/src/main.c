/**
  **********************************************************************************
  * @file           : main.c
  * @brief          : This is the main entrypoint for the application.
  * @author         : Marius Englund, Eirik Skorve Haugland, Ingrid Hovland
  **********************************************************************************
  * 
  * The central entry point for the program code that has been implemented for the 
  * NUCLEO-L476RG development board, which is utilized in the inverter.
  *
  **********************************************************************************
  */


/* Includes ----------------------------------------------------------------------*/
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include <stm32l4xx_ll_usart.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_cortex.h>
#include <stm32l4xx_ll_rcc.h>
#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_tim.h>
#include <stm32l4xx_ll_utils.h>
#include <stm32l4xx_ll_pwr.h>
#include <stm32l4xx_ll_system.h>

#include "bsp_timer.h"


/* Define Valid Modulation Algorithms --------------------------------------------*/
// Defines valid modulation schemes for the inverter.
typedef enum {
  SPWM = 0,
  SVPWM
} scheme_t;


/* Define Rotor Direction --------------------------------------------------------*/
// Defines valid rotation directions for the rotor.
typedef enum {
  CW = 0,
  CCW
} direction_t;


/* Define Valid Operating States -------------------------------------------------*/
// Defines valid operating states for the inverter.
typedef enum {
  NOT_READY = 0,
  READY,
  RUNNING,
  FAULT
} state_t;


/* Define Compare Struct ---------------------------------------------------------*/
// Defines a struct of compare values for the timer.
typedef struct {
  uint32_t L1;
  uint32_t L2;
  uint32_t L3;
} compare_t;


/* Define Global Variables -------------------------------------------------------*/
// Defines globally accessible variables.
uint32_t sine_res_g           = 0;              // No. of Compare Values for Each Period (max 1024).
float sine_table[] = {0,0.0627905195293134,0.125333233564304,0.187381314585725,0.248689887164855,0.309016994374947,0.368124552684678,0.425779291565073,0.481753674101715,0.535826794978997,0.587785252292473,0.63742398974869,0.684547105928689,0.728968627421412,0.770513242775789,0.809016994374947,0.844327925502015,0.876306680043864,0.904827052466019,0.929776485888251,0.951056516295154,0.968583161128631,0.982287250728689,0.992114701314478,0.998026728428272,1,0.998026728428272,0.992114701314478,0.982287250728689,0.968583161128631,0.951056516295154,0.929776485888251,0.904827052466019,0.876306680043864,0.844327925502015,0.809016994374947,0.770513242775789,0.728968627421412,0.684547105928689,0.63742398974869,0.587785252292473,0.535826794978997,0.481753674101715,0.425779291565072,0.368124552684678,0.309016994374947,0.248689887164855,0.187381314585725,0.125333233564304,0.0627905195293133,0,-0.0627905195293133,-0.125333233564304,-0.187381314585725,-0.248689887164855,-0.309016994374947,-0.368124552684678,-0.425779291565072,-0.481753674101715,-0.535826794978997,-0.587785252292473,-0.63742398974869,-0.684547105928689,-0.728968627421412,-0.770513242775789,-0.809016994374947,-0.844327925502015,-0.876306680043863,-0.90482705246602,-0.929776485888251,-0.951056516295154,-0.968583161128631,-0.982287250728689,-0.992114701314478,-0.998026728428272,-1,-0.998026728428272,-0.992114701314478,-0.982287250728689,-0.968583161128631,-0.951056516295154,-0.929776485888251,-0.90482705246602,-0.876306680043863,-0.844327925502015,-0.809016994374947,-0.770513242775789,-0.728968627421412,-0.684547105928689,-0.63742398974869,-0.587785252292473,-0.535826794978996,-0.481753674101715,-0.425779291565072,-0.368124552684678,-0.309016994374947,-0.248689887164854,-0.187381314585725,-0.125333233564304,-0.0627905195293138,0};


float Ud_g                    = 0;              // DC-link Voltage [V]
float f1_g                    = 0;              // Modulation Frequency [Hz]
float fs_g                    = 0;              // Switching Frequency [Hz]
float ma_g                    = 0;              // Amplitude Modulation Index
float mf_g                    = 0;              // Frequency Modulation Index
// float time_samples_g[1024]    = {0};            // Time Samples [s]

scheme_t algorithm_g          = SVPWM;          // Modulation Scheme

direction_t direction_g       = CW;             // Rotor Rotation Direction

state_t operating_state_g     = NOT_READY;      // Inverter Operating State


/* Define UART I/O ---------------------------------------------------------------*/
// I/O declarations for the universal asynchronous receiver-transmitter.
#define UART2_PORT            GPIOA             // UART2 Port
#define UART2_TX_PIN          LL_GPIO_PIN_2     // UART2 Transmitter Pin
#define UART2_RX_PIN          LL_GPIO_PIN_3     // UART2 Receiver Pin


/* Define Gate Drivers I/O -------------------------------------------------------*/
// I/O declarations for the gate drivers.
/* Gate Driver 1 */
#define DRIVER1_RDY_PORT      GPIOB             // Gate Driver Leg A Ready Port
#define DRIVER1_RDY_PIN       LL_GPIO_PIN_9     // Gate Driver Leg A Ready Pin
#define DRIVER1_INHS_PORT     GPIOA             // Gate Driver Leg A High Side PWM Port
#define DRIVER1_INHS_PIN      LL_GPIO_PIN_8     // Gate Driver Leg A High Side PWM Pin
#define DRIVER1_INLS_PORT     GPIOA             // Gate Driver Leg A Low Side PWM Port
#define DRIVER1_INLS_PIN      LL_GPIO_PIN_7     // Gate Driver Leg A Low Side PWM Pin
#define DRIVER1_RST_PORT      GPIOC             // Gate Driver Leg A Reset Port
#define DRIVER1_RST_PIN       LL_GPIO_PIN_4     // Gate Driver Leg A Reset Pin
#define DRIVER1_FLT_PORT      GPIOA             // Gate Driver Leg A Fault Port
#define DRIVER1_FLT_PIN       LL_GPIO_PIN_12    // Gate Driver Leg A Fault Pin

/* Gate Driver 2 */
#define DRIVER2_RDY_PORT      GPIOA             // Gate Driver Leg B Ready Port
#define DRIVER2_RDY_PIN       LL_GPIO_PIN_11    // Gate Driver Leg B Ready Pin
#define DRIVER2_INHS_PORT     GPIOA             // Gate Driver Leg B High Side PWM Port
#define DRIVER2_INHS_PIN      LL_GPIO_PIN_9     // Gate Driver Leg B High Side PWM Pin
#define DRIVER2_INLS_PORT     GPIOB             // Gate Driver Leg B Low Side PWM Port
#define DRIVER2_INLS_PIN      LL_GPIO_PIN_0     // Gate Driver Leg B Low Side PWM Pin
#define DRIVER2_RST_PORT      GPIOA             // Gate Driver Leg B Reset Port
#define DRIVER2_RST_PIN       LL_GPIO_PIN_6     // Gate Driver Leg B Reset Pin
#define DRIVER2_FLT_PORT      GPIOC             // Gate Driver Leg B Fault Port
#define DRIVER2_FLT_PIN       LL_GPIO_PIN_9     // Gate Driver Leg B Fault Pin

/* Gate Driver 3 */
#define DRIVER3_RDY_PORT      GPIOC             // Gate Driver Leg C Ready Port
#define DRIVER3_RDY_PIN       LL_GPIO_PIN_8     // Gate Driver Leg C Ready Pin
#define DRIVER3_INHS_PORT     GPIOA             // Gate Driver Leg C High Side PWM Port
#define DRIVER3_INHS_PIN      LL_GPIO_PIN_10    // Gate Driver Leg C High Side PWM Pin
#define DRIVER3_INLS_PORT     GPIOB             // Gate Driver Leg C Low Side PWM Port
#define DRIVER3_INLS_PIN      LL_GPIO_PIN_1     // Gate Driver Leg C Low Side PWM Pin
#define DRIVER3_RST_PORT      GPIOA             // Gate Driver Leg C Reset Port
#define DRIVER3_RST_PIN       LL_GPIO_PIN_4     // Gate Driver Leg C Reset Pin
#define DRIVER3_FLT_PORT      GPIOC             // Gate Driver Leg C Fault Port
#define DRIVER3_FLT_PIN       LL_GPIO_PIN_7     // Gate Driver Leg C Fault Pin


/* Define HMI I/O ----------------------------------------------------------------*/
// I/O declarations for the human–machine interface.
/* Start Button */
#define HMI_SW1_PORT          GPIOA             // HMI Start Button Port
#define HMI_SW1_PIN           LL_GPIO_PIN_15    // HMI Start Button Pin

/* Stop Button */
#define HMI_SW2_PORT          GPIOC             // HMI Stop Button Port
#define HMI_SW2_PIN           LL_GPIO_PIN_10    // HMI Stop Button Pin

/* Green LED */
#define HMI_LED1_PORT         GPIOB             // HMI Green LED Port
#define HMI_LED1_PIN          LL_GPIO_PIN_4     // HMI Green LED Pin

/* Yellow LED */
#define HMI_LED2_PORT         GPIOB             // HMI Yellow LED Port
#define HMI_LED2_PIN          LL_GPIO_PIN_5     // HMI Yellow LED Pin

/* Red LED */
#define HMI_LED3_PORT         GPIOB             // HMI Red LED Port
#define HMI_LED3_PIN          LL_GPIO_PIN_8     // HMI Red LED Pin


/* Configure System Clock --------------------------------------------------------*/
// Configures the system core clock frequency equal to 80 MHz.
void configure_system_clock(void) {

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4){
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is Ready */
  while(LL_RCC_HSI_IsReady() != 1){
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is Ready */
  while(LL_RCC_PLL_IsReady() != 1){
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System Clock is Ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(80000000); // 80 MHz

  LL_SetSystemCoreClock(80000000); // 80 MHz
}


/* Initialize UART ---------------------------------------------------------------*/
// Initializes the universal asynchronous receiver-transmitter.
void initialize_UART(void) {

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(UART2_PORT, UART2_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(UART2_PORT, UART2_RX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(UART2_PORT, UART2_TX_PIN, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_0_7(UART2_PORT, UART2_RX_PIN, LL_GPIO_AF_7);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_8);

    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_8, 9600);

    LL_USART_EnableDirectionRx(USART2);
    LL_USART_EnableDirectionTx(USART2);

    LL_USART_Enable(USART2);
}


/* Set SysTick Frequency ---------------------------------------------------------*/
// Sets the SysTick_Handler() frequency [Hz].
void set_SysTick_frequency(uint32_t freq) {
  
  if (SysTick_Config(SystemCoreClock/freq)) {
    while (1); // Stop program if configuration failed.
  }
}


/* Initialize TIM1 ---------------------------------------------------------------*/
// Initializes TIM1 for PWM generation on CH1, CH1N, CH2, CH2N, CH3 and CH3N.
void initialize_TIM1(void) {

  /* Enable Peripheral Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* General Settings */
  LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_SetRepetitionCounter(TIM1, 0);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

  /* CH1 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH1);

  /* CH2 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH2);

  /* CH3 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH3);

  /* Other Settings */
  LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_NC);
  LL_TIM_DisableExternalClock(TIM1);
  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  LL_TIM_OC_SetDeadTime(TIM1, 32);

  /* Enable PWM Outputs */
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N); 
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3); 
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
}


/* Initialize TIM2 ---------------------------------------------------------------*/
// Initializes TIM2 for interrupt requests.
void initialize_TIM2(void) {

  /* Enable Peripheral Clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* General Settings */
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);

  /* IRQ Settings */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);
}


/* Enable GPIO Ports Clock -------------------------------------------------------*/
// Enables the GPIO ports clock.
void enable_gpio_ports_clock(void) {

  /* Enable GPIO Ports Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
}


/* Initialize Gate Drivers -------------------------------------------------------*/
// Initializes the gate driver pins for PWM using TIM1.
void initialize_gate_drivers(void) {

  /* Configure Gate Driver 1 RDY */
  LL_GPIO_SetPinMode(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 1 INHS */
  LL_GPIO_SetPinMode(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 1 INLS */
  LL_GPIO_SetPinMode(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 1 RST */
  LL_GPIO_SetPinMode(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 1 FLT */
  LL_GPIO_SetPinMode(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 2 RDY */
  LL_GPIO_SetPinMode(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 2 INHS */
  LL_GPIO_SetPinMode(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 2 INLS */
  LL_GPIO_SetPinMode(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 2 RST */
  LL_GPIO_SetPinMode(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 2 FLT */
  LL_GPIO_SetPinMode(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 RDY */
  LL_GPIO_SetPinMode(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 INHS */
  LL_GPIO_SetPinMode(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 3 INLS */
  LL_GPIO_SetPinMode(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 3 RST */
  LL_GPIO_SetPinMode(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 FLT */
  LL_GPIO_SetPinMode(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_PULL_NO);
}


/* Initialize HMI ----------------------------------------------------------------*/
// Initializes the human–machine interface pins.
void initialize_hmi(void) {

  /* Configure Start Button */
  LL_GPIO_SetPinMode(HMI_SW1_PORT, HMI_SW1_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(HMI_SW1_PORT, HMI_SW1_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(HMI_SW1_PORT, HMI_SW1_PIN, LL_GPIO_PULL_NO);

  /* Configure Stop Button */
  LL_GPIO_SetPinMode(HMI_SW2_PORT, HMI_SW2_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(HMI_SW2_PORT, HMI_SW2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(HMI_SW2_PORT, HMI_SW2_PIN, LL_GPIO_PULL_NO);

  /* Configure Green LED */
  LL_GPIO_SetPinMode(HMI_LED1_PORT, HMI_LED1_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(HMI_LED1_PORT, HMI_LED1_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(HMI_LED1_PORT, HMI_LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(HMI_LED1_PORT, HMI_LED1_PIN, LL_GPIO_PULL_NO);

  /* Configure Yellow LED */
  LL_GPIO_SetPinMode(HMI_LED2_PORT, HMI_LED2_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(HMI_LED2_PORT, HMI_LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(HMI_LED2_PORT, HMI_LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(HMI_LED2_PORT, HMI_LED2_PIN, LL_GPIO_PULL_NO);

  /* Configure Red LED */
  LL_GPIO_SetPinMode(HMI_LED3_PORT, HMI_LED3_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(HMI_LED3_PORT, HMI_LED3_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(HMI_LED3_PORT, HMI_LED3_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(HMI_LED3_PORT, HMI_LED3_PIN, LL_GPIO_PULL_NO);
}


/* Set Ud ------------------------------------------------------------------------*/
// Sets the DC-link voltage parameter [V].
void set_Ud(float Ud) {

  Ud_g = Ud;
}


/* Set Switching Frequency -------------------------------------------------------*/
// Sets the switching frequency parameter [Hz].
void set_switching_frequency(float fs) {

  fs_g = fs;
}


/* Set Sine Res ------------------------------------------------------------------*/
// Sets the no. of compare values for each period.
void set_sine_res(uint32_t sine_res) {

  sine_res_g = sine_res;
}


/* Set Modulation Frequency ------------------------------------------------------*/
// Sets the modulation frequency parameter [Hz].
void set_modulation_frequency(float f1) {

  f1_g = f1;
}


/* Set Rotor Direction -----------------------------------------------------------*/
// Sets the rotor rotation direction parameter.
void set_rotor_direction(direction_t direction) {

  direction_g = direction;
}


/* Set Modulation Algorithm ------------------------------------------------------*/
// Sets the modulation algorithm.
void set_modulation_algorithm(scheme_t algorithm) {

  algorithm_g = algorithm;
}


/* Set ma ------------------------------------------------------------------------*/
// Sets the amplitude modulation index.
void set_ma(float ma) {

  ma_g = ma;
}


/* Set mf ------------------------------------------------------------------------*/
// Sets the frequency modulation index.
void set_mf(float mf) {

  mf_g = mf;
}


/* Set Operating State -----------------------------------------------------------*/
// Sets the operating state of the inverter.
void set_operating_state(state_t state) {
  
  operating_state_g = state;
}


/* Get Operating State -----------------------------------------------------------*/
// Returns the operating state of the inverter.
state_t get_operating_state(void) {
  
  return operating_state_g;
}


/* Get Modulation Frequency ------------------------------------------------------*/
// Returns the modulation frequency parameter [Hz].
float get_modulation_frequency(void) {

  return f1_g;
}


/* Get Ud ------------------------------------------------------------------------*/
// Returns the DC-link voltage parameter [V].
float get_Ud(void) {

  return Ud_g;
}


/* Get ma ------------------------------------------------------------------------*/
// Returns the amplitude modulation index.
float get_ma(void) {

  return ma_g;
}


/* Get Switching Frequency -------------------------------------------------------*/
// Returns the switching frequency parameter [Hz].
float get_switching_frequency(void) {

  return fs_g;
}


/* Get mf ------------------------------------------------------------------------*/
// Returns the frequency modulation index.
float get_mf(void) {

  return mf_g;
}


/* Get Rotor Direction -----------------------------------------------------------*/
// Returns the rotor rotation direction parameter.
direction_t get_rotor_direction(void) {

  return direction_g;
}


/* Get Modulation Algorithm ------------------------------------------------------*/
// Returns the modulation algorithm.
scheme_t get_modulation_algorithm(void) {

  return algorithm_g;
}


/* Get Sine Res ------------------------------------------------------------------*/
// Returns the no. of compare values for each period.
uint32_t get_sine_res(void) {

  return sine_res_g;
}


/* Set Green LED State -----------------------------------------------------------*/
// Sets the HMIs green LED state.
void set_green_led_state(bool state) {

  /* Turn on LED */
  if (state)
    LL_GPIO_SetOutputPin(HMI_LED1_PORT, HMI_LED1_PIN);

  /* Turn off LED */
  else
    LL_GPIO_ResetOutputPin(HMI_LED1_PORT, HMI_LED1_PIN);
}


/* Set Yellow LED State ----------------------------------------------------------*/
// Sets the HMIs yellow LED state.
void set_yellow_led_state(bool state) {

  /* Turn on LED */
  if (state)
    LL_GPIO_SetOutputPin(HMI_LED2_PORT, HMI_LED2_PIN);

  /* Turn off LED */
  else
    LL_GPIO_ResetOutputPin(HMI_LED2_PORT, HMI_LED2_PIN);
}


/* Set Red LED State -------------------------------------------------------------*/
// Sets the HMIs green LED state.
void set_red_led_state(bool state) {

  /* Turn on LED */
  if (state)
    LL_GPIO_SetOutputPin(HMI_LED3_PORT, HMI_LED3_PIN);

  /* Turn off LED */
  else
    LL_GPIO_ResetOutputPin(HMI_LED3_PORT, HMI_LED3_PIN);
}


/* Calculate Timer Compare SPWM --------------------------------------------------*/
// Computes and returns timer compare values using the SPWM algorithm.
compare_t calculate_timer_compare_spwm(float ma, uint32_t k_reload, float f1, float time) {

  compare_t compare;

  float bias = ma*k_reload;

  if (ma <= 1.0) { // Linear modulation
    compare.L1 = (uint32_t)((ma*k_reload*sin(2*M_PI*f1*time)+bias)/2.0);
    compare.L2 = (uint32_t)((ma*k_reload*sin(2*M_PI*f1*time-120*(M_PI/180.0))+bias)/2.0);
    compare.L3 = (uint32_t)((ma*k_reload*sin(2*M_PI*f1*time+120*(M_PI/180.0))+bias)/2.0);
  }

  return compare;
}


/* Min ---------------------------------------------------------------------------*/
// Returns the lowest floating-point number among the three inputs.
float min(float x, float y, float z) {

  return x < y ? (x < z ? x : z) : (y < z ? y : z);
}


/* Max ---------------------------------------------------------------------------*/
// Returns the highest floating-point number among the three inputs.
float max(float x, float y, float z) {

  return x > y ? (x > z ? x : z) : (y > z ? y : z);
}


/* Calculate Timer Compare SV-PWM ------------------------------------------------*/
// Computes and returns timer compare values using the SV-PWM algorithm.
compare_t calculate_timer_compare_svpwm(float ma, uint32_t k_reload, float f1, float time, float Ud) {

  compare_t compare = {0};

  float bias = ma*k_reload;

  if (ma <= 1.0) { // Linear modulation
    float UAo1 = ma*(Ud/sqrt(3))*sin(2*M_PI*f1*time);
    float UBo1 = ma*(Ud/sqrt(3))*sin(2*M_PI*f1*time-120*(M_PI/180.0));
    float UCo1 = ma*(Ud/sqrt(3))*sin(2*M_PI*f1*time+120*(M_PI/180.0));
    
    float Uk = (1/2.0)*(max(UAo1, UBo1, UCo1)+min(UAo1, UBo1, UCo1));

    compare.L1 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sin(2*M_PI*f1*time)-(Uk/Ud))+bias/2.0);
    compare.L2 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sin(2*M_PI*f1*time-120*(M_PI/180.0))-(Uk/Ud))+bias/2.0);
    compare.L3 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sin(2*M_PI*f1*time+120*(M_PI/180.0))-(Uk/Ud))+bias/2.0);
  }

  return compare;
}


/* Enable Gate Drivers -----------------------------------------------------------*/
// Enables Gate Drivers RST pin.
void enable_gate_drivers() {

  /* Enable Gate Drivers */
  LL_GPIO_SetOutputPin(DRIVER1_RST_PORT, DRIVER1_RST_PIN);
  LL_GPIO_SetOutputPin(DRIVER2_RST_PORT, DRIVER2_RST_PIN);
  LL_GPIO_SetOutputPin(DRIVER3_RST_PORT, DRIVER3_RST_PIN);
}


/* Disable Gate Drivers ----------------------------------------------------------*/
// Disable Gate Drivers RST pin.
void disable_gate_drivers() {

  /* Disable Gate Drivers */
  LL_GPIO_ResetOutputPin(DRIVER1_RST_PORT, DRIVER1_RST_PIN);
  LL_GPIO_ResetOutputPin(DRIVER2_RST_PORT, DRIVER2_RST_PIN);
  LL_GPIO_ResetOutputPin(DRIVER3_RST_PORT, DRIVER3_RST_PIN);
}


/* Start Modulation --------------------------------------------------------------*/
// Starts modulation of AC voltage.
void start_modulation(void) {

  /* Enable Gate Drivers */
  enable_gate_drivers();

  /* Enable TIM1 */
  uint32_t fs = (uint32_t)get_switching_frequency();
  bsp_set_TIM1_frequency(SystemCoreClock, fs);
  LL_TIM_EnableCounter(TIM1);

  /* Enable TIM2 */
  uint32_t f_tick = 10000;
  bsp_set_TIM2_frequency(SystemCoreClock, f_tick);
  LL_TIM_EnableCounter(TIM2);
}


/* Stop Modulation ---------------------------------------------------------------*/
// Stops modulation of AC voltage.
void stop_modulation(void) {

  /* Disable Gate Drivers */
  disable_gate_drivers();

  /* Disable TIM1 */
  LL_TIM_DisableCounter(TIM1);

  /* Disable TIM2 */
  LL_TIM_DisableCounter(TIM2);
}


/* SysTick Handler ---------------------------------------------------------------*/
// Ticks at the frequency set by set_SysTick_frequency().
void SysTick_Handler(void) {

}


/* TIM2 IRQ Handler --------------------------------------------------------------*/
// Ticks at the frequency set by bsp_set_TIM2_frequency().
void TIM2_IRQHandler(void) {

  static uint_fast32_t ticks = 0;

  /* Sinusoidal Pulse Width Modulation */
  if (ticks >= 2) {

    static uint32_t i1 = 0;
    static uint32_t i2 = 67;
    static uint32_t i3 = 34;

    // scheme_t algorithm = get_modulation_algorithm();
    // float ma = get_ma();
    // uint32_t k_reload = LL_TIM_GetAutoReload(TIM1);
    // // float f1 = get_modulation_frequency();
    // float Ud = get_Ud();
    // uint32_t sine_res = get_sine_res();

    scheme_t algorithm = SPWM;
    float ma = 0.5;
    uint32_t k_reload = LL_TIM_GetAutoReload(TIM1);
    // float f1 = get_modulation_frequency();
    float Ud = 500;
    uint32_t sine_res = 101;

    // float period = 1/f1;
    // float step = period/(sine_res-1);
    // float t = ticks*step;

    compare_t compare;

    float bias = k_reload;
    

    if (algorithm == SPWM) {

      compare.L1 = (uint32_t)((ma*k_reload*sine_table[i1]+bias)/2.0);
      compare.L2 = (uint32_t)((ma*k_reload*sine_table[i2]+bias)/2.0);
      compare.L3 = (uint32_t)((ma*k_reload*sine_table[i3]+bias)/2.0);

    }

    else if (algorithm == SVPWM) {

    float UAo1 = ma*(Ud/sqrt(3))*sine_table[i1];
    float UBo1 = ma*(Ud/sqrt(3))*sine_table[i2];
    float UCo1 = ma*(Ud/sqrt(3))*sine_table[i3];
    
    float Uk = (1/2.0)*(max(UAo1, UBo1, UCo1)+min(UAo1, UBo1, UCo1));

    // float bias = k_reload;

    compare.L1 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i1]-(Uk/Ud))+bias/2.0);
    compare.L2 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i2]-(Uk/Ud))+bias/2.0);
    compare.L3 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i3]-(Uk/Ud))+bias/2.0);

    }

    /* Define Rotor Direction */
    if (get_rotor_direction() == CCW) { // Counterclockwise direction
    
      uint32_t temp = compare.L2;
      compare.L2 = compare.L3;
      compare.L3 = temp;
    }
    
    bsp_set_TIM1_compare(compare.L1, compare.L2, compare.L3);

    // 50 % DUTY CYCLE:
    // compare.L1 = k_reload/2;
    // bsp_set_TIM1_compare(compare.L1, compare.L1, compare.L1);

    // LL_GPIO_TogglePin(HMI_LED1_PORT, HMI_LED1_PIN);

    i1++; i2++; i3++;
    if (i1 >= sine_res) i1 = 1;
    if (i2 >= sine_res) i2 = 1;
    if (i3 >= sine_res) i3 = 1;

    ticks = 0;
  }

  
  ticks++;
  

  /* Clear the Auto Reload match interrupt flag */
  if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) LL_TIM_ClearFlag_UPDATE(TIM2);
}


// /* TIM2 IRQ Handler --------------------------------------------------------------*/
// // Ticks at the frequency set by bsp_set_TIM2_frequency().
// void TIM2_IRQHandler(void) {

//   static uint_fast32_t ticks = 0;
//   float f1 = get_modulation_frequency();
//   float period = 1/f1;
//   uint32_t sine_res = get_sine_res();
//   float step = period/(sine_res-1);
//   // static float T = 1/1000000;

//   if (ticks >= step*1000) {
//     static uint32_t i = 0;
//     update_output_voltage(time_samples_g[i]);
//     i++;

//     if (i >= (sine_res-1))
//       i = 1; // Skips the first value in array.
//   }

//   ticks++;

//   /* Clear the Auto Reload match interrupt flag */
//   if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
//     LL_TIM_ClearFlag_UPDATE(TIM2);
// }



char *intToStrDec(uint32_t x, char *s)
{
    *--s = 0; // Decrement pointer and NULL terminate string
    if (!x) *--s = '0'; // Add a zero in ASCII if x is zero
    for (; x; x/=10) *--s = '0'+x%10; // Loop as long as x is not equal to zero, divide by 10 and add the remainder as the current digit
    // (remember that x/=10 is performed after the body x%10)
    return s; // Return a pointer to the beginning of the string (this does not have to be the beginning of the buffer)
}

void print_uint32(uint32_t val) {

    char buf[3*sizeof(uint32_t)+1];

    char *str = intToStrDec(val, buf + sizeof(buf));

    while(*str){
        while(!LL_USART_IsActiveFlag_TC(USART2));
        LL_USART_TransmitData8(USART2, *str++);
    }
}

void print_str(char *str){
    while(*str){
        while(!LL_USART_IsActiveFlag_TC(USART2));
        LL_USART_TransmitData8(USART2, *str++);
    }
}










/* Main --------------------------------------------------------------------------*/
int main(void) {

  /* Configure System Clock */
  configure_system_clock();

  /* Initialize UART */
  initialize_UART();

  /* Initialize Timer */
  initialize_TIM1();
  initialize_TIM2();

  /* Initialize GPIO */
  enable_gpio_ports_clock();
  initialize_gate_drivers();
  initialize_hmi();

  /* Define Inital Start Parameters */
  set_Ud(565.0);
  set_ma(1.0);
  set_modulation_frequency(50.0);
  set_switching_frequency(5000.0);
  set_rotor_direction(CW);
  set_modulation_algorithm(SPWM);
  set_sine_res(101);
  set_operating_state(READY);

  // start_modulation();


  // char buf[128];
  // snprintf(buf, sizeof(buf), "Avgitt spenning: %d V\nFrekvens: %d Hz", 400, 50);
  // print_str(buf);



  /* Endless Loop */
  while (true) {

    /* Check Operating State */
    state_t operating_state = get_operating_state();

    switch (operating_state) {
      case NOT_READY:
        set_green_led_state(false);
        set_yellow_led_state(false);
        set_red_led_state(false);
        break;

      case READY:
        set_green_led_state(false);
        set_yellow_led_state(true);
        set_red_led_state(false);
        break;

      case RUNNING:
        set_green_led_state(true);
        set_yellow_led_state(false);
        set_red_led_state(false);
        break;

      case FAULT:
        set_green_led_state(false);
        set_yellow_led_state(false);
        set_red_led_state(true);
        break;
      
      default:
        break;
    }

    /* Check Start Button */
    static bool SW1_last_state = false;
    bool SW1_state = LL_GPIO_IsInputPinSet(HMI_SW1_PORT, HMI_SW1_PIN);
    if(SW1_state != SW1_last_state) {
      if(SW1_state) { // The button went from off to on
        start_modulation();
        set_operating_state(RUNNING);
      }
      SW1_last_state = SW1_state;
    }

    /* Check Stop Button */
    static bool SW2_last_state = false;
    bool SW2_state = LL_GPIO_IsInputPinSet(HMI_SW2_PORT, HMI_SW2_PIN);
    if(SW2_state != SW2_last_state) {
      if(SW2_state) { // The button went from off to on
        stop_modulation();
        set_operating_state(READY);
      }
      SW2_last_state = SW2_state;
    }
  }
}