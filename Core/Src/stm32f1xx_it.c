/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint64_t tim3tick_overflows = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */

//  /* USER CODE END SysTick_IRQn 0 */
//  HAL_IncTick();
//  /* USER CODE BEGIN SysTick_IRQn 1 */

//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/* Set the clock frequency appropriately for your board. */
#define CPU_MHZ   (8*1000000)


/* SysTick registers */
/* SysTick control & status */
#define INITCPU_SYST_CSR	 ((volatile unsigned int *)0xE000E010)
/* SysTick Reload value */
#define INITCPU_SYST_RVR	 ((volatile unsigned int *)0xE000E014)
/* SysTick Current value */
#define INITCPU_SYST_CVR	 ((volatile unsigned int *)0xE000E018)


/* SysTick CSR register bits */
#define INITCPU_SYST_CSR_COUNTFLAG (1 << 16)

#define INITCPU_SYST_CSR_CLKSOURCE (1 << 2)
#define INITCPU_SYST_CSR_TICKINT (1 << 1)
#define INITCPU_SYST_CSR_ENABLE (1 << 0)

static volatile unsigned int systick_overflows = 0;

/* This function is called by the SysTick overflow interrupt handler. The
* address of this function must appear in the SysTick entry of the vector
* table. */
void SysTick_Handler(void)
{
	systick_overflows++;

}

static void reset_cycle_counter(void)
{
	/* Set the reload value and clear the current value. */
	*INITCPU_SYST_RVR = 0x00ffffff;
	*INITCPU_SYST_CVR = 0;
	/* Reset the overflow counter */
	systick_overflows = 0;
}

static void start_cycle_counter(void)
{
	/* Enable the SysTick timer and enable the SysTick overflow interrupt */
	*INITCPU_SYST_CSR |=
		(INITCPU_SYST_CSR_CLKSOURCE |
		INITCPU_SYST_CSR_ENABLE |
		INITCPU_SYST_CSR_TICKINT);
}

static uint64_t get_cycle_counter(void)
{
	unsigned int overflows = systick_overflows;
	/* A systick overflow might happen here */
	unsigned int systick_count = *INITCPU_SYST_CVR;
	/* check if it did and reload the low bit if it did */
	unsigned int new_overflows = systick_overflows;
	if (overflows != new_overflows)
	{
		/* This suffices as long as there is no chance that a second
		overflow can happen since new_overflows was read */
		systick_count = *INITCPU_SYST_CVR;
		overflows = new_overflows;
	}
	/* Recall that the SysTick counter counts down. */
	return (((uint64_t)overflows * 0x00FFFFFF) + (0x00FFFFFF - systick_count));
}

static uint64_t cycle_counter_init_value;

extern uint64_t clock_new(void)
{
//	return (clock_t) (((get_cycle_counter() -
//													cycle_counter_init_value) *
//													CLOCKS_PER_SEC) / CPU_MHZ);
	

	
	uint64_t now=get_cycle_counter()-cycle_counter_init_value;
	return now;
	
}

/* Microlib calls the _clock_init() function during library initialization to
* set up the cycle_counter_init_value variable. The value of this variable is
* then used as the baseline for subsequent calls to the clock() function. */
extern void _clock_init(void)
{
	reset_cycle_counter();
	start_cycle_counter();
	cycle_counter_init_value = get_cycle_counter();
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
