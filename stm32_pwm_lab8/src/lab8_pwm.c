/* File - lab8_pwm.c:
 *
 * Description 	- implement the breathing with multiple profiles for the 'breathing' linear, sine and parablic using 3 PWM outputs
 *       	- pass commands for pwm set up, quit/stop and breathing profiles 
 * 			PA8	TIM1_CH1	D7
 * 			PA9	TIM1_CH2	D8
 * 			PA10	TIM1_CH3	D2
 * 			PA11	TIM1_CH4	CN10/14
 * Date		- 2022-11-09
 * Programmer 	- Sahan Amarasinghe
 *  	
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "common.h"
#include "main.h"
#include "math.h"

#define CLOCK_CYCLES_PER_SECOND 100000000
#define MAX_RELOAD              5000
#define PI 3.142857

TIM_HandleTypeDef htim1;	//handle timer 11
TIM_HandleTypeDef tim11;

ADC_ChannelConfTypeDef sConfig;
ADC_HandleTypeDef hadc1;	//Define a global ADC handle

HAL_StatusTypeDef rc;


/*global variable decleration */
uint32_t dutyCycle;
uint32_t channel;
uint32_t pwmFrequency;
uint32_t prescalerIntoPeriod;
uint16_t prescaler;
uint32_t period;
uint32_t pattern;
float angle = 0;
float x = -10;

/*funtion prototype decleration*/
static void MX_GPIO_Init_Pwm (void);
void timer1_init (void);
void timer11_init (void);
void set_pwm (uint32_t * channel, uint32_t * duty, uint32_t * pwmFrequency);




void
pwmInit (void *data)
{

  /* Place Initialization things here.  This function gets called once
   * at startup.
   */

  timer1_init ();
  timer11_init ();
  MX_GPIO_Init_Pwm ();
}


void
pwmTask (void *data)
{

  /* Place your task functionality in this function.  This function
   * will be called repeatedly, as if you placed it inside the main
   * while(1){} loop.
   */

}

ADD_TASK (pwmTask,		/* This is the name of the function for the task */
	  pwmInit,		/* This is the initialization function */
	  NULL,			/* This pointer is passed as 'data' to the functions */
	  0,			/* This is the number of milliseconds between calls */
	  "This is the help text for the task")
// FUNCTION      : Cmdpwm
//
// DESCRIPTION   :
//   Input the channe, duty cycle and pwm frequency to initialize the PWM
//
// PARAMETERS    :
//   mode - can be ignored
//
// RETURNS       :
//   CmdReturnOk is successfull
     ParserReturnVal_t Cmdpwm (int mode)
{
  uint32_t rc1, rc2, rc3;

  if (mode != CMD_INTERACTIVE)
    return CmdReturnOk;

  rc1 = fetch_uint32_arg (&channel);
  rc2 = fetch_uint32_arg (&dutyCycle);
  rc3 = fetch_uint32_arg (&pwmFrequency);
  if (rc1 || rc2 || rc3)
    {
      printf ("Please enter channel or/and value\n");
      return CmdReturnBadParameter1;
    }
  set_pwm (&channel, &dutyCycle, &pwmFrequency);

  return CmdReturnOk;
}

ADD_CMD ("pwm", Cmdpwm, "	        	step <steps> <delay>")
// FUNCTION      : Cmdpwm_pattern
//
// DESCRIPTION   :
//   input the pwm type - linear = 1, sin = 2 or parabolic = 3
//
// PARAMETERS    :
//   mode - can be ignored
//
// RETURNS       :
//   CmdReturnOk is successfull
     ParserReturnVal_t Cmdpwm_pattern (int mode)
{
  uint32_t rc1;

  if (mode != CMD_INTERACTIVE)
    return CmdReturnOk;

  rc1 = fetch_uint32_arg (&pattern);

  if (rc1)
    {
      printf ("Please input the pattern\n");
      return CmdReturnBadParameter1;
    }


  return CmdReturnOk;
}

ADD_CMD ("pwm_pattern", Cmdpwm_pattern,
	 "	        	step <steps> <delay>")
// FUNCTION      : Cmdpwm_pattern
//
// DESCRIPTION   :
//   quit from interrupt call back function and stop the pwm
//
// PARAMETERS    :
//   mode - can be ignored
//
// RETURNS       :
//   CmdReturnOk is successfull
     ParserReturnVal_t Cmdquit (int mode)
{

  pattern = 0;
  HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_3);


  return CmdReturnOk;
}

ADD_CMD ("q", Cmdquit, "	        	step <steps> <delay>")


     void set_pwm (uint32_t * channel, uint32_t * duty,
		   uint32_t * pwmFrequency)
{

  prescalerIntoPeriod = (uint32_t) (CLOCK_CYCLES_PER_SECOND / *pwmFrequency);
  prescaler = (uint32_t) (prescalerIntoPeriod / (MAX_RELOAD + 1));
  period = (uint32_t) (prescalerIntoPeriod) / prescaler;

  TIM1->PSC = prescaler - 1;
  TIM1->ARR = period - 1;

  switch (*channel)
    {
    case 1:
      TIM1->CCR1 = (*duty) * period / 100;
      HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
      break;

    case 2:
      TIM1->CCR2 = (*duty) * period / 100;
      HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);

      break;

    case 3:
      TIM1->CCR3 = (*duty) * period / 100;
      HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);
      break;

    default:
      HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_3);
      printf ("Stopped the PWM output\n");
      break;

      printf
	("prescaler:%d\t  period:%ld\t frequency:%ldHz\t duty cycle:%ld \n",
	 prescaler, period, *pwmFrequency, *duty);
    }

}



// FUNCTION      : timer1_init
//
// DESCRIPTION   :
//   Initialise the parameters for timer 11
//
// PARAMETERS    :
//   None
//
// RETURNS       :
//   Nothing

void
timer1_init (void)
{

  __HAL_RCC_TIM1_CLK_ENABLE ();

  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init (&htim1) != HAL_OK)
    {
      Error_Handler ();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim1, &sMasterConfig) !=
      HAL_OK)
    {
      Error_Handler ();
    }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler ();
    }
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler ();
    }
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler ();
    }


  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime (&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_IC_MspInit (&htim1);

}


// FUNCTION      : timer11_init
//
// DESCRIPTION   :
//   Initialise the parameters for timer 11
//
// PARAMETERS    :
//   None
//
// RETURNS       :
//   Nothing


void
timer11_init (void)
{

  __HAL_RCC_TIM11_CLK_ENABLE ();
  tim11.Instance = TIM11;
  tim11.Init.Prescaler = 10000 - 1;	// 100MHz/10000 = 10kHz
  tim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim11.Init.Period = 1000 - 1;	//by setting 10000 here, counter will overflow within 1s
  tim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim11.Init.RepetitionCounter = 0;
  tim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;	// this parameter will cause the timer to reset when timed out
  HAL_TIM_Base_Init (&tim11);

  HAL_NVIC_SetPriority (TIM1_TRG_COM_TIM11_IRQn, 10, 0U);	// note the timer 11 IRQ on this board is shared with Timer 1
  HAL_NVIC_EnableIRQ (TIM1_TRG_COM_TIM11_IRQn);

  HAL_TIM_Base_Start_IT (&tim11);

}


// FUNCTION      : TIM1_TRG_COM_TIM11_IRQHandler
//
// DESCRIPTION   :
//   This is the interrupt handler for timer 11 and 1.  It is usually created by CubeMX if we were using it
//
// PARAMETERS    :
//   None
//
// RETURNS       :
//   Nothing

void
TIM1_TRG_COM_TIM11_IRQHandler (void)
{
  HAL_TIM_IRQHandler (&tim11);

}


// FUNCTION      : HAL_TIM_PeriodElapsedCallback
//
// DESCRIPTION   :
//   This is the interrupt handler for timer 11 and 1.  It is usually created by CubeMX if we were using it
//
// PARAMETERS    :
//   htim - the handle to the timer configuration information
//
// RETURNS       :
//   Nothing
void
HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

  pwmFrequency = 1000;

  if (htim == &tim11)
    {

      if (pattern == 1)
	{
	  channel = 1;
	  if (dutyCycle >= 100)
	    {
	      dutyCycle = 0;
	    }
	  else
	    {

	      printf ("duty cycle:%ld \n", dutyCycle);
	      set_pwm (&channel, &dutyCycle, &pwmFrequency);
	      dutyCycle++;
	    }
	}

      if (pattern == 2)
	{
	  channel = 2;
	  if (angle >= PI)
	    {
	      angle = 0;
	    }
	  else
	    {


	      dutyCycle = 100 * sin (angle);
	      set_pwm (&channel, &dutyCycle, &pwmFrequency);
	      printf ("Angle:%.2f rad\t Duty cycle:%ld\n", angle, dutyCycle);
	      angle = angle + 0.01;
	    }
	}

      if (pattern == 3)
	{
	  channel = 3;
	  x = x + 0.1;
	  dutyCycle = x * x;
	  set_pwm (&channel, &dutyCycle, &pwmFrequency);
	  printf ("X:%f \t Duty cycle:%ld\n", x, dutyCycle);
	  if (x >= 10)
	    {
	      x = -10;
	    }
	}

    }

}



// FUNCTION      : MX_GPIO_Init_Pwm
//
// DESCRIPTION   :
//   Initialize the parameter for pwm pins
//
// PARAMETERS    :
//   Nothing
//
// RETURNS       :
//   Nothing
static void
MX_GPIO_Init_Pwm (void)
{

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* ADC pins configuration
     Enable the clock for the ADC GPIOs */
  __HAL_RCC_GPIOA_CLK_ENABLE ();

  GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 1;	//set 1 for pwm mode
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);


}
