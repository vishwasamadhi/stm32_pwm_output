/* example.c:
 *
 *   Template code for implementing a simple task, using the ADD_TASK()
 *   macro.  Also contains template code for a simple monitor command.
 *
 */
#include <stdio.h>
#include <stdint.h>

#include "common.h"
#include "main.h"
#include "API_TIMR.h"



void delay(uint32_t *delayVal)
{
	printf("test:%lu\n",delayVal);
}
/*
TIM_HandleTypeDef tim11;	
void timer_init(void);											// handle for timer11 used in this example
void delay_period(uint32_t delayVal);

//void delay(uint32_t delayVal);

uint32_t myTicks = 0;
//uint32_t *delayVal;

void timer_init(void)
{
	
	__HAL_RCC_TIM11_CLK_ENABLE();
	tim11.Instance = TIM11;
	tim11.Init.Prescaler = 10000-1;
	//HAL_RCC_GetPCLK2Freq() / val - 1;			// this will cause the timer to create ms (set to 1000000 - 1 for us)
	tim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim11.Init.Period = 10000-1;											// in this example the number here will be the number of ms for on/off
	tim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim11.Init.RepetitionCounter = 0;
	tim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;		// this parameter will cause the timer to reset when timed out
	HAL_TIM_Base_Init(&tim11);

	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 10, 0U);				// note the timer 11 IRQ on this board is shared with Timer 1
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	HAL_TIM_Base_Start_IT(&tim11);
 
}


void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&tim11);

}
void delay(uint32_t *delayVal)
{
	if(myTicks >= delayVal)
	{
	   	HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
		printf("myTicks :%lu\n",myTicks);
		
		myTicks = 0;
		
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim )
{
	//if( htim == &tim11 ){								// check if this was meant for us
	//	HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
	//}
	
	if( htim == &tim11 )
	{	
	myTicks++;
		delay(delayVal);
	

			
	}
}
*/

/*
ParserReturnVal_t Cmddelay(int mode)
{
	  uint32_t rc;
	  
	if(mode != CMD_INTERACTIVE) return CmdReturnOk;

	
	

  rc = fetch_uint32_arg(&delayVal);
  if(rc) {
    printf("Blinkding LD2 for the delay entered\n");
    return CmdReturnBadParameter1;
  }
  
	
	delay(delayVal);
   	
	printf(" delayVal :%lu\n",delayVal);
		
	
	return CmdReturnOk;
}
ADD_CMD("delay",Cmddelay,"                Enter delay")
*/

