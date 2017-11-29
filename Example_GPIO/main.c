#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

int main(void)
{
  int i;
  GPIO_InitTypeDef  GPIO_InitStructure;

  //+++++++++
  ErrorStatus HSEStartUpStatus;

  /* Системный RESET RCC (делать не обязательно, но полезно на этапе отладки) */
  	 RCC_DeInit();

  /* Включаем HSE (внешний кварц) */
     RCC_HSEConfig( RCC_HSE_ON);

  /* Ждем пока HSE будет готов */
     HSEStartUpStatus = RCC_WaitForHSEStartUp();

  /* Если с HSE все в порядке */
     if (HSEStartUpStatus == SUCCESS)
     {
         /* PLLCLK = 8MHz * 4 = 32 MHz */
         /* Указываем PLL от куда брать частоту (RCC_PLLSource_HSE_Div1) и на сколько ее умножать (RCC_PLLMul_9) */
         /* PLL может брать частоту с кварца как есть (RCC_PLLSource_HSE_Div1) или поделенную на 2 (RCC_PLLSource_HSE_Div2). Смотри схему */
         RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_2);

         /* Включаем PLL */
         RCC_PLLCmd( ENABLE);

         /* Ждем пока PLL будет готов */
         while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
         {
         }

         /* Переключаем системное тактирование на PLL */
         RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

         /* Ждем пока переключиться */
         while (RCC_GetSYSCLKSource() != 0x08)
         {
         }
     }
     else
     { /* Проблемы с HSE. Тут можно написать свой код, если надо что-то делать когда микроконтроллер не смог перейти на работу с внешним кварцом */

         /* Пока тут заглушка - вечный цикл*/
         while (1)
         {

         }
     }
  //+++++++++

  /* Initialize LED which connected to PC13 */
  // Enable PORTC Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_13); // Set C13 to High level ("1")
//  GPIO_ResetBits(GPIOA, GPIO_Pin_13); // Set C13 to Low level ("0")

  /* Initialize Button input PB0 */
  // Enable PORTB Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure the GPIO_BUTTON pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  while (1) {
	  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//  	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != 0) {
      	/* Toggle LED which connected to PC13*/
//  		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//      }
//      else {
//      	GPIO_SetBits(GPIOC, GPIO_Pin_13);
//      }

    }
  }
