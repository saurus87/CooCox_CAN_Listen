#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

int main(void)
{
  int i;
  GPIO_InitTypeDef  GPIO_InitStructure;

  //+++++++++
  ErrorStatus HSEStartUpStatus;

  /* ��������� RESET RCC (������ �� �����������, �� ������� �� ����� �������) */
  	 RCC_DeInit();

  /* �������� HSE (������� �����) */
     RCC_HSEConfig( RCC_HSE_ON);

  /* ���� ���� HSE ����� ����� */
     HSEStartUpStatus = RCC_WaitForHSEStartUp();

  /* ���� � HSE ��� � ������� */
     if (HSEStartUpStatus == SUCCESS)
     {
         /* PLLCLK = 8MHz * 4 = 32 MHz */
         /* ��������� PLL �� ���� ����� ������� (RCC_PLLSource_HSE_Div1) � �� ������� �� �������� (RCC_PLLMul_9) */
         /* PLL ����� ����� ������� � ������ ��� ���� (RCC_PLLSource_HSE_Div1) ��� ���������� �� 2 (RCC_PLLSource_HSE_Div2). ������ ����� */
         RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_2);

         /* �������� PLL */
         RCC_PLLCmd( ENABLE);

         /* ���� ���� PLL ����� ����� */
         while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
         {
         }

         /* ����������� ��������� ������������ �� PLL */
         RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

         /* ���� ���� ������������� */
         while (RCC_GetSYSCLKSource() != 0x08)
         {
         }
     }
     else
     { /* �������� � HSE. ��� ����� �������� ���� ���, ���� ���� ���-�� ������ ����� ��������������� �� ���� ������� �� ������ � ������� ������� */

         /* ���� ��� �������� - ������ ����*/
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
