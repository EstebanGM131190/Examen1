/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Pushbutton.c
 * @brief   Application entry point.
 */#include <stdio.h>
#include "DataTypeDefinitions.h"
#include "MK64F12.h"

void delay(uint16 delay);
/*
 * @brief   Application entry point.
 */
#define VERDE 0
#define AZUL  1
#define MORADO 2
#define ROJO  3
#define AMARILLO  4

#define SW2_P  0
#define SW3_P  1
uint8 currentState = 0;
 void LED_COLOR(uint8 x){
		if(x == 0)
		{
			GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			GPIOB->PDOR |= 0x00400000;/**Read led off*/
			GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
		}
		else if(x == 1)
		{
			GPIOB->PDOR &= ~(0x00200000);/**Blue led off*/
			GPIOB->PDOR |= 0x00400000;/**Read led off*/
			GPIOE->PDOR |= 0x4000000;/**Green led off*/

		}
		else if(x == 2)
		{
			GPIOB->PDOR &= ~(0x00200000);/**Blue led off*/
			GPIOB->PDOR &= ~(0x00400000);/**Read led off*/
			GPIOE->PDOR |= 0x4000000;/**Green led off*/

		}
		else if(x == 3)
		{
			GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			GPIOB->PDOR &= ~(0x00400000);/**Read led off*/
			GPIOE->PDOR |= 0x4000000;/**Green led off*/

		}
		else if(x == 4)
		{
			GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			GPIOB->PDOR &= ~(0x00400000);/**Read led off*/
			GPIOE->PDOR &= ~(0x4000000);/**Green led off*/

		}


	 return 0;
 }

 typedef struct
 {
	uint8 Estado;
 	void(*fptrPort)(uint8); //Colores
	uint8 SW_PRESSED[2];				//SW
 	uint8 next[5];
 }StateType;

 const StateType FSM_Moore[6]=
 		{
 				{VERDE,LED_COLOR,{SW2_P,SW3_P},{AZUL,MORADO,ROJO,AMARILLO,VERDE}}, /**Even*/
 				{AZUL,LED_COLOR,{SW2_P,SW3_P},{MORADO,ROJO,AMARILLO,VERDE, AZUL}},  /**Odd*/
 				{MORADO,LED_COLOR,{SW2_P,SW3_P},{ROJO,AMARILLO,VERDE, AZUL,MORADO}}, /**Even*/
 				{ROJO,LED_COLOR,{SW2_P,SW3_P},{AMARILLO,VERDE, AZUL,MORADO,ROJO}}, /**Odd*/
 				{AMARILLO,LED_COLOR,{SW2_P,SW3_P},{VERDE, AZUL,MORADO,ROJO,AMARILLO}}  /**Odd*/

 		};
int main(void) {
	/**Variable to capture the input value*/
	uint32 inputValue = 0;

	/**Activating the GPIOB, GPIOC and GPIOE clock gating*/
	SIM->SCGC5 = 0x2C00;
	/**Pin control configuration of GPIOB pin22 and pin21 as GPIO*/
	PORTB->PCR[21] = 0x00000100;
	PORTB->PCR[22] = 0x00000100;
	/**Pin control configuration of GPIOC pin6 as GPIO with is pull-up resistor enabled*/
	PORTC->PCR[6] = 0x00000103;
	/**Pin control configuration of GPIOE pin26 as GPIO*/
	PORTE->PCR[26] = 0x00000100;
	/**Assigns a safe value to the output pin21 of the GPIOB*/
	GPIOB->PDOR = 0x00200000;
	/**Assigns a safe value to the output pin22 of the GPIOB*/
	GPIOB->PDOR |= 0x00400000;
	/**Assigns a safe value to the output pin26 of the GPIOE*/
	GPIOE->PDOR |= 0x04000000;

	GPIOC->PDDR &=~(0x40);
	/**Configures GPIOB pin21 as output*/
	GPIOB->PDDR = 0x00200000;
	/**Configures GPIOB pin22 as output*/
	GPIOB->PDDR |= 0x00400000;
	/**Configures GPIOE pin26 as output*/
	GPIOE->PDDR |= 0x04000000;

//nunca poner una constante del lado derecho
//para usar los push buttons siempre se necesita activar los pull enable y el pull select
//Slewrate es para dispositivo serial
//ODE	Open Drain Enable.

	while(1) {
		FSM_Moore[currentState].fptrPort(FSM_Moore[currentState].Estado);

    	/**Reads all the GPIOC*/
		inputValue = GPIOC->PDIR;
		/**Masks the GPIOC in the bit of interest*/
		inputValue = inputValue & 0x40;
		if(FALSE == inputValue){			//SI SE PRESIONA EL SWITCH
			currentState = FSM_Moore[currentState].next[currentState];
		}


    }
    return 0 ;
}
