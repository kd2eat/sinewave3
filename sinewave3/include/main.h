/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#include "stm32l1xx_hal.h"

#include <stdio.h>
#include "diag/Trace.h"
#include "mxconstants.h"
#include "Contrib/si5351.h"



typedef struct  {
	uint64_t	XmitFrequency;
	volatile uint32_t	mSec;
	volatile uint32_t	Seconds;
	volatile int	ADCdone;
	volatile int	GPSTXcomplete;
	volatile int	GPSRXcomplete;

	volatile int	LEDsOn;		// Flag indicating whether to be flashing LEDs.
	int GPSstate;
	volatile int WhichClock;
	int NumberOfTransmits;		// Number of times we have transmitted
	int TelemGPSresets;
	int TelemAltitude;
	int TelemBattVolts;
	int TelemSolarVolts;
	int TelemSatellites;
	int TelemTemperature;
	char TelemGridSquare[6];
	volatile int	Tim7Complete;
	volatile int	CurrentTone;		// Current tone being played in the tonearray
	int Fenced;
	volatile int	LSIcaptureState;
	volatile uint32_t	LSIfirstValue;
	volatile uint32_t	LSIcapture;
	int	JustSentSecondaryTelem;
	uint32_t	LSIfrequency;
	I2C_HandleTypeDef I2cHandle;

}  MyGlobals;
extern MyGlobals Globals;
extern Si5351 MySi5351;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac_ch1;

extern TIM_HandleTypeDef htim2;			// Baud rate counter
extern TIM_HandleTypeDef htim6;			// Timer for DAC
extern TIM_HandleTypeDef htim7;			// Unused in demo code - used for WSPR
extern void Error_Handler(char *, int);
extern  void MX_TIM6_Init(void);
extern  void MX_TIM6_DeInit(void);
extern  void MX_TIM2_Init(void);
extern  void MX_TIM2_DeInit(void);
extern void afsk_send(const uint8_t *buffer, int len);
extern void afsk_start();
extern "C" void afsk_ISR();

//#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
