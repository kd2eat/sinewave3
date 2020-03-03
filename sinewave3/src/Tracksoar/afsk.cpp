/*
 * afsk.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: mqh1
 */

/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* Credit to:
 *
 * Michael Smith for his Example of Audio generation with two timers and PWM:
 * http://www.arduino.cc/playground/Code/PCMAudio
 *
 * Ken Shirriff for his Great article on PWM:
 * http://arcfn.com/2009/07/secrets-of-arduino-pwm.html
 *
 * The large group of people who created the free AVR tools.
 * Documentation on interrupts:
 * http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 */

#include "config.h"
#include "afsk_avr.h"
#include "afsk_pic32.h"
#include "pin.h"
#include "radio_hx1.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
#include <stdint.h>

#ifdef WISP
#include "main.h"


#endif //WISP

// Module consts
#ifndef WISP
// The actual baudrate after rounding errors will be:
// PLAYBACK_RATE / (integer_part_of((PLAYBACK_RATE * 256) / BAUD_RATE) / 256)
static const uint16_t BAUD_RATE       = 1200;
static const uint16_t SAMPLES_PER_BAUD = ((uint32_t)PLAYBACK_RATE << 8) / BAUD_RATE;  // Fixed point 8.8
static const uint16_t PHASE_DELTA_1200 = (((TABLE_SIZE * 1200UL) << 7) / PLAYBACK_RATE); // Fixed point 9.7
static const uint16_t PHASE_DELTA_2200 = (((TABLE_SIZE * 2200UL) << 7) / PLAYBACK_RATE);
static const uint8_t SAMPLE_FIFO_SIZE = 32;


// Module globals
volatile static uint8_t current_byte;
volatile static uint16_t current_sample_in_baud;          // 1 bit = SAMPLES_PER_BAUD samples
volatile static bool go = false;                         // Modem is on
volatile static uint16_t phase_delta;                    // 1200/2200 for standard AX.25
volatile static uint16_t phase;                          // Fixed point 9.7 (2PI = TABLE_SIZE)
volatile static uint16_t packet_pos;                     // Next bit to be sent out
volatile static uint8_t sample_fifo[SAMPLE_FIFO_SIZE];   // queue of samples
volatile static uint8_t sample_fifo_head = 0;            // empty when head == tail
volatile static uint8_t sample_fifo_tail = 0;
volatile static uint32_t sample_overruns = 0;

// The radio (class defined in config.h)
static RadioHx1 radio;
#else //WISP
#define	COUNTER_1200	10			// DAC Timer counter to achieve 1200 hz
#define	COUNTER_2200	5			// DAC Timer counter to achieve 2200 hz

#define DAC_PCT 35          // Percentage of DAC output
#define SINE_RES        24
const uint16_t sinewave[SINE_RES] = {
        (0 * DAC_PCT / 100) + 2048, (530 * DAC_PCT / 100) + 2048, (1023 * DAC_PCT / 100) + 2048,
        (1448 * DAC_PCT / 100) + 2048, (1773 * DAC_PCT / 100) + 2048, (1978 * DAC_PCT / 100) + 2048,
        (2047 * DAC_PCT / 100) + 2048, (1978 * DAC_PCT / 100) + 2048, (1773 * DAC_PCT / 100) + 2048,
        (1448 * DAC_PCT / 100) + 2048, (1024 * DAC_PCT / 100) + 2048, (530 * DAC_PCT / 100) + 2048,
        (0 * DAC_PCT / 100) + 2048, (-530 * DAC_PCT / 100) + 2048, (-1023 * DAC_PCT / 100) + 2048,
        (-1448 * DAC_PCT / 100) + 2048, (-1773 * DAC_PCT / 100) + 2048, (-1978 * DAC_PCT / 100) + 2048,
        (-2047 * DAC_PCT / 100) + 2048, (-1978 * DAC_PCT / 100) + 2048, (-1773 * DAC_PCT / 100) + 2048,
        (-1448 * DAC_PCT / 100) + 2048, (-1024 * DAC_PCT / 100) + 2048, (-530 * DAC_PCT / 100) + 2048,
};


volatile static bool go = false;                         // Modem is on
volatile static uint8_t current_byte;
volatile static uint16_t packet_pos;                     // Next bit to be sent out
volatile static	uint16_t current_timer_counter;			// value of the current timer counter

#endif //WISP
volatile static unsigned int afsk_packet_size = 0;
volatile static const uint8_t *afsk_packet;

#ifndef WISP
// Module functions

inline static bool afsk_is_fifo_full()
{
  return (((sample_fifo_head + 1) % SAMPLE_FIFO_SIZE) == sample_fifo_tail);
}

inline static bool afsk_is_fifo_full_safe()
{
  noInterrupts();
  boolean b = afsk_is_fifo_full();
  interrupts();
  return b;
}

inline static bool afsk_is_fifo_empty()
{
  return (sample_fifo_head == sample_fifo_tail);
}

inline static bool afsk_is_fifo_empty_safe()
{
  noInterrupts();
  bool b = afsk_is_fifo_empty();
  interrupts();
  return b;
}

inline static void afsk_fifo_in(uint8_t s)
{
  sample_fifo[sample_fifo_head] = s;
  sample_fifo_head = (sample_fifo_head + 1) % SAMPLE_FIFO_SIZE;
}

inline static void afsk_fifo_in_safe(uint8_t s)
{
  noInterrupts();
  afsk_fifo_in(s);
  interrupts();
}

inline static uint8_t afsk_fifo_out()
{
  uint8_t s = sample_fifo[sample_fifo_tail];
  sample_fifo_tail = (sample_fifo_tail + 1) % SAMPLE_FIFO_SIZE;
  return s;
}

inline static uint8_t afsk_fifo_out_safe()
{
  noInterrupts();
  uint8_t b = afsk_fifo_out();
  interrupts();
  return b;
}


// Exported functions

void afsk_setup()
{
  // Start radio
  radio.setup();
}
#endif //WISP
void afsk_send(const uint8_t *buffer, int len)
{
  afsk_packet_size = len;
  afsk_packet = buffer;
}

#ifndef WISP
void afsk_start()
{
  phase_delta = PHASE_DELTA_1200;
  phase = 0;
  packet_pos = 0;
  current_sample_in_baud = 0;
  go = true;

  // Prime the fifo
  afsk_flush();

  // Start timer (CPU-specific)
  afsk_timer_setup();

  // Key the radio
  radio.ptt_on();

  // Start transmission
  afsk_timer_start();
}
#else // WISP
void afsk_start()
{
	current_timer_counter = COUNTER_1200;
  packet_pos = 0;
  go = true;

  // Start timer (CPU-specific)
	MX_TIM2_Init();	// Set up on baud timer
	MX_TIM6_Init();	// Set up  DAC/DMA timer
	__HAL_TIM_SET_AUTORELOAD(&htim6,current_timer_counter);		// Set frequency


  // Key the radio
	MySi5351.output_enable(SI5351_CLK0, ENABLE);		// Turn off radio

  // Start transmission
	// Start DAC timer
	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
		Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *) sinewave, SINE_RES, DAC_ALIGN_12B_R) != HAL_OK) {
		Error_Handler(__FILE__, __LINE__);
	}

	// Start Baud timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		Error_Handler(__FILE__, __LINE__);
	}
}
#endif // WISP

#ifndef WISP
bool afsk_flush()
{
  while (! afsk_is_fifo_full_safe()) {
    // If done sending packet
    if (packet_pos == afsk_packet_size) {
      go = false;         // End of transmission
    }
    if (go == false) {
      if (afsk_is_fifo_empty_safe()) {
        afsk_timer_stop();  // Disable modem
        radio.ptt_off();    // Release PTT
        return false;       // Done
      } else {
        return true;
      }
    }

    // If sent SAMPLES_PER_BAUD already, go to the next bit
    if (current_sample_in_baud < (1 << 8)) {    // Load up next bit
      if ((packet_pos & 7) == 0) {         // Load up next byte
        current_byte = afsk_packet[packet_pos >> 3];
      } else {
        current_byte = current_byte / 2;  // ">>1" forces int conversion
      }
      if ((current_byte & 1) == 0) {
        // Toggle tone (1200 <> 2200)
        phase_delta ^= (PHASE_DELTA_1200 ^ PHASE_DELTA_2200);
      }
    }

    phase += phase_delta;
    uint8_t s = afsk_read_sample((phase >> 7) & (TABLE_SIZE - 1));

#ifdef DEBUG_AFSK
    Serial.print((uint16_t)s);
    Serial.print('/');
#endif

#if PRE_EMPHASIS == 1
    if (phase_delta == PHASE_DELTA_1200)
      s = s / 2 + 64;
#endif

#ifdef DEBUG_AFSK
    Serial.print((uint16_t)s);
    Serial.print(' ');
#endif

    afsk_fifo_in_safe(s);

    current_sample_in_baud += (1 << 8);
    if (current_sample_in_baud >= SAMPLES_PER_BAUD) {
#ifdef DEBUG_AFSK
      Serial.println();
#endif
      packet_pos++;
      current_sample_in_baud -= SAMPLES_PER_BAUD;
    }
  }

  return true;  // still working
}

// This is called at PLAYBACK_RATE Hz to load the next sample.
AFSK_ISR
{
  if (afsk_is_fifo_empty()) {
    if (go) {
      sample_overruns++;
    }
  } else {
    afsk_output_sample(afsk_fifo_out());
  }
  afsk_clear_interrupt_flag();
}
#else //WISP

extern "C" void
afsk_ISR()
{
	/*
	 * This timer pops once per BAUD.. note, this is different from typical arduino implimentations that pop many times per baud.
	 * When the timer pops, we simply look to see if we need to adjust the DAC timers (aka frequency) for the next bit.
	 */

	// Are we done transmitting?
	if (packet_pos == afsk_packet_size) {			// If we have completed the last bit.
		go = false;         // End of transmission
//	    MySi5351.output_enable(SI5351_CLK0, DISABLE);		// Turn off radio

		if (HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1) != HAL_OK) {
			Error_Handler(__FILE__, __LINE__);
		}
		// Stop Baud timer
		if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) {
			Error_Handler(__FILE__, __LINE__);
		}
		// Stop DAC timer
		if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
			Error_Handler(__FILE__, __LINE__);
		}
		MX_TIM2_DeInit();	// Set up on baud timer
		MX_TIM6_DeInit();	// Set up  DAC/DMA timer
	}

	// Find the next bit
	if ((packet_pos & 7) == 0) {         // Load up next byte if we've used up the last one (or this is the first)
		current_byte = afsk_packet[packet_pos >> 3];
	} else {
		current_byte = current_byte / 2;  // ">>1" forces int conversion
	}
	if ((current_byte & 1) == 0) {
		// Toggle tone (1200 <> 2200)
		current_timer_counter ^= (COUNTER_1200 ^ COUNTER_2200);	// Switch to the opposite counter value
		__HAL_TIM_SET_AUTORELOAD(&htim6,current_timer_counter);		// Set frequency
	}
	packet_pos++;		// Point to the next bit for the next baud tick.
}

#endif //WISP

#ifdef DEBUG_MODEM
void afsk_debug()
{
  Serial.print("fifo overruns=");
  Serial.println(sample_overruns);

  sample_overruns = 0;
}
#endif



