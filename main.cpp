/* File: main.cpp
 * Contains base main function and usually all the other stuff that avr does...
 */
/* Copyright (c) 2012-2013 Domen Ipavec (domen.ipavec@z-v.si)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <stdint.h>

#include "bitop.h"

uint16_t EEMEM signal_min_ee[3];
uint16_t EEMEM signal_max_ee[3];

volatile uint16_t signal_min[3];
volatile uint16_t signal_max[3];
volatile uint16_t signal[3];
volatile bool signal_update[3] = {false, false, false};
volatile uint8_t signal_divider[3];

volatile uint16_t signal_start[3];
volatile uint8_t signal_value[3];

bool signal_calibrated[3] = {false,false,false};

static const uint8_t SIGNAL_STEP = 16;

volatile uint8_t mode = 1;
volatile uint16_t battery = 65535;

void update_divider(uint8_t i) {
	signal_divider[i] = (signal_max[i] - signal_min[i] - 2*SIGNAL_STEP)>>8;
	if (signal_divider[i] < 1) {
		signal_divider[i] = 1;
	}
}

ISR(TIMER2_COMPA_vect) {
	if (mode == 0) {
		CLEARBIT(DDRA, PA3);
	} else if (mode == 1) {
		SETBIT(DDRA, PA3);
		if (OCR0A == 24) {
			OCR0A = 18;
		} else {
			OCR0A = 24;
		}
	} else if (mode == 2) {
		OCR0A = 21;
		TOGGLEBIT(DDRA, PA3);
	}
}

ISR(PCINT1_vect) {
	static uint8_t last = 0;
	uint8_t curr = PINB & 0b0111;
	uint8_t diff = last ^ (curr);
	uint16_t t = TCNT1;

	for (uint8_t i = 0; i < 3; i++) {
		if (BITSET(diff, i)) {
			if (BITSET(curr, i)) {
				signal_start[i] = t;
			} else {
				signal[i] = t - signal_start[i];
				if (t < signal_start[i]) {
					signal[i] -= 15536;
				}
				signal_update[i] = true;
			}
		}
	}

	last = curr;
}

ISR(ADC_vect) {
	battery = ADC;
}

int main() {
	// init
	// init button
	SETBIT(PORTA, PA7);

	eeprom_read_block((void*)&signal_min, (const void *)&signal_min_ee, 6);
	eeprom_read_block((void*)&signal_max, (const void *)&signal_max_ee, 6);
	for (uint8_t i = 0; i < 3; i++) {
		update_divider(i);
	}

	// configure timer 0 to 50% pwm for speaker
	// enable tocc2, it is by default set to OC0B
	SETBIT(TOCPMCOE, TOCC2OE);
	// toggle OC0B on compare
	SETBIT(TCCR0A, COM0B0);
	// clear timer on compare
	SETBIT(TCCR0A, WGM01);
	// init OCR0A
	OCR0A = 100;
	// enable and set prescaler to 256 (31250Hz)
	TCCR0B = BIT(CS02);

	// configure timer 2 for speaker toggle
	// clear timer on compare
	SETBIT(TCCR2B, WGM22);
	// enable compare a interrupt
	SETBIT(TIMSK2, OCIE2A);
	// init OCR0A
	OCR2A = 4096;
	// enable and set prescaler to 1024 (7812.5Hz)
	SETBITS(TCCR2B, BIT(CS20) | BIT(CS22));

	// configure adc
	// set internal 4.096 reference
	ADMUXB = BIT(REFS1) | BIT(REFS0);
	// enable freerunning with interrupts
	ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADIE) | BIT(ADATE) | BIT(ADPS2) | BIT(ADPS1);

	// init pc interrupts 8,9,10
	GIMSK = 0b00100000;
	PCMSK1 = 0b00000111;

	// timer clear on compare, 100ms/16
	TIMSK1 = 0b00000010;
	OCR1A = 49999;
	TCCR1B = 0b00001001;

	// enable interrupts
	sei();

	for (;;) {
		for (uint8_t i = 0; i < 3; i++) {
			if (signal_update[i]) {
				if (BITCLEAR(PINA, PA7)) {
					if (!signal_calibrated[i]) {
						signal_min[i] = signal[i];
						signal_max[i] = signal[i];
						signal_divider[i] = 1;
						eeprom_write_block((void*)signal_min, (void*)signal_min_ee, 6);
						eeprom_write_block((void*)signal_max, (void*)signal_max_ee, 6);
						signal_calibrated[i] = true;
					}
					if (signal[i] < signal_min[i]) {
						while (signal[i] < signal_min[i]) {
							signal_min[i] -= SIGNAL_STEP;
						}
						eeprom_write_word(&signal_min_ee[i], signal_min[i]);
						update_divider(i);
					}
					if (signal[i] > signal_max[i]) {
						while (signal[i] > signal_max[i]) {
							signal_max[i] += SIGNAL_STEP;
						}
						eeprom_write_word(&signal_max_ee[i], signal_max[i]);
						update_divider(i);
					}
				}
				uint16_t value = (signal[i] - signal_min[i] + SIGNAL_STEP)/signal_divider[i];
				if (signal[i] < signal_min[i]) {
					value = 0;
				}
				if (value > 255) {
					value = 255;
				}
				signal_value[i] = value;
				signal_update[i] = false;
			}
		}

		if (battery < 756) {
			mode = 2;

		} else if (signal_value[0] > 30) {
			mode = 1;
			OCR2A = uint16_t(signal_value[0])<<7;
		} else {
			mode = 0;
		}
	}
}
