#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "i2c_master.h"
#include "liquid_crystal_i2c.h"

// LCD string buffers
char str_joy_x[6];
char str_joy_y[6];
char str_dist[8];

// Convert integer to string
void int_to_str(int num, char* str) {
	if (num == 0) {
		str[0] = '0';
		str[1] = '\0';
		return;
	}
	char temp[16];
	uint8_t i = 0;
	uint8_t negative = 0;

	if (num < 0) {
		negative = 1;
		num = -num;
	}

	while (num > 0) {
		temp[i++] = '0' + (num % 10);
		num /= 10;
	}

	uint8_t j = 0;
	if (negative) str[j++] = '-';

	for (int8_t k = i - 1; k >= 0; k--) {
		str[j++] = temp[k];
	}
	str[j] = '\0';
}

void hardware_init(void) {
	// --- JOYSTICK ADC INIT ---
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// --- JOYSTICK BUTTON INIT (SW) ---
	DDRD &= ~(1 << PD7);
	PORTD |= (1 << PD7);

	// --- ULTRASONIC RADAR INIT ---
	DDRD |= (1 << PD3);
	DDRB &= ~(1 << PB5);
	PORTD &= ~(1 << PD3);

	// --- POWER CIRCUIT PIN DIRECTION ---
	DDRB |= (1 << PB1);
	DDRB |= (1 << PB0) | (1 << PB4);
	DDRD |= (1 << PD4) | (1 << PD2);
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

// Precision ASM loop (6 NOPs)
uint16_t get_distance_raw(void) {
	PORTD |= (1 << PD3);
	_delay_us(10);
	PORTD &= ~(1 << PD3);

	uint32_t timeout = 0;
	while (!(PINB & (1 << PB5))) {
		timeout++;
		if (timeout > 40000) return 0;
	}

	uint16_t duration = 0;
	while (PINB & (1 << PB5)) {
		duration++;
		__asm__ __volatile__(
			"nop \n\t"
			"nop \n\t"
			"nop \n\t"
			"nop \n\t"
			"nop \n\t"
			"nop \n\t"
		);
		if (duration > 35000) break;
	}

	// Suppress residual acoustic echoes in the room
	_delay_ms(25);

	if (duration > 33000 || duration < 10) return 0;
	return duration;
}

// Joystick deadzone and calibration settings
int8_t calculate_speed(uint16_t adc_val, uint8_t invert) {
	int8_t speed = 0;
	if (adc_val >= 480 && adc_val <= 520) return 0;

	if (adc_val > 520) {
		if (adc_val < 620)       speed = 1;
		else if (adc_val < 730)  speed = 2;
		else if (adc_val < 840)  speed = 3;
		else if (adc_val < 970)  speed = 4;
		else                     speed = 5;
	}
	else {
		if (adc_val > 400)       speed = -1;
		else if (adc_val > 300)  speed = -2;
		else if (adc_val > 190)  speed = -3;
		else if (adc_val > 60)   speed = -4;
		else                     speed = -5;
	}

	if (invert) speed = -speed;
	return speed;
}

int main(void) {
	hardware_init();
	i2c_master_init();
	_delay_ms(100);

	LiquidCrystalDevice_t lcd = lq_init(0x27, 16, 2, LCD_5x8DOTS);
	lq_turnOnBacklight(&lcd);
	lq_clear(&lcd);

	uint8_t lcd_counter = 0;
	int8_t motor_speed_x = 0;
	int8_t motor_speed_y = 0;

	while(1) {
		uint16_t raw_x = adc_read(0);
		uint16_t raw_y = adc_read(1);

		motor_speed_x = calculate_speed(raw_x, 1);
		motor_speed_y = calculate_speed(raw_y, 0);

		lcd_counter++;
		if (lcd_counter > 30) {
			int_to_str(motor_speed_x, str_joy_x);
			int_to_str(motor_speed_y, str_joy_y);

			// Line 1: Output joystick axis data
			lq_setCursor(&lcd, 0, 0);
			lq_print(&lcd, "X:");
			lq_print(&lcd, str_joy_x);
			lq_print(&lcd, " Y:");
			lq_print(&lcd, str_joy_y);
			lq_print(&lcd, "    ");

			// Event-driven check: Trigger measurement only when SW is pressed (active low)
			if (!(PIND & (1 << PD7))) {
				lq_print(&lcd, " [SW]  ");

				// Line 2: Process a series of 4 ultrasonic measurements
				lq_setCursor(&lcd, 1, 0);
				
				uint32_t duration_sum = 0;
				uint8_t valid_samples = 0;

				for(uint8_t i = 0; i < 4; i++) {
					uint16_t sample = get_distance_raw();
					if (sample > 0) {
						duration_sum += sample;
						valid_samples++;
					}
				}

				if (valid_samples == 0) {
					lq_print(&lcd, "D:<1cm          ");
				}
				else {
					// Math scaling under stable ASM clock (divider 64)
					uint16_t avg_duration = duration_sum / valid_samples;
					uint16_t dist_mm = (avg_duration * 10) / 64;

					if (dist_mm < 100) {
						int_to_str(dist_mm, str_dist);
						lq_print(&lcd, "D:");
						lq_print(&lcd, str_dist);
						lq_print(&lcd, "mm        ");
					}
					else {
						uint16_t dist_cm = dist_mm / 10;
						int_to_str(dist_cm, str_dist);
						lq_print(&lcd, "D:");
						lq_print(&lcd, str_dist);
						lq_print(&lcd, "cm        ");
					}
				}
			} else {
				// Standby mode: Sonar is off, processor resources are saved
				lq_print(&lcd, "       ");
				
				lq_setCursor(&lcd, 1, 0);
				lq_print(&lcd, "D: READY        ");
			}

			lcd_counter = 0;
		}
		_delay_ms(1);
	}
}
