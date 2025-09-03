#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// GPIO pins
#define LED_PIN1 4
#define LED_PIN2 5
#define TX_PIN 16
#define RX_PIN 17

// ELRS UART configuration
#define UART_ID uart0
#define UART_DATA_BITS 8
#define UART_STOP_BITS 1
#define UART_PARITY UART_PARITY_NONE

#define MAIN_LOOP_TIMER 0
#define MAIN_LOOP_FREQ 2000

unsigned int frame = 0;

void controlLoop();
void setupTimer(unsigned int timer, unsigned int irq, unsigned int loopFreq, irq_handler_t function);
void resetTimer(unsigned int timer, unsigned int sampleInterval);

int main()
{
	stdio_init_all();
	uart_init(UART_ID, 420000);

	// GPIO setup
	gpio_init(LED_PIN1);
	gpio_init(LED_PIN2);
	gpio_set_dir(LED_PIN1, GPIO_OUT);
	gpio_set_dir(LED_PIN2, GPIO_OUT);
	gpio_set_function(TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(RX_PIN, GPIO_FUNC_UART);

	uart_set_hw_flow(UART_ID, false, false);
	uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
	uart_set_fifo_enabled(UART_ID, true);

	// Control loop interrupt routine
	setupTimer(MAIN_LOOP_TIMER, TIMER_IRQ_0, MAIN_LOOP_FREQ, controlLoop);

	// Main loop
	while (1) {

		// Blink LEDs
		gpio_put(LED_PIN1, 1);
		gpio_put(LED_PIN2, 0);
		sleep_ms(500);
		gpio_put(LED_PIN1, 0);
		gpio_put(LED_PIN2, 1);
		sleep_ms(500);
	}
	return 0;
}

// Get IMU and control data, calculate PID, then update ESCs
void controlLoop()
{
	static int frame = 0;

	resetTimer(MAIN_LOOP_TIMER, 1000000/MAIN_LOOP_FREQ);
	frame++;

	if (frame%1000 == 0) {
		printf("%d seconds\n", frame/1000);
	}
	// Get data from controller
}

// Configures an interrupt routine to run when the timer goes off
void setupTimer(unsigned int timer, unsigned int irq, unsigned int loopFreq, irq_handler_t function)
{
	hw_set_bits(&timer_hw->inte, 1u << timer);
	irq_set_exclusive_handler(irq, function);
	irq_set_enabled(irq, true);
	resetTimer(timer, 1000000/loopFreq);
}

// Acknowledge interrupt and set timer for next cycle
void resetTimer(unsigned int timer, unsigned int sampleInterval)
{
	hw_clear_bits(&timer_hw->intr, 1u << timer); // ACK interrupt
	timer_hw->alarm[timer] = timer_hw->timelr + sampleInterval;
}
