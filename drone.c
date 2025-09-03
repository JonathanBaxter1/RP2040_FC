#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define LED_PIN1 4
#define LED_PIN2 5
#define MAIN_LOOP_TIMER 0
#define MAIN_LOOP_FREQ 2000

unsigned int frame = 0;

void mainLoop();
void setupTimer(unsigned int timer, unsigned int irq, unsigned int loopFreq, irq_handler_t function);
void resetTimer(unsigned int timer, unsigned int sampleInterval);

int main()
{
	stdio_init_all();
	gpio_init(LED_PIN1);
	gpio_init(LED_PIN2);
	gpio_set_dir(LED_PIN1, GPIO_OUT);
	gpio_set_dir(LED_PIN2, GPIO_OUT);

	setupTimer(MAIN_LOOP_TIMER, TIMER_IRQ_0, MAIN_LOOP_FREQ, mainLoop);

	while (1) {
		printf("frame: %d\n", frame);
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
void mainLoop()
{
	resetTimer(MAIN_LOOP_TIMER, 1000000/MAIN_LOOP_FREQ);
	frame++;
}

// Configures an interrupt routine to run when the timer goes off
void setupTimer(unsigned int timer, unsigned int irq, unsigned int loopFreq, irq_handler_t function)
{
	hw_set_bits(&timer_hw->inte, 1u << timer);
	irq_set_exclusive_handler(irq, function);
	irq_set_enabled(irq, true);
	resetTimer(timer, 1000000/loopFreq);
}


void resetTimer(unsigned int timer, unsigned int sampleInterval)
{
	hw_clear_bits(&timer_hw->intr, 1u << timer); // ACK interrupt
	timer_hw->alarm[timer] = timer_hw->timelr + sampleInterval;
}
