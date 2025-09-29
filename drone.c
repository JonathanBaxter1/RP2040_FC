#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define NUM_CHANNELS 16
#define BITS_PER_CHANNEL 11
#define CHANNEL_DATA_BYTES NUM_CHANNELS*BITS_PER_CHANNEL/8 // 22

// GPIO pins
#define LED_PIN1 4
#define LED_PIN2 5
#define TX_PIN 16
#define RX_PIN 17
#define ESC_EN_PIN 18

// ELRS UART configuration
#define UART_ID uart0
#define UART_DATA_BITS 8
#define UART_STOP_BITS 1
#define UART_PARITY UART_PARITY_NONE
#define UART_BUFFER_SIZE 256

#define MAIN_LOOP_TIMER 0
#define MAIN_LOOP_FREQ 100

unsigned int frame = 0;
unsigned char packetDataRaw[CHANNEL_DATA_BYTES + 2] = {0}; // 1 byte for type, 1 byte for CRC8
unsigned int channelData[NUM_CHANNELS] = {0};
unsigned char uartBuffer[UART_BUFFER_SIZE] = {0};
unsigned int uartBufferIndex = 0;

void controlLoop();
void setupTimer(unsigned int timer, unsigned int irq, unsigned int loopFreq, irq_handler_t function);
void resetTimer(unsigned int timer, unsigned int sampleInterval);
void getReceiverData(unsigned int frame);
char calculateCRC8(unsigned char* data, unsigned char divisor, unsigned int dataLenBytes);
void onUartRx();

int main()
{
	stdio_init_all();
	uart_init(UART_ID, 420000);

	// GPIO setup
	gpio_init(LED_PIN1);
	gpio_init(LED_PIN2);
	gpio_init(ESC_EN_PIN);
	gpio_set_dir(LED_PIN1, GPIO_OUT);
	gpio_set_dir(LED_PIN2, GPIO_OUT);
	gpio_set_dir(ESC_EN_PIN, GPIO_OUT);
	gpio_put(ESC_EN_PIN, 1);

	gpio_set_function(TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(RX_PIN, GPIO_FUNC_UART);

//	uart_set_hw_flow(UART_ID, false, false);
	uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
//	uart_set_fifo_enabled(UART_ID, true);

	// UART interrupt setup
	uart_set_irqs_enabled(UART_ID, true, false);
	irq_set_exclusive_handler(UART0_IRQ, onUartRx);
	irq_set_enabled(UART0_IRQ, true);

	// Control loop interrupt routine
	setupTimer(MAIN_LOOP_TIMER, TIMER_IRQ_0, MAIN_LOOP_FREQ, controlLoop);

	// Main loop
	while (1) {

		// Blink LEDs
		gpio_put(LED_PIN1, 1);
		gpio_put(LED_PIN2, 0);
		sleep_ms(100);
		gpio_put(LED_PIN1, 0);
		gpio_put(LED_PIN2, 1);
		sleep_ms(100);

		printf("Roll: %d, Pitch: %d, Throttle: %d, Yaw: %d\n", channelData[0], channelData[1], channelData[2], channelData[3]);
	}
	return 0;
}

// Get IMU and control data, calculate PID, then update ESCs
void controlLoop()
{
	static int frame = 0;

	resetTimer(MAIN_LOOP_TIMER, 1000000/MAIN_LOOP_FREQ);
	frame++;

	getReceiverData(frame);
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

// Reads input from the ELRS receiver
void getReceiverData(unsigned int frame)
{
//	static unsigned int goodPackets = 0;
//	static unsigned int badPackets = 0;

	static unsigned int curIndex = 0;
	static unsigned int crcIndex = 0;
	static unsigned int frameSize = 0;
	static unsigned char packetType = 0;
	if (uartBufferIndex == 0) return;
	for (unsigned int i = 0; i < uartBufferIndex; i++) {
		unsigned char uartChar = uartBuffer[i];
		if (curIndex == 0) {
			if (uartChar != 0xC8) continue;
			curIndex++;
		} else if (curIndex == 1) {
			frameSize = (unsigned int)uartChar;
			crcIndex = 1 + frameSize;
			curIndex++;
		} else if (curIndex > 1 + frameSize) {
			curIndex = 0;
			crcIndex = 0;
			frameSize = 0;
			packetType = 0;
		} else if (curIndex >= 2) {
			if (curIndex == 2) {
				packetType = uartChar;
			}
			if (packetType == 0x16 && frameSize == 24 && curIndex-2 < sizeof(packetDataRaw)/sizeof(unsigned char)) {
				// Packet data will be used for channel data and crc check
				packetDataRaw[curIndex-2] = uartChar;
			}
			curIndex++;
		}
	}
	uartBufferIndex = 0;

	// Check CRC value
	unsigned char crcDivisor = 0xD5;
	unsigned char crcResult = calculateCRC8(packetDataRaw, crcDivisor, sizeof(packetDataRaw));
	if (crcResult == 0x00) {
//		goodPackets++;
		// Unpack channel data if CRC is correct
		for (unsigned int curChannel = 0; curChannel < NUM_CHANNELS; curChannel++) {
			channelData[curChannel] = 0;
		}
		for (unsigned int curIndex = 0; curIndex < CHANNEL_DATA_BYTES; curIndex++) {
			for (unsigned int curBit = 0; curBit < 8; curBit++) {
				unsigned int curChannel = (curIndex*8 + curBit)/11;
				unsigned int curChannelBit = (curIndex*8 + curBit)%11;
				unsigned char insertBit = (packetDataRaw[curIndex + 1] & (1 << curBit)) >> curBit;
				channelData[curChannel] |= (insertBit << curChannelBit);
			}
		}
	} else {
//		badPackets++;
		printf("Packet Lost\n");
	}

//	int linkQuality = (int)(100.0*(float)goodPackets/(float)(goodPackets+badPackets));
//	if (frame%100 == 0) {
//		printf("LQ: %d%%\n", linkQuality);
//	}
}

char calculateCRC8(unsigned char* data, unsigned char divisor, unsigned int dataLenBytes)
{
/*	unsigned char crcRegister = 0;
	unsigned int dataLenBits = dataLenBytes*8;
	unsigned int inputStreamIndex = 0;
	while (inputStreamIndex < dataLenBits) {
		// Shift crcRegister left until MSB = 1, then shift once more
		unsigned int lastShift = 0;
		while (inputStreamIndex < dataLenBits) {
			if (lastShift == 1) break;
			if ((crcRegister & 0x80) == 0x80) lastShift = 1;
			crcRegister <<= 1;
			unsigned int indexByte = inputStreamIndex/8;
			unsigned int indexBit = inputStreamIndex%8;
			unsigned char insertBit = (data[indexByte] & (1 << indexBit)) >> indexBit;
			crcRegister |= insertBit;
			inputStreamIndex++;
		}

		// Bitwise XOR crc register with divisor
		crcRegister ^= divisor;
	}
	return crcRegister;*/
	unsigned char crc = 0;
	for (unsigned int i = 0; i < dataLenBytes; i++) {
		crc ^= data[i];  // XOR input byte into crc
		for (unsigned int bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ divisor;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

void onUartRx()
{
	while (uart_is_readable(UART_ID)) {
		unsigned char uartChar = uart_getc(UART_ID);
		if (uartBufferIndex < UART_BUFFER_SIZE) {
			uartBuffer[uartBufferIndex] = uartChar;
			uartBufferIndex++;
		} else {
			uartBufferIndex = 0;
		}
	}
}
