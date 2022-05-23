/*
 * Creator : CopperBot
 * Creation : 05/2022
 */

#include "mbed.h"

// GPIO
DigitalOut led(LED1);

// LIDAR
UnbufferedSerial serialLidar(PA_9, PA_10, 115200);
Thread serialLidarThread;
EventQueue serialLidarEventQueue;

// ENCODERS UPDATE
#define ENCODERS_UPDATE_RATE  5ms
Ticker encodersTicker;


// Main Debug
#define MAIN_LOOP_RATE     1s
#define MAIN_LOOP_FLAG     0x01
Ticker mainLoopTicker;
EventFlags mainLoopFlag;

void process_serialLidarTX(char carac) {
	printf("\t\t\t\tCharacter received from lidar : 0x%X\n", carac);

}

void rxLidarCallback() {

	char c;
	if (serialLidar.read(&c, 1)) {
		serialLidarEventQueue.call(process_serialLidarTX, c);
	}
}

void encodersUpdate() {
	// Read and updates global encoders here
}


// Update main
void mainLoopUpdate() {
	mainLoopFlag.set(MAIN_LOOP_FLAG);
}

int main() {
	// Initialize GPIO
	led = 0;

	// Setup Lidar Thread
	serialLidarThread.start(callback(&serialLidarEventQueue, &EventQueue::dispatch_forever));
	serialLidar.attach(&rxLidarCallback);

	// Encoders update
	encodersTicker.attach(&encodersUpdate, ENCODERS_UPDATE_RATE);

	// Main setup
	mainLoopTicker.attach(&mainLoopUpdate, MAIN_LOOP_RATE);

	int i = 0;
	while (true) {
		mainLoopFlag.wait_any(MAIN_LOOP_FLAG); // ... So instead we use a ticker to trigger a flag every second.
		led = !led;

		printf("Pokirobot v1 alive since %ds...\n", i++);

	}
}

