/*
 * POKISOFT 2022 MBED
 * Creator : CopperBot
 * Creation : 05/2022
 */

#include "mbed.h"

// Use only if necessary, default printf with STLINK should be fine
//UnbufferedSerial usb_serial(USBTX, USBRX, 115200);
//FileHandle *mbed::mbed_override_console(int) {
//    return &usb_serial;
//}

// GPIO
//DigitalOut led(LED1); CAN'T USE NUCLEO LED, USED BY SPI CLOCK (PA5)

// LIDAR
UnbufferedSerial serialLidar(PA_9, PA_10, 115200);
Thread serialLidarThread;
EventQueue serialLidarEventQueue;

// ASSERV UPDATE
#define ASSERV_UPDATE_RATE  1ms
#define ASSERV_FLAG     0x02
EventFlags asservFlag;
Ticker asservTicker;
Thread asservThread(osPriorityRealtime);

// ENCODERS
#define ENC_LEFT 0
#define ENC_RIGHT 1
#define ENC_REVOLUTION 16384
SPI spiAS5047p(PA_7, PA_6, PA_5); // mosi, miso, sclk
DigitalOut enc_left_cs(PA_8);
DigitalOut enc_right_cs(PB_2);
volatile uint16_t enc_raw[2];
volatile int32_t enc_revol[2];
volatile int64_t enc_count[2];
volatile int64_t enc_dir[2];

// Main Debug
#define MAIN_LOOP_RATE     200ms
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

uint16_t getEncodeurValue(DigitalOut *chip_select){
    chip_select->write(0);
    uint8_t receivedDataH = (0x3F & spiAS5047p.write(0xFF)); //Get the first part (8bits)
    uint8_t receivedDataL = spiAS5047p.write(0xFF); //Get the second part (8bits)
    chip_select->write(1);
    return (uint16_t)((receivedDataH << 8) | (receivedDataL & 0xff)); //Combine the two parts to get a 16bits
}

void encoderUpdate(int encoder) {

    uint16_t countOld = enc_raw[encoder];
    uint16_t countNow = 0;

    if(encoder == ENC_LEFT)
        countNow = getEncodeurValue(&enc_left_cs);
    else
        countNow = getEncodeurValue(&enc_right_cs);

    int32_t countDelta = (int16_t) (countNow) - (int16_t) (countOld);

    enc_raw[encoder] = countNow;

    // Check if we did an overflow
    if (countDelta >= ((int32_t) ENC_REVOLUTION / 2)) {
        enc_revol[encoder] = enc_revol[encoder] - 1 ;
    } else if (countDelta <= -((int32_t) ENC_REVOLUTION / 2)) {
        enc_revol[encoder] = enc_revol[encoder] + 1 ;
    }

    enc_count[encoder] = enc_dir[encoder] * enc_revol[encoder] * ((int64_t) ENC_REVOLUTION) + ( enc_dir[encoder] * (int64_t) countNow );

}

void asservUpdate() {

    while(true){
        asservFlag.wait_any(ASSERV_FLAG);

        // Convert current rate of the loop in seconds (float)
//        auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(ASSERV_UPDATE_RATE);
//        float dt_pid = f_secs.count();


//        // Read and updates global encoders here
//        enc_left_count = getEncodeurValue(&enc_left_cs);

        encoderUpdate(ENC_LEFT);
        encoderUpdate(ENC_RIGHT);

    }

}

void asservSetFlag() {
    asservFlag.set(ASSERV_FLAG);
}

// Update main
void mainLoopUpdate() {
	mainLoopFlag.set(MAIN_LOOP_FLAG);
}

int main() {
	// Initialize GPIO

    // Init encodeurs SPI
    enc_left_cs = 1;
    enc_right_cs = 1;
    spiAS5047p.format(8, 1);
    spiAS5047p.frequency(1000000);
    enc_raw[ENC_LEFT] = 0;
    enc_raw[ENC_RIGHT] = 0;
    enc_count[ENC_LEFT] = 0;
    enc_count[ENC_RIGHT] = 0;
    enc_revol[ENC_LEFT] = 0;
    enc_revol[ENC_RIGHT] = 0;
    enc_dir[ENC_LEFT] = -1;
    enc_dir[ENC_RIGHT] = 1;

	// Setup Lidar Thread
	serialLidarThread.start(callback(&serialLidarEventQueue, &EventQueue::dispatch_forever));
	serialLidar.attach(&rxLidarCallback);

	// asserv update
    asservThread.start(asservUpdate);
	asservTicker.attach(&asservSetFlag, ASSERV_UPDATE_RATE);

	// Main setup
	mainLoopTicker.attach(&mainLoopUpdate, MAIN_LOOP_RATE);

	int i = 0;
    char printf_buffer[100];
	while (true) {
		mainLoopFlag.wait_any(MAIN_LOOP_FLAG); // ... So instead we use a ticker to trigger a flag every second.


		printf("Pokirobot v1 alive since %ds (enc left = %lld, enc right = %lld) ...\n", i++, enc_count[ENC_LEFT], enc_count[ENC_RIGHT]);

        // more speeeeeeed
//        sprintf(printf_buffer, "Pokirobot v1 alive since %ds (enc left = %lld) ...\n", i++,enc_count[ENC_LEFT]);
//        usb_serial.write(printf_buffer);

	}
}

