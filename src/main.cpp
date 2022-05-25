/*
 * POKISOFT 2022 MBED
 * Creator : CopperBot
 * Creation : 05/2022
 */

#include "mbed.h"
#include "PID.h"
#include "Biquad.h"
#include "servo.h"

// Use only if necessary, default printf with STLINK should be fine
//UnbufferedSerial usb_serial(USBTX, USBRX, 115200);
//FileHandle *mbed::mbed_override_console(int) {
//    return &usb_serial;
//}

// GPIO
//DigitalOut led(LED1); CAN'T USE NUCLEO LED, USED BY SPI CLOCK (PA5)
DigitalOut debug_pin(PC_8);

// LIDAR
UnbufferedSerial serialLidar(PA_9, PA_10, 115200);
Thread serialLidarThread;
EventQueue serialLidarEventQueue(1024 * EVENTS_EVENT_SIZE);

// ASSERV UPDATE
#define ASSERV_UPDATE_RATE  1ms
#define ASSERV_FLAG     0x02
EventFlags asservFlag;
Ticker asservTicker;
Thread asservThread(osPriorityRealtime);
sixtron::PID *pid_left, *pid_right;
volatile float motors_speed[2] = {0.0f};
volatile float motors_pwm[2] = {0.0f};
volatile float dt_pid = 0.0f;
Biquad filter_speed_left, filter_speed_right;

// ENCODERS
#define ENC_LEFT 0
#define ENC_RIGHT 1
#define ENC_REVOLUTION 16384
SPI spiAS5047p(PA_7, PA_6, PA_5); // mosi, miso, sclk
DigitalOut enc_left_cs(PA_8);
DigitalOut enc_right_cs(PB_2);
volatile uint16_t enc_raw[2] = {0};
volatile int32_t enc_revol[2] = {0};
volatile int64_t enc_count[2] = {0};
volatile int64_t enc_dir[2] = {0};
volatile int64_t enc_offset[2] = {0};

// ELECTRO AIMANT
DigitalOut electroaimant(PC_4);



// MOTORS
PwmOut motor_left(PB_7);
PwmOut motor_right(PB_6);
DigitalOut motor_dir_left(PA_14);
DigitalOut motor_dir_right(PA_13);

// Main Debug
#define MAIN_LOOP_RATE     200ms
#define MAIN_LOOP_FLAG     0x01
Ticker mainLoopTicker;
EventFlags mainLoopFlag;

// LIDAR
#define LIDAR_MODE_HEADER 	0
#define LIDAR_MODE_MSG		1
#define LIDAR_MSG_LENGTH	32
#define CAMSENSE_X1_SPEED_L_INDEX 0
#define CAMSENSE_X1_SPEED_H_INDEX 1
#define CAMSENSE_X1_START_ANGLE_L_INDEX 2
#define CAMSENSE_X1_START_ANGLE_H_INDEX 3
#define CAMSENSE_X1_END_ANGLE_L_INDEX 28
#define CAMSENSE_X1_END_ANGLE_H_INDEX 29
#define CAMSENSE_X1_MAX_INDEX		400
uint8_t lidar_mode = 0;
uint8_t lidar_header_incr = 0;
uint8_t lidar_msg_incr = 0;
uint8_t lidar_msg[LIDAR_MSG_LENGTH];
float lidar_hz;
float lidar_startAngle,lidar_endAngle;
float lidar_offset = 16.0f; // 0 degrees seems to be 16 degrees of center.
float lidar_IndexMultiplier = 400.0f / 360.0f;
int lidar_index;
uint16_t lidar_distanceArray[CAMSENSE_X1_MAX_INDEX];
uint8_t lidar_qualityArray[CAMSENSE_X1_MAX_INDEX];


void process_serialLidarTX(char carac) {
//	printf("\t\t\t\tCharacter received from lidar : 0x%X\n", carac);
	if(lidar_mode == LIDAR_MODE_HEADER) {

		if ((lidar_header_incr == 0) && (carac == 0x55)){
			lidar_header_incr++;
		} else if ((lidar_header_incr == 1) && (carac == 0xAA)){
			lidar_header_incr++;
		} else if ((lidar_header_incr == 2) && (carac == 0x03)){
			lidar_header_incr++;
		} else if ((lidar_header_incr == 3) && (carac == 0x08)){
			lidar_header_incr = 0;
			lidar_mode = LIDAR_MODE_MSG;
		} else {
			//Error
			lidar_header_incr = 0;
			lidar_mode = LIDAR_MODE_HEADER;
//			printf("Lidar ERROR (carac = %X)\n", carac);
		}

	} else if (lidar_mode == LIDAR_MODE_MSG) {
		lidar_msg[lidar_msg_incr] = carac;
		lidar_msg_incr++;

		if(lidar_msg_incr == LIDAR_MSG_LENGTH){
//			printf("Lidar lidar_msg Received\n");
			lidar_mode = LIDAR_MODE_HEADER;
			lidar_msg_incr=0;

			// Do calculus
			lidar_hz = float((uint16_t) (lidar_msg[CAMSENSE_X1_SPEED_H_INDEX] << 8) | lidar_msg[CAMSENSE_X1_SPEED_L_INDEX]) / 3840.0f; // 3840.0 = (64 * 60)
			lidar_startAngle = float(lidar_msg[CAMSENSE_X1_START_ANGLE_H_INDEX] << 8 | lidar_msg[CAMSENSE_X1_START_ANGLE_L_INDEX]) / 64.0f - 640.0f;
			lidar_endAngle   = float(lidar_msg[CAMSENSE_X1_END_ANGLE_H_INDEX] << 8 | lidar_msg[CAMSENSE_X1_END_ANGLE_L_INDEX])   / 64.0f - 640.0f;

			//Get distance


			float step = 0.0;
			if(lidar_endAngle > lidar_startAngle)
			{
				step = (lidar_endAngle - lidar_startAngle) / 8;
			}
			else
			{
				step = (lidar_endAngle - (lidar_startAngle - 360)) / 8;
			}

			for(int i = 0; i < 8; i++) // for each of the 8 samples
			{
				float sampleAngle = (lidar_startAngle + step * i) + (lidar_offset + 180);
				float sampleIndexFloat = sampleAngle * lidar_IndexMultiplier; // map 0-360 to 0-400
				int sampleIndex = round(sampleIndexFloat); // round to closest value.
				lidar_index = sampleIndex % 400; // limit sampleIndex between 0 and 399 to prevent segmentation fault

				uint8_t distanceL = lidar_msg[8+(i*3)];
				uint8_t distanceH = lidar_msg[9+(i*3)];
				uint8_t quality = lidar_msg[10+(i*3)];


				if(quality == 0) // invalid data
				{
					lidar_distanceArray[lidar_index] = 0;
					lidar_qualityArray[lidar_index] = 0;
				}
				else
				{
					lidar_distanceArray[lidar_index] = ((uint16_t) distanceH << 8) | distanceL;
					lidar_qualityArray[lidar_index] = quality;
				}

//				printf("a = %f, d = %d\n", lidar_startAngle, lidar_distanceArray[lidar_index]);

			}
		}



	}
}

void rxLidarCallback() {

	char c;
	if (serialLidar.read(&c, 1)) {
		serialLidarEventQueue.call(process_serialLidarTX, c);
	}
}

uint16_t getEncodeurValue(DigitalOut *chip_select) {
	chip_select->write(0);
	uint8_t receivedDataH = (0x3F & spiAS5047p.write(0xFF)); //Get the first part (8bits)
	uint8_t receivedDataL = spiAS5047p.write(0xFF); //Get the second part (8bits)
	chip_select->write(1);
	return (uint16_t)((receivedDataH << 8) | (receivedDataL & 0xff)); //Combine the two parts to get a 16bits
}

void encoderUpdate(int encoder) {

	uint16_t countOld = enc_raw[encoder];
	uint16_t countNow = 0;

	if (encoder == ENC_LEFT)
		countNow = getEncodeurValue(&enc_left_cs);
	else
		countNow = getEncodeurValue(&enc_right_cs);

	int32_t countDelta = (int16_t)(countNow) - (int16_t)(countOld);

	enc_raw[encoder] = countNow;

	// Check if we did an overflow
	if (countDelta >= ((int32_t) ENC_REVOLUTION / 2)) {
		enc_revol[encoder] = enc_revol[encoder] - 1;
	} else if (countDelta <= -((int32_t) ENC_REVOLUTION / 2)) {
		enc_revol[encoder] = enc_revol[encoder] + 1;
	}

	enc_count[encoder] = -enc_offset[encoder] +
						 (enc_dir[encoder] * enc_revol[encoder] * ((int64_t) ENC_REVOLUTION)) +
						 (enc_dir[encoder] * (int64_t) countNow);

}

void asservUpdate() {

	// Convert current rate of the loop in seconds (float)
	auto f_secs = std::chrono::duration_cast < std::chrono::duration < float >> (ASSERV_UPDATE_RATE);
	dt_pid = f_secs.count();



	sixtron::PID_params pid_params;
	pid_params.Kp = 0.0000011f;
	pid_params.Ki = 0.00000003f;
	pid_params.Kd = 0.0f;
	pid_params.dt_seconds = dt_pid;
	pid_left = new sixtron::PID(pid_params);
	pid_right = new sixtron::PID(pid_params);

	pid_left->setLimit(sixtron::PID_limit::output_limit_HL, 1.0f);

	sixtron::PID_args args_motor_left, args_motor_right;

	args_motor_left.target = 200000.0f;

	// Update just in case
	filter_speed_left.setBiquad(bq_type_lowpass, 20.0f * dt_pid, 0.707f, 0.0f);
	encoderUpdate(ENC_LEFT);
	encoderUpdate(ENC_RIGHT);

	int64_t old_count_left = enc_count[ENC_LEFT];

	while (true) {
		// Wait for trig
		asservFlag.wait_any(ASSERV_FLAG);
		debug_pin = 1;
		// Update codeurs
		encoderUpdate(ENC_LEFT);
		encoderUpdate(ENC_RIGHT);

		// update speed
		motors_speed[ENC_LEFT] = filter_speed_left.process((float((enc_count[ENC_LEFT]) - old_count_left)) / dt_pid);
		old_count_left = enc_count[ENC_LEFT];

		// Update args and compute
		args_motor_left.actual = motors_speed[ENC_LEFT];
		pid_left->compute(&args_motor_left);
		motors_pwm[ENC_LEFT] = args_motor_left.output;

		// update motor PWM
		if (motors_pwm[ENC_LEFT] >= 0.0f) {
			motor_dir_left.write(0);
		} else {
			motors_pwm[ENC_LEFT] = -motors_pwm[ENC_LEFT];
			motor_dir_left.write(1);
		}

//		motor_left.write(motors_pwm[ENC_LEFT]);

		debug_pin = 0;
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
	debug_pin = 0;

	// Init encodeurs SPI
	enc_left_cs = 1;
	enc_right_cs = 1;
	spiAS5047p.format(8, 1);
	spiAS5047p.frequency(1000000);
	enc_dir[ENC_LEFT] = -1;
	enc_dir[ENC_RIGHT] = 1;
	encoderUpdate(ENC_LEFT);
	encoderUpdate(ENC_RIGHT);
	enc_offset[ENC_LEFT] = enc_count[ENC_LEFT];
	enc_offset[ENC_RIGHT] = enc_count[ENC_RIGHT];

	// Init motors
	motor_dir_left.write(0);
	motor_dir_right.write(0);
	motor_left.period_us(50); //20kHz
	motor_right.period_us(50); //20kHz
	motor_left.write(0.0f);
	motor_right.write(0.0f);

	// 0.55f buté haute
	// 0.105f


	// Setup servos
//	electroaimant.write(1);
//	servosTimerInit();
//	servoSetPwmDuty(SERVO0, MG996R_MIN);
//	ThisThread::sleep_for(2s);
//	servoSetPwmDuty(SERVO0, MG996R_MAX);


	// Setup Lidar Thread
	serialLidarThread.start(callback(&serialLidarEventQueue, &EventQueue::dispatch_forever));
	serialLidar.attach(&rxLidarCallback);

	// asserv update (LAST TO BE SETUP)
	asservThread.start(asservUpdate);
	asservTicker.attach(&asservSetFlag, ASSERV_UPDATE_RATE);

	// Main setup
	mainLoopTicker.attach(&mainLoopUpdate, MAIN_LOOP_RATE);

	int i = 0;
	char printf_buffer[100];
	while (true) {
		mainLoopFlag.wait_any(MAIN_LOOP_FLAG); // ... So instead we use a ticker to trigger a flag every second.

//		motor_dir_left != motor_dir_left;
//		printf("Pokirobot v1 alive since %ds (lidar hz = %f, lidar start = %f, lidar end = %f, distance = %d) ...\n",
//			   i++,
//			   lidar_hz,
//			   lidar_startAngle,
//			   lidar_endAngle,
//			   lidar_distanceArray[100]);



		// more speeeeeeed
//        sprintf(printf_buffer, "Pokirobot v1 alive since %ds (enc left = %lld) ...\n", i++,enc_count[ENC_LEFT]);
//        usb_serial.write(printf_buffer);

	}
}

