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

DigitalOut debug_pin(PB_5);

DigitalOut led_out_green(PC_8);
DigitalOut led_out_red(PB_4);

DigitalIn tirette(PC_9);

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
sixtron::PID *pid_motor_left, *pid_motor_right;
sixtron::PID *pid_dv, *pid_dteta;
static float motors_speed[2] = {0.0f};
static float motors_pwm[2] = {0.0f};
static float dt_pid = 0.0f;
Biquad filter_speed_left, filter_speed_right;
float robot_target_speed, robot_target_angle;
sixtron::PID_args args_motor_left, args_motor_right;
sixtron::PID_args args_dv, args_dteta;

// ENCODERS
#define ENC_LEFT 0
#define ENC_RIGHT 1
#define ENC_REVOLUTION 16384
#define MOTOR_REDUCTION    50
SPI spiAS5047p(PA_7, PA_6, PA_5); // mosi, miso, sclk
DigitalOut enc_left_cs(PA_8);
DigitalOut enc_right_cs(PB_2);
static uint16_t enc_raw[2] = {0};
static int32_t enc_revol[2] = {0};
static int64_t enc_count[2] = {0};
static int64_t enc_old_count[2] = {0};
static int64_t enc_dir[2] = {0};
static int64_t enc_offset[2] = {0};

// ELECTRO AIMANT
DigitalOut electroaimant(PC_4);


// MOTORS
PwmOut motor_left(PB_7);
PwmOut motor_right(PB_6);
DigitalOut motor_dir_left(PH_1);
DigitalOut motor_dir_right(PA_4);

// Main Debug
#define MAIN_LOOP_RATE     1000ms
#define MAIN_LOOP_FLAG     0x01
Ticker mainLoopTicker;
EventFlags mainLoopFlag;

// LIDAR
#define LIDAR_MODE_HEADER    0
#define LIDAR_MODE_MSG        1
#define LIDAR_MSG_LENGTH    32
#define CAMSENSE_X1_SPEED_L_INDEX 0
#define CAMSENSE_X1_SPEED_H_INDEX 1
#define CAMSENSE_X1_START_ANGLE_L_INDEX 2
#define CAMSENSE_X1_START_ANGLE_H_INDEX 3
#define CAMSENSE_X1_END_ANGLE_L_INDEX 28
#define CAMSENSE_X1_END_ANGLE_H_INDEX 29
#define CAMSENSE_X1_MAX_INDEX        400
#define CAMSENSE_X1_MAX_PAQUET        50
static uint8_t lidar_mode = 0;
static uint8_t lidar_header_incr = 0;
static uint8_t lidar_msg_incr = 0;
static uint8_t lidar_msg[LIDAR_MSG_LENGTH];
static float lidar_hz;
static float lidar_startAngle, lidar_endAngle;
static float lidar_offset = 16.0f; // 0 degrees seems to be 16 degrees of center.
static float lidar_IndexMultiplier = 400.0f / 360.0f;
static int lidar_index;
static uint16_t lidar_distanceArray[CAMSENSE_X1_MAX_INDEX];
static uint8_t lidar_qualityArray[CAMSENSE_X1_MAX_INDEX];
static uint16_t lidar_distanceArray_Median[CAMSENSE_X1_MAX_PAQUET];

#define LIDAR_FRONT_MIN    44
#define LIDAR_FRONT_MAX    8
#define LIDAR_BACK_MIN    20
#define LIDAR_BACK_MAX    32
#define LIDAR_TRIG_DETECT 300
static uint8_t lidar_back_trig = 0, lidar_front_trig = 0;

// ODOMETRY
#define TICK_PER_REVOL (ENC_REVOLUTION * MOTOR_REDUCTION)
#define TICK_PER_180 (TICK_PER_REVOL / 2.0f)
#define WHEEL_PERIMETER_MM    (2.0f * float(M_PI) * 35.0f)
#define TICK_PER_MM ((1.0f / (WHEEL_PERIMETER_MM)) * TICK_PER_REVOL)
#define MM_PER_TICK    (1.0f / TICK_PER_MM)
#define TICK2RAD(tick)  (((float(tick)) * float(M_PI)) / ((float)TICK_PER_180))
#define TICK2DEG(tick)  (((float(tick)) * 180.0f) / ((float)TICK_PER_180))
#define TICK2MM(tick)  (((float(tick))) * MM_PER_TICK)
#define MM2TICK(mm)  ((float(mm)) * TICK_PER_MM)
#define DEG2TICK(deg)  (((float(deg)) / 180.0f) * ((float)TICK_PER_180))

#define TICK_PER_ROBOT_REVOL    7340000
#define THETA_TICK2DEG(tick) (((float(tick)) * 180.0f) / (float(TICK_PER_ROBOT_REVOL/2.0f)))
#define THETA_DEG2TICK(deg) (((float(deg)) / 180.0f) * (float(TICK_PER_ROBOT_REVOL/2.0f)))

static float robot_x = 0.0f, robot_y = 0.0f;
static int64_t robot_distance = 0, robot_angle = 0, robot_angle_offset = 0;
static int64_t robot_linear_speed = 0, robot_angular_velocity = 0;


void process_serialLidarTX(char carac) {
//	printf("\t\t\t\tCharacter received from lidar : 0x%X\n", carac);
	if (lidar_mode == LIDAR_MODE_HEADER) {

		if ((lidar_header_incr == 0) && (carac == 0x55)) {
			lidar_header_incr++;
		} else if ((lidar_header_incr == 1) && (carac == 0xAA)) {
			lidar_header_incr++;
		} else if ((lidar_header_incr == 2) && (carac == 0x03)) {
			lidar_header_incr++;
		} else if ((lidar_header_incr == 3) && (carac == 0x08)) {
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

		if (lidar_msg_incr == LIDAR_MSG_LENGTH) {
//			printf("Lidar lidar_msg Received\n");
			lidar_mode = LIDAR_MODE_HEADER;
			lidar_msg_incr = 0;

			// Do calculus
			lidar_hz = float((uint16_t) (lidar_msg[CAMSENSE_X1_SPEED_H_INDEX] << 8) | lidar_msg[CAMSENSE_X1_SPEED_L_INDEX]) /
					   3840.0f; // 3840.0 = (64 * 60)
			lidar_startAngle =
					float(lidar_msg[CAMSENSE_X1_START_ANGLE_H_INDEX] << 8 | lidar_msg[CAMSENSE_X1_START_ANGLE_L_INDEX]) / 64.0f -
					640.0f;
			lidar_endAngle =
					float(lidar_msg[CAMSENSE_X1_END_ANGLE_H_INDEX] << 8 | lidar_msg[CAMSENSE_X1_END_ANGLE_L_INDEX]) / 64.0f -
					640.0f;

			//Get distance


			float step = 0.0;
			if (lidar_endAngle > lidar_startAngle) {
				step = (lidar_endAngle - lidar_startAngle) / 8;
			} else {
				step = (lidar_endAngle - (lidar_startAngle - 360)) / 8;
			}

			uint32_t sum = 0;
			uint8_t sum_num = 0;
			for (int i = 0; i < 8; i++) // for each of the 8 samples
			{
				float sampleAngle = (lidar_startAngle + step * i) + (lidar_offset + 180);
				float sampleIndexFloat = sampleAngle * lidar_IndexMultiplier; // map 0-360 to 0-400
				int sampleIndex = round(sampleIndexFloat); // round to closest value.
				lidar_index = sampleIndex % 400; // limit sampleIndex between 0 and 399 to prevent segmentation fault


				uint8_t distanceL = lidar_msg[4 + (i * 3)];
				uint8_t distanceH = lidar_msg[5 + (i * 3)];
				uint8_t quality = lidar_msg[6 + (i * 3)];


				if (quality == 0) // invalid data
				{
					lidar_distanceArray[lidar_index] = 0;
					lidar_qualityArray[lidar_index] = 0;
				} else {
					lidar_distanceArray[lidar_index] = ((uint16_t) distanceH << 8) | distanceL;
					lidar_qualityArray[lidar_index] = quality;
					sum += lidar_distanceArray[lidar_index];
					sum_num++;
				}

//				median += lidar_distanceArray[lidar_index];
//				printf("a = %f, d = %d\n", lidar_startAngle, lidar_distanceArray[lidar_index]);
			}

			lidar_distanceArray_Median[(lidar_index / 8)] = sum_num == 0 ? 0xFFFF : uint16_t(sum / sum_num);
//			printf("a = %d, d = %d\n", (lidar_index / 8), lidar_distanceArray_Median[(lidar_index / 8)]);
		}


	}
}

void updateLidarDetect() {

	// back
	for (int index = LIDAR_BACK_MIN; index < LIDAR_BACK_MAX; index++) {
//		printf("%d,%d\n", index, lidar_distanceArray_Median[index]);
		lidar_back_trig = 0;
		if (lidar_distanceArray_Median[index] < LIDAR_TRIG_DETECT) {
			lidar_back_trig = 1;
			break;
		}
	}

	// front
	for (int index = LIDAR_FRONT_MIN; index < (CAMSENSE_X1_MAX_PAQUET + LIDAR_FRONT_MAX); index++) {

//		printf("%d,%d\n", index, lidar_distanceArray_Median[index]);
		lidar_front_trig = 0;
		if (lidar_distanceArray_Median[index % CAMSENSE_X1_MAX_PAQUET] < LIDAR_TRIG_DETECT) {
			lidar_front_trig = 1;
			break;
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
	return (uint16_t) ((receivedDataH << 8) | (receivedDataL & 0xff)); //Combine the two parts to get a 16bits
}

void encoderUpdate(int encoder) {

	uint16_t countOld = enc_raw[encoder];
	uint16_t countNow = 0;

	if (encoder == ENC_LEFT)
		countNow = getEncodeurValue(&enc_left_cs);
	else
		countNow = getEncodeurValue(&enc_right_cs);

	int32_t countDelta = (int16_t) (countNow) - (int16_t) (countOld);

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

void motorUpdate(int enc, DigitalOut *dir, PwmOut *motor, sixtron::PID *pid_motor, Biquad *filter, sixtron::PID_args *args) {

	motors_speed[enc] = filter->process((float((enc_count[enc]) - enc_old_count[enc])) / dt_pid);
	enc_old_count[enc] = enc_count[enc];

	args->actual = motors_speed[enc];

	pid_motor->compute(args);

	motors_pwm[enc] = args->output;

	// update motor PWM
	if (motors_pwm[enc] >= 0.0f) {
		dir->write(0);
	} else {
		motors_pwm[enc] = -motors_pwm[enc];
		dir->write(1);
	}

	motor->write(motors_pwm[enc]);
}

void odometryUpdate() {

	//compute curvilinear distance
	int64_t new_distance = (enc_count[ENC_LEFT] + enc_count[ENC_RIGHT]) / 2;
	int64_t delta_distance = new_distance - robot_distance;

	//compute new angle value
	int64_t new_angle = (enc_count[ENC_RIGHT] - enc_count[ENC_LEFT]);
	int64_t delta_angle = new_angle - robot_angle;
//	float delta_anglef = float(delta_angle) / float(TICK_PER_ROBOT_REVOL) * float(M_PI) * 2.0f;

	//compute X/Y coordinates
	float mid_angle = THETA_TICK2DEG(robot_angle + delta_angle / 2.0f);
	float dx = float(delta_distance) * cosf(mid_angle/180.0f*float(M_PI));
	float dy = float(delta_distance) * sinf(mid_angle/180.0f*float(M_PI));
	robot_x += dx;
	robot_y += dy;

	//update global values
	robot_angle += delta_angle;
	robot_distance += delta_distance;
	robot_linear_speed = delta_distance;
	robot_angular_velocity = delta_angle;

}

void robotSpeedTargetSet(float speed_ms, float angle_ds, sixtron::PID_args * left, sixtron::PID_args *right){
	// Convert in [./s]
	float speed_tick_dt = (MM2TICK(speed_ms) * 1000.0f);
	float angle_tick_dt  = (THETA_DEG2TICK(angle_ds));

	// Asserv
//	args_dteta.actual = THETA_TICK2DEG(robot_angle);
//	args_dteta.target = angle_ds

	// Set both motors speed
	left->target = speed_tick_dt - (angle_tick_dt/2.0f);
	right->target = speed_tick_dt + (angle_tick_dt/2.0f);

}

void asservUpdate() {

	// Convert current rate of the loop in seconds (float)
	auto f_secs = std::chrono::duration_cast<std::chrono::duration<float >>(ASSERV_UPDATE_RATE);
	dt_pid = f_secs.count();


	sixtron::PID_params pid_motor_params;
	pid_motor_params.Kp = 0.0000011f;
	pid_motor_params.Ki = 0.000006f;
	pid_motor_params.Kd = 0.0f;
	pid_motor_params.dt_seconds = dt_pid;
	pid_motor_left = new sixtron::PID(pid_motor_params);
	pid_motor_right = new sixtron::PID(pid_motor_params);

	pid_motor_left->setLimit(sixtron::PID_limit::output_limit_HL, 1.0f);
	pid_motor_right->setLimit(sixtron::PID_limit::output_limit_HL, 1.0f);


	sixtron::PID_params pid_dv_params;
	pid_dv_params.Kp = 1.0f;
	pid_dv_params.Ki = 0.00000006f;
	pid_dv_params.Kd = 0.0f;
	pid_dv_params.dt_seconds = dt_pid;
	pid_dv = new sixtron::PID(pid_dv_params);

	sixtron::PID_params pid_dteta_params;
	pid_dteta_params.Kp = 1.0f;
	pid_dteta_params.Ki = 0.00000006f;
	pid_dteta_params.Kd = 0.0f;
	pid_dteta_params.dt_seconds = dt_pid;
	pid_dteta = new sixtron::PID(pid_dteta_params);


	// Update just in case
	filter_speed_left.setBiquad(bq_type_lowpass, 20.0f * dt_pid, 0.707f, 0.0f);
	filter_speed_right.setBiquad(bq_type_lowpass, 20.0f * dt_pid, 0.707f, 0.0f);
	encoderUpdate(ENC_LEFT);
	encoderUpdate(ENC_RIGHT);

	int64_t old_count[2];
	old_count[ENC_LEFT] = enc_count[ENC_LEFT];
	old_count[ENC_RIGHT] = enc_count[ENC_RIGHT];

	int32_t yolo = 0;

	uint8_t carre = 0;

	while (true) {
		// Wait for trig
		asservFlag.wait_any(ASSERV_FLAG);
		debug_pin = 1;

		// Update lidar detect
		updateLidarDetect();
		led_out_green = !!lidar_front_trig;
		led_out_red = !!lidar_back_trig;

		// Update codeurs
		encoderUpdate(ENC_LEFT);
		encoderUpdate(ENC_RIGHT);

		// Update odometry
		odometryUpdate();

		// target Update
//		robotSpeedTargetSet(0.0f, 45.0f, &args_motor_left, &args_motor_right);

		// Update target

//		switch (carre) {
//			case 0:
//				args_motor_left.target = 500000.0f;
//				args_motor_right.target = 500000.0f;
//				if (TICK2MM(robot_x) > 500.0f)
//					carre++;
//				break;
//			case 1:
//				args_motor_left.target = 300000.0f;
//				args_motor_right.target = -300000.0f;
//				if (THETA_TICK2DEG(robot_angle) < -90.0f)
//					carre++;
//				break;
//			case 2:
//				args_motor_left.target = 500000.0f;
//				args_motor_right.target = 500000.0f;
//				if (TICK2MM(robot_y) < -500.0f)
//					carre++;
//				break;
//			case 3:
//				args_motor_left.target = 300000.0f;
//				args_motor_right.target = -300000.0f;
//				if (THETA_TICK2DEG(robot_angle) < -180.0f)
//					carre++;
//				break;
//			case 4:
//				args_motor_left.target = 500000.0f;
//				args_motor_right.target = 500000.0f;
//				if (TICK2MM(robot_x) < 0.0f)
//					carre++;
//				break;
//			case 5:
//				args_motor_left.target = 300000.0f;
//				args_motor_right.target = -300000.0f;
//				if (THETA_TICK2DEG(robot_angle) < -270.0f)
//					carre++;
//				break;
//			case 6:
//				args_motor_left.target = 500000.0f;
//				args_motor_right.target = 500000.0f;
//				if (TICK2MM(robot_y) > 0.0f)
//					carre++;
//				break;
//			case 7:
//				args_motor_left.target = 300000.0f;
//				args_motor_right.target = -300000.0f;
//				if (THETA_TICK2DEG(robot_angle) < -360.0f)
//					carre++;
//				break;
//			case 8:
//				args_motor_left.target = 0.0f;
//				args_motor_right.target = 0.0f;
//		}


//		if (yolo < 5000) {
//		if (robot_angle > (-3*TICK_PER_ROBOT_REVOL)){


		// Update motors
		motorUpdate(ENC_LEFT, &motor_dir_left, &motor_left, pid_motor_left, &filter_speed_left, &args_motor_left);
		motorUpdate(ENC_RIGHT, &motor_dir_right, &motor_right, pid_motor_right, &filter_speed_right, &args_motor_right);

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
//	debug_pin = 0;

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

	// Setup servos
	servosTimerInit();
//	servoSetPwmDuty(SERVO0, MG996R_MIN);
//	ThisThread::sleep_for(2s);
//	servoSetPwmDuty(SERVO0, MG996R_MAX);

	motor_dir_left = 0;
	motor_dir_right = 0;
	motor_left = 0.0f;
	motor_right = 0.0f;

	ThisThread::sleep_for(2s);
//	while(tirette);

	// Setup Lidar Thread
	serialLidarThread.start(callback(&serialLidarEventQueue, &EventQueue::dispatch_forever));
	serialLidar.attach(&rxLidarCallback);

	ThisThread::sleep_for(2s);

	// asserv update (LAST TO BE SETUP)
	asservThread.start(asservUpdate);
	asservTicker.attach(&asservSetFlag, ASSERV_UPDATE_RATE);

//	ThisThread::sleep_for(2s);

	// Main setup
	mainLoopTicker.attach(&mainLoopUpdate, MAIN_LOOP_RATE);

	int i = 0;
	char printf_buffer[100];
	while (true) {
		mainLoopFlag.wait_any(MAIN_LOOP_FLAG); // ... So instead we use a ticker to trigger a flag every second.


		motor_dir_left != motor_dir_left;
		printf("Pokirobot v1 alive since %ds (X= %f, speed Y = %f, angle = %f, codeur = %lld) ...\n",
			   i++,
			   TICK2MM(robot_x),
			   TICK2MM(robot_y),
			   THETA_TICK2DEG(robot_angle),
			   enc_count[ENC_RIGHT]);

//		for(int a =0 ; a<CAMSENSE_X1_MAX_PAQUET ; a++){
//			printf("%d;%d\n", a, lidar_distanceArray_Median[a]);
//		}

//		for(int a =0 ; a<CAMSENSE_X1_MAX_INDEX; a++){
//			printf("%d;%d;%d\n", a, lidar_distanceArray[a], lidar_qualityArray[a]);
//		}



		// more speeeeeeed
//        sprintf(printf_buffer, "Pokirobot v1 alive since %ds (enc left = %lld) ...\n", i++,enc_count[ENC_LEFT]);
//        usb_serial.write(printf_buffer);

	}
}

