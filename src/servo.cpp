#include <servo.h>

TIM_HandleTypeDef htim2;

void servosTimerInit() {

	__HAL_RCC_TIM2_CLK_ENABLE();
	// Configuration du TIMER2 en tant que PWM OUT
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = SERVO_PWM_PRESCAL; // 180MHz/60 ==> 3 000 000
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = SERVO_PWM_PERIOD; // 3 000 000 / 60 000 => 50Hz
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		printf("Erreur d'initialisation du Timer 2.\n");
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		printf("Erreur d'initialisation de la PWM du Timer 2.\n");
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		printf("Erreur d'initialisation de la PWM du Timer 2 (master).\n");
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = SERVO_INIT; // alpha de la PWM
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		printf("Erreur d'initialisation du channel 1.\n");
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		printf("Erreur d'initialisation du channel 2.\n");
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		printf("Erreur d'initialisation du channel 3.\n");
	}

	// Configuration du pin pwm (channel 1 du timer 10 pour avoir PWM sur PB8)
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;

	/**TIM10 GPIO Configuration
	 PB8     ------> TIM2_CH1
	 PB9     ------> TIM2_CH2
	 PB10     ------> TIM2_CH3
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// On force le démarrage des modules (merci Google)
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void servoSetPwmDuty(int servo, int dutyCycle) {

	if (dutyCycle > SERVO_PWM_PERIOD)
		dutyCycle = 1200;
	if (dutyCycle < 0)
		dutyCycle = 0;

	// Checker le site ci dessous si la ligne suivante ne fonctionne pas :
	// https://electronics.stackexchange.com/questions/179546/getting-pwm-to-work-on-stm32f4-using-sts-hal-libraries

	if(servo == SERVO0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dutyCycle);
	} else if (servo == SERVO1){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dutyCycle);
	} else if (servo == SERVO2){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyCycle);
	}
}