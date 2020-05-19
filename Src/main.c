// $branch$


// For global variables declaration
#define VAR_DECLS

#include "implementation.h"
#include "device.h"


// ***************3******************* //
// ****** Motor 1 declarations ****** //
// ********************************** //
motor motor1 =
			{
					.encoder_constant = 1540.0f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

speed_control motor1_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor1_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};

// ********************************** //
// ****** Motor 2 declarations ****** //
// ********************************** //
motor motor2 =
			{
					.encoder_constant = 1540.0f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

speed_control motor2_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor2_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};


// *********************************** //
// ****** NRF24L01+ declaration ****** //
// *********************************** //
nrf24l01p robot_nrf24 = {.device_was_initialized = 0};

uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x15};	// green LEDs car
//uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x25};	// blue LEDs car
uint16_t nrf_input_data[5] = {0, 0, 0, 0, 0};

uint32_t nrf24_data_has_been_captured = 0;
uint32_t nrf24_safety_counter = 0;

int32_t current_duty_cycle = 0;

uint16_t adc_current_value [2] = {0,0};
float current_input_voltage = 0.0f;
float current_divider_voltage = 0.0f;

int main(void)
{
	// ****** Motor 1 initialization ****** //
	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor1_pwm;
	motor1.get_encoder_counter_value = get_motor1_encoder_value;
	motor1.speed_controller = &motor1_speed_cotroller;
	motor1.position_controller = &motor1_position_controller;
	motor1_speed_cotroller.kp = 200.0f;
	motor1_speed_cotroller.ki = 5000.0f;
	motor1_position_controller.kp = 1.0f;
	motor1_position_controller.position_precision = 8.0f/motor1.encoder_constant;

	// ****** Motor 2 initialization ****** //
	motor2.motor_disable = gpioc6_low;
	motor2.motor_enable = gpioc6_high;
	motor2.set_pwm_duty_cycle = set_motor2_pwm;
	motor2.get_encoder_counter_value = get_motor2_encoder_value;
	motor2.speed_controller = &motor2_speed_cotroller;
	motor2.position_controller = &motor2_position_controller;
	motor2_speed_cotroller.kp = 200.0f;
	motor2_speed_cotroller.ki = 5000.0f;
	motor2_position_controller.kp = 1.0f;
	motor2_position_controller.position_precision = motor1_position_controller.position_precision;

	// ****** NRF24L01+ initialization ****** //
	robot_nrf24.ce_high = gpiob0_high;
	robot_nrf24.ce_low = gpiob0_low;
	robot_nrf24.csn_high = gpiob1_high;
	robot_nrf24.csn_low = gpiob1_low;
	robot_nrf24.spi_write_byte = spi1_write_single_byte;
	robot_nrf24.frequency_channel = 45;
	robot_nrf24.payload_size_in_bytes = 10;
	robot_nrf24.power_output = nrf24_pa_high;
	robot_nrf24.data_rate = nrf24_1_mbps;

	// Init MCU peripherals
	full_device_setup(yes, yes);

	// Enables both motors
	motor1.motor_enable();

//	delay_in_milliseconds(100);

	GPIOD->ODR |= 0x02;

	// NRF24L01+ device setup
	add_to_mistakes_log(nrf24_basic_init(&robot_nrf24));
	add_to_mistakes_log(nrf24_enable_pipe1(&robot_nrf24, nrf24_rx_address));
	add_to_mistakes_log(nrf24_enable_interrupts(&robot_nrf24, yes, no, no));
	add_to_mistakes_log(nrf24_rx_mode(&robot_nrf24));

	// led
	motor1.set_pwm_duty_cycle(-1 * PWM_PRECISION);

	// *** ADC setup for voltage measurement testing *** //

	// ADC and DMA clocking enable
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Enable voltage regulator
	ADC1->CR |= ADC_CR_ADVREGEN;
	delay_in_milliseconds(2);

	// ADC calibration
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL){} // Wait until calibration is complite

	// Enable ADC
	ADC1->ISR |= ADC_ISR_ADRDY; // clear ready flag
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY)){} // Wait until adc is ready

	// Sampling time setup
	ADC1->SMPR |= 2 << ADC_SMPR_SMP1_Pos; // All channels will use sampling time of smp1 which is 12.5 ADC clock cycles

	// Sampling sequence setup
	ADC1->CFGR1 |= ADC_CFGR1_CHSELRMOD;		// Enable sequencer ??
	ADC1->CHSELR |= 0xF << ADC_CHSELR_SQ2_Pos | 0 << ADC_CHSELR_SQ1_Pos; // only ADC1_IN0 is scuned
	while(!(ADC1->ISR & ADC_ISR_CCRDY)){}
	ADC1->CFGR1 |= ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN; // enable DMA

	// DMA channel 1 setup
	DMA1_Channel1->CPAR = (uint16_t *)&(ADC1->DR);	// Direct read from TIM15->CNT regester
	DMA1_Channel1->CMAR = (uint16_t *)adc_current_value;	// Memory address to write to. Уже указатель, поэтому нет необходимости получать его адрес

	// for 3 conversions
	DMA1_Channel1->CNDTR = 1;			// Number of transfers
	DMA1_Channel1->CCR |= 1 << DMA_CCR_MSIZE_Pos | 1 << DMA_CCR_PSIZE_Pos | DMA_CCR_MINC | DMA_CCR_CIRC;		// 16 bit in and out, circular mode, increment in memmory
	DMAMUX1_Channel0->CCR |= (5 << DMAMUX_CxCR_DMAREQ_ID_Pos); 	// ADC is 5
	DMA1_Channel1->CCR |= DMA_CCR_EN;		// enable DMA


	// *** End of ADC and DMA setup *** //

	while(1)
	{

		delay_in_milliseconds(200);
		GPIOD->ODR ^= 0x06;
		ADC1->CR |= ADC_CR_ADSTART;
		while(!(ADC1->ISR & ADC_ISR_EOS)){} // wait until sequence is complete
		ADC1->ISR |= ADC_ISR_EOS;

		current_divider_voltage = (float)(adc_current_value[0])/4096.0f * 3.3f;
		current_input_voltage = current_divider_voltage * 3.818f;

		// Testing ADC

//		// *** Testing of RGB strip control *** //
//
//		uint32_t counter_value = PWM_PRECISION / 10;
//
//		for (uint32_t i = 0; i < counter_value; ++i)
//		{
//			motor2.set_pwm_duty_cycle (i*10);
//
//			delay_in_milliseconds(10);
//		}
//
//		for (uint32_t i = 0; i < counter_value; ++i)
//		{
//			motor2.set_pwm_duty_cycle (PWM_PRECISION  - i*10);
//			delay_in_milliseconds(10);
//		}
//
//
//		for (uint32_t i = 0; i < counter_value; ++i)
//		{
//			motor2.set_pwm_duty_cycle (-1 * (float)(i*10));
//
//			delay_in_milliseconds(10);
//		}
//
//		for (uint32_t i = 0; i < counter_value; ++i)
//		{
//			motor2.set_pwm_duty_cycle (-1 * (float)(PWM_PRECISION) + (float)(i*10));
//			delay_in_milliseconds(10);
//		}



//		// *** Testing of position control for differential drive robot *** //
//
//		// For my position control system to properly work i need to use increments and not absolute values.
//		motor1.position_controller->target_position += 5.0f;
//		motor2.position_controller->target_position += 5.0f;
//		delay_in_milliseconds(1000);
//
//
//		motor1.position_controller->target_position += 3.0f;
//		motor2.position_controller->target_position -= 3.0f;
//		delay_in_milliseconds(1000);
//
//		motor1.position_controller->target_position -= 5.0f;
//		motor2.position_controller->target_position -= 5.0f;
//		delay_in_milliseconds(1000);
//
//		motor1.position_controller->target_position -= 3.0f;
//		motor2.position_controller->target_position += 3.0f;
//		delay_in_milliseconds(1000);
//		// ****************************** //

//		delay_in_milliseconds(500);
//		GPIOD->ODR ^= 0x01;
	}
}


// ****** Control loop handler ****** //
// Control loop handles by the SysTick timer

// *** Speed controller setup variables ***//
uint32_t speed_loop_call_counter = 0;

#define SPEED_LOOP_FREQUENCY				20	// Times per second. Must be not bigger then SYSTICK_FREQUENCY.
#define SPEED_LOOP_COUNTER_MAX_VALUE 		SYSTICK_FREQUENCY / SPEED_LOOP_FREQUENCY	// Times.
#define SPEED_LOOP_PERIOD					(float)(SPEED_LOOP_FREQUENCY) / (float)(SYSTICK_FREQUENCY) // Seconds.

// *** Position controller setup variables *** //
uint32_t position_loop_call_counter = 0;

#define POSITION_LOOP_FREQUENCY				10	// Times per second. Must be not bigger than SYSTICK_FREQUENCY. It is better if it is at least 2 times slower than the speed loop.
#define POSITION_LOOP_COUNTER_MAX_VALUE 	SYSTICK_FREQUENCY / POSITION_LOOP_FREQUENCY	// Times
//#define POSITION_LOOP_PERIOD				(float)(POSITION_LOOP_FREQUENCY) / (float)(SYSTICK_FREQUENCY) // Seconds

void SysTick_Handler()
{

	// *** Speed control handling *** //
	speed_loop_call_counter += 1;
	if ( speed_loop_call_counter == SPEED_LOOP_COUNTER_MAX_VALUE )	// 20 times per second
	{
		speed_loop_call_counter = 0;
		motors_get_speed_by_incements(&motor1, SPEED_LOOP_PERIOD);
		motors_get_speed_by_incements(&motor2, SPEED_LOOP_PERIOD);
//		float m1_speed_task = motors_speed_controller_handler(&motor1, SPEED_LOOP_PERIOD);
//		float m2_speed_task = motors_speed_controller_handler(&motor2, SPEED_LOOP_PERIOD);
//		motor1.set_pwm_duty_cycle((int32_t)m1_speed_task);
//		motor2.set_pwm_duty_cycle((int32_t)m2_speed_task);
	}

//	// *** Position control handling *** //
//	position_loop_call_counter += 1;
//	if ( position_loop_call_counter == POSITION_LOOP_COUNTER_MAX_VALUE )	// 10 times per second
//	{
//		position_loop_call_counter = 0;
//		motors_get_position(&motor1);
//		motors_get_position(&motor2);
//		motor1.speed_controller->target_speed = motors_position_controller_handler(&motor1);
//		motor2.speed_controller->target_speed = motors_position_controller_handler(&motor2);
//	}


	// *** Nrf24l01+ safety clock *** //
	if(nrf24_data_has_been_captured == 1)
	{
		nrf24_data_has_been_captured = 0;
		nrf24_safety_counter = 0;
	}
	else
	{
		nrf24_safety_counter += 1;
		if(nrf24_safety_counter == SYSTICK_FREQUENCY) // Exactly one second delay
		{
			// Stop and show that data is not capturing any more
			GPIOD->ODR &= ~0x01;
			motor1.speed_controller->target_speed = 0.0f;
			motor2.speed_controller->target_speed = 0.0f;
		}
	}

}
// **************************************** //

// ****** NRf24l01+ IRQ handler ****** //
void EXTI2_3_IRQHandler()
{
	// Clear interrupt flag
	EXTI->FPR1 |= 0x04;

	// Get new data
	add_to_mistakes_log(nrf24_read_message(&robot_nrf24, nrf_input_data, 10));
	GPIOD->ODR |= 0x01;

	nrf24_data_has_been_captured = 1;
	float left_motor_speed_task = 0.0f;
	float left_motor_boost = 0.0f;
	float right_motor_speed_task = 0.0f;
	float right_motor_boost = 0.0f;

	// Check for buttons press
	if((nrf_input_data[4] & 0x14) == 0x14){ // means, that top right button is unpressed
		// Do nothing
	}
	else if((nrf_input_data[4] & 0x04) == 0){
		left_motor_boost = 0.3f;
		right_motor_boost = 0.3f;
	}
	else{
		left_motor_boost = 0.6f;
		right_motor_boost = 0.6f;
	}

	// Evaluate the speed tasks
	if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000) // Forward
	{
		left_motor_speed_task = 1.0f + left_motor_boost;
		right_motor_speed_task = 1.0f + right_motor_boost;
	}
	else if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 1000)	// Forward left
	{
		left_motor_speed_task = 0.2f + left_motor_boost;
		right_motor_speed_task = 1.0f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] < 1000)	// Turn left
	{
		left_motor_speed_task = -0.6f - left_motor_boost;
		right_motor_speed_task = 0.6f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] < 1000)	// Backward left
	{
		left_motor_speed_task = -0.2f - left_motor_boost;
		right_motor_speed_task = -1.0f - right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000  && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000)	// Backward
	{
		left_motor_speed_task = -1.0f - left_motor_boost;
		right_motor_speed_task = -1.0f - right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] > 3000)	// Backward right
	{
		left_motor_speed_task = -1.0f - left_motor_boost;
		right_motor_speed_task = -0.2f - right_motor_boost;
	}
	else if(nrf_input_data[2] < 1000 && nrf_input_data[1] > 3000)	// Forward right
	{
		left_motor_speed_task = 1.0f + left_motor_boost;
		right_motor_speed_task = 0.2f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] > 3000)	// Turn right
	{
		left_motor_speed_task = 0.6f + left_motor_boost;
		right_motor_speed_task = -0.6f - right_motor_boost;
	}

	// Set new speed tasks
	motor1.speed_controller->target_speed = right_motor_speed_task;
	motor2.speed_controller->target_speed = left_motor_speed_task;
}
// **************************************** //

// EOF

