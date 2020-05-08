
#include "motors.h"


float motors_get_speed_by_incements(const motor *motor_instance, const float time_increment)
{
	// Check if pointer exist
	if(motor_instance->speed_controller == 0)
	{
		return 0.0f;
	}

	int16_t current_encoder_couter_value = motor_instance->get_encoder_counter_value();
	int16_t encoder_counter_increment =  motor_instance->get_encoder_counter_value() - motor_instance->speed_controller->previous_encoder_counter_value;

	if (encoder_counter_increment == 0)
	{
		motor_instance->speed_controller->current_speed = 0.0f;
		return 0.0f;
	}

	motor_instance->speed_controller->current_speed = (float)(encoder_counter_increment) / (motor_instance->encoder_constant * time_increment);
	motor_instance->speed_controller->previous_encoder_counter_value = current_encoder_couter_value;

	return motor_instance->speed_controller->current_speed;
}

float motors_speed_controller_handler(const motor *motor_instance, const float time_increment)
{
	// Check if pointer exist
	if(motor_instance->speed_controller == 0)
	{
		return 0.0f;
	}

	float current_speed_mistake = motor_instance->speed_controller->target_speed - motor_instance->speed_controller->current_speed;
	float speed_limitation = motor_instance->speed_controller->controller_output_limitation_value;

	// At that point contain previous control signal
	float control_signal = motor_instance->speed_controller->regulator_control_signal;

	if ( !( (control_signal > speed_limitation && current_speed_mistake >= 0)
				|| (control_signal < -speed_limitation && current_speed_mistake < 0) ) )
	{
		motor_instance->speed_controller->current_integral += (current_speed_mistake + motor_instance->speed_controller->previous_speed_mistake) / 2.0f * time_increment;
	}

	motor_instance->speed_controller->previous_speed_mistake = current_speed_mistake;

	control_signal = (motor_instance->speed_controller->kp * current_speed_mistake)
				+ (motor_instance->speed_controller->ki * motor_instance->speed_controller->current_integral);

	motor_instance->speed_controller->regulator_control_signal = control_signal;

	if ( control_signal > speed_limitation )
	{
		return speed_limitation;
	}
	else if ( control_signal < -speed_limitation )
	{
		return -speed_limitation;
	}
	else
	{
		return control_signal;
	}
}



uint32_t motors_rotation_deiraction_test (motor *motor_instance)
{

	if(motor_instance->speed_controller != 0)
	{
		motor_instance->speed_controller->target_speed = 0.0f;
	}
	// make sure that motor is not rotating right now
	motor_instance->motor_disable();

	// Give motor a moment to stop
	for(int i = 0; i < 1000000; ++i);

	int16_t current_encoder_counter_value = motor_instance->get_encoder_counter_value();

	// Start motor with 1/2 of power
	motor_instance->set_pwm_duty_cycle(motor_instance->max_duty_cycle_coefficient/2);
	motor_instance->motor_enable();

	// Give motor a moment to run
	for(int i = 0; i < 1000000; ++i);

	motor_instance->motor_disable();
	motor_instance->set_pwm_duty_cycle(0);

	int16_t new_current_encoder_counter_value = motor_instance->get_encoder_counter_value();

	if (new_current_encoder_counter_value - current_encoder_counter_value < 0)
	{
		return 1;
	}
	else if (new_current_encoder_counter_value == current_encoder_counter_value)
	{
		return 2;
	}

	return 0;
}


float motors_get_position(motor *motor_instance)
{
	if ( motor_instance->position_controller == 0 )
	{
		return 0.0f;
	}

	int16_t current_encoder_counter_value = motor_instance->get_encoder_counter_value();
	int16_t encoder_counter_value_change = current_encoder_counter_value - motor_instance->position_controller->previous_encoder_counter_value;

	motor_instance->position_controller->current_position += encoder_counter_value_change / (motor_instance->encoder_constant);

	motor_instance->position_controller->previous_encoder_counter_value = current_encoder_counter_value;

	return current_encoder_counter_value;
}





float motors_position_controller_handler(motor *motor_instance)
{
	if ( motor_instance->position_controller == 0 )
	{
		return 0.0f;
	}

	float current_position_mistake = motor_instance->position_controller->target_position - motor_instance->position_controller->current_position;

	if(current_position_mistake < motor_instance->position_controller->position_precision && current_position_mistake > -motor_instance->position_controller->position_precision)
	{
		motor_instance->position_controller->regulator_control_signal = 0.0f;
	}
	else
	{
		motor_instance->position_controller->regulator_control_signal = motor_instance->position_controller->kp	* current_position_mistake;
	}


	return motor_instance->position_controller->regulator_control_signal;
}






void motor_reset(motor *motor_instance)
{
//	motor_instance->motor_disable();
	if(motor_instance->speed_controller != 0)
	{
		motor_instance->speed_controller->target_speed = 0.0f;
		motor_instance->speed_controller->current_integral = 0.0f;
	}
	else
	{
		// if there is no speed controller for the motor
		motor_instance->set_pwm_duty_cycle(0);
	}
}
