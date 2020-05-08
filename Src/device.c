#include "device.h"


void device_self_diagnosticks(icm_20600 * icm_instance, nrf24l01p *nrf24_instance, motor * first_motor_instance, motor * second_motor_instance)
{

	GPIOD->ODR |= 0x08;

	// Icm-20600 diagnostics
	add_to_mistakes_log(icm_20600_check_if_alive(icm_instance));

	// NRF24 diagnostics
	add_to_mistakes_log(nrf24_check_if_alive(nrf24_instance));

	// Motor1 rotation check
	uint32_t motors_rotation_test_resalt = motors_rotation_deiraction_test(first_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M1_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M1_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}


	delay_in_milliseconds(500);

	// Motor 2 rotation check
	motors_rotation_test_resalt = motors_rotation_deiraction_test(second_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M2_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M2_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}
}


/*
	@brief Gets 100 gyroscope measurements and counts average
 */

void imu_gyro_calibration(icm_20600 *icm_instance, int16_t calibration_coeficients[3])
{
	// Delete previous calibration results to get proper values
	icm_instance->gyro_calibration_coefficients[icm_x] = 0;
	icm_instance->gyro_calibration_coefficients[icm_y] = 0;
	icm_instance->gyro_calibration_coefficients[icm_z] = 0;

	int16_t raw_imu_data[7] = {0, 0, 0, 0, 0, 0, 0};

	int32_t gyroscope_raw_sums[3] = {0, 0, 0};

	for (uint32_t i = 0; i < 256; i++)
	{
		delay_in_milliseconds(25); // 40 times per second
		icm_20600_get_raw_data(icm_instance, raw_imu_data);
		gyroscope_raw_sums[icm_x] += raw_imu_data[icm_gyroscope_x];
		gyroscope_raw_sums[icm_y] += raw_imu_data[icm_gyroscope_y];
		gyroscope_raw_sums[icm_z] += raw_imu_data[icm_gyroscope_z];
	}

	calibration_coeficients[icm_x] = (int16_t)(gyroscope_raw_sums[icm_x] / 256);
	calibration_coeficients[icm_y] = (int16_t)(gyroscope_raw_sums[icm_y] / 256);
	calibration_coeficients[icm_z] = (int16_t)(gyroscope_raw_sums[icm_z] / 256);
}


// !!!!!!!!!!!!!!!!!!!!!!!!!!  //
// Danger zone
// !!!!!!!!!!!!!!!!!!!!!!!!!!! //


