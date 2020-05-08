/*
	@brief This file contains all data specific to two sided balancing robot
 */

#ifndef DEVICE_H_
#define DEVICE_H_


// *** Libraries based defines *** //
#define NRF24L01P_MISTAKES_OFFSET 		100
#define ICM_20600_MISTAKES_OFFSET		200



#include "implementation.h"
#include "icm-20600_impi.h"
#include "nrf24l01p_impi.h"
#include "motors.h"

enum controller_states{
	up_side_down_balancing_state = (0U),
	lower_horizon_state = (1U),
	upper_horizon_state  = (2U),
	balancing_state = (3U),
};

/*
	@brief Runs all board diagnostics tests
 */
void device_self_diagnosticks(icm_20600 * icm_instance, nrf24l01p *nrf24_instance, motor * first_motor_instance, motor * second_motor_instance);

void imu_gyro_calibration(icm_20600 *icm_instance, int16_t calibration_coeficients[3]);

void balancing_init(icm_20600 *icm_instance);



#endif /* DEVICE_H_ */
