#ifndef IMU_H_
#define IMU_H_

#include "math.h"


/*
	@brief Calculates angle based on the accelerations and rotation speed.
 */
void basic_filtering(float acceleration_1, float acceleration_2, float axis_rotation_speed);


#endif /* IMU_H_ */
