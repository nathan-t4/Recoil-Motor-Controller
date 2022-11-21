/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


void Motor_init(Motor *motor) {
  motor->pole_pairs = 7; // gripper: 12
  motor->kv_rating = 340; // gripper: 250

  motor->flux_angle_offset = 3.202; // motor 1: 5.626, motor 2: 3.202, motor 3: 3.466, motor 4: 3.534
}

