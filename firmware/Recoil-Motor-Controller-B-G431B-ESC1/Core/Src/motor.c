/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


void Motor_init(Motor *motor) {
  motor->pole_pairs = 12; // gripper: 12, tail: 7
  motor->kv_rating = 250; // gripper: 250, tail: 340

  motor->flux_angle_offset = 3.509751; // motor 1: 5.626, motor 2: 3.670817, motor 3: 2.186690, motor 4: 3.509751
}

