/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


void Motor_init(Motor *motor) {
  motor->pole_pairs = 7;
  motor->kv_rating = 340;

  motor->flux_angle_offset = 2.314;
}

