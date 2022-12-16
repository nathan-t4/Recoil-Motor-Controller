/*
 * position_controller.c
 *
 *  Created on: Aug 27, 2022
 *      Author: TK
 */

#include "position_controller.h"

void PositionController_init(PositionController *controller) {
  controller->position_kp = 4;
  controller->position_ki = 0;
  controller->position_kd = 0.05;

  controller->torque_limit_lower = -5;
  controller->torque_limit_upper = 5;

  controller->velocity_limit_lower = -100;
  controller->velocity_limit_upper = 100;

  controller->position_limit_lower = -2. * 3.14159 * 4.5; // 2*pi*GR
  controller->position_limit_upper = 2 * 3.14159 * 4.5;
}

void PositionController_update(PositionController *controller, Mode mode) {
	  /*
	   * TODO
	   * 	Change setpoint to error
	   * 	Support MODE_VELOCITY (only MODE_POSITION and MODE_TORQUE)
	   * 	Add position ki and velocity PID gains?
	   * 	clampf(iq) instead of clamp(torque_setpoint)
	   * 	how to set position / torque limits?
	   * 	add motor->gear_ratio
	   */

	  float position_setpoint = controller->position_target - controller->position_measured;
	  position_setpoint = clampf(
	      position_setpoint,
	      controller->position_limit_lower,
	      controller->position_limit_upper);

	  controller->position_setpoint = position_setpoint;

	  float velocity_setpoint = controller->velocity_target - controller->velocity_measured;

	  velocity_setpoint = clampf(
	      velocity_setpoint,
	      controller->velocity_limit_lower,
	      controller->velocity_limit_upper);
	  controller->velocity_setpoint = velocity_setpoint; 

	  if (mode != MODE_TORQUE) {
		controller->torque_setpoint = controller->position_kp * controller->position_setpoint
							  	  	+ controller->position_kd * controller->velocity_setpoint
								  	+ controller->torque_target;

	    controller->torque_setpoint = clampf(controller->torque_setpoint, controller->torque_limit_lower, controller->torque_limit_upper);
	  }
	  else {
	    controller->torque_setpoint = controller->torque_target;
	  }
}
