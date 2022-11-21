/*
 * app.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */


#include "app.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;

MotorController controller;

float user_input_pot;
float pot_init_pos;
float motor_init_pos;
uint8_t user_input_button;
uint8_t user_output_led;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  CAN_Frame rx_frame;
  CAN_getRxFrame(&hfdcan1, &rx_frame);
  MotorController_handleCANMessage(&controller, &rx_frame);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    MotorController_updateCommutation(&controller, &hadc1);
  }
  else if (htim == &htim2) {
    if (controller.mode != MODE_IDLE && controller.mode != MODE_CALIBRATION) {
      MotorController_setMode(&controller, MODE_DISABLED);
      controller.error = ERROR_HEARTBEAT_TIMEOUT;
    }
  }
  else if (htim == &htim4) {
    MotorController_triggerPositionUpdate(&controller);
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  MotorController_updatePositionReading(&controller);

  /* ====== Start user APP code ====== */
//  controller.position_controller.position_target = APP_getUserPot() * M_PI;
//  if (controller.mode == MODE_POSITION) {
////	  controller.position_controller.velocity_target = 0;
//	  controller.position_controller.position_target = (APP_getUserPot() - pot_init_pos) * (5 * M_PI) + motor_init_pos;
//  }
//  else if (controller.mode == MODE_VELOCITY) {
//	  controller.position_controller.position_target = 0;
//	  controller.position_controller.velocity_target = APP_getUserPot() * 2 * M_PI;
//  }

  /* ====== End user APP code ====== */

  MotorController_updatePositionController(&controller);
}

uint8_t APP_getUserButton() {
  user_input_button = HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) ? 0 : 1;
  return user_input_button;
}

float APP_getUserPot() {
  user_input_pot = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * ADC_READING_COEFFICIENT / 3.3;
  return user_input_pot;
}

void APP_initFlashOption() {
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  FLASH->OPTR = 0xFBEFF8AA;  // default to boot from flash

  //SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);
  FLASH->CR |= FLASH_CR_OPTSTRT;

  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Lock();
  HAL_FLASH_OB_Launch();  // reload the new settings
}

void handleHostCommand() {
  char str[128];

  uint8_t command = 0;

  if (HAL_UART_Receive(&huart2, &command, 1, 1000) != HAL_OK) {
    return;
  }

  if (command == '0') {  // idle mode
    MotorController_setMode(&controller, MODE_IDLE);
    sprintf(str, "IDLE mode\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == '1') {  // position mode
    sprintf(str, "Start Calibration\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

    MotorController_setMode(&controller, MODE_CALIBRATION);
    sprintf(str, "Calibration Done!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == '2') {  // torque mode
    MotorController_setMode(&controller, MODE_TORQUE);
    sprintf(str, "TORQUE mode\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == '3') {  // velocity mode
    MotorController_setMode(&controller, MODE_VELOCITY);
    sprintf(str, "VELOCITY mode\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == '4') {  // position mode
    MotorController_setMode(&controller, MODE_POSITION);

    // update pot_init_pos to current position
    MotorController_triggerPositionUpdate(&controller);
    MotorController_updatePositionReading(&controller);
    motor_init_pos = controller.position_controller.position_measured;
    pot_init_pos = APP_getUserPot();

//    sprintf(str, "POSITION mode, init pos: %f,\t current pot: %f \r\n", motor_init_pos, pot_init_pos);
    sprintf(str, "POSITION mode\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == 'I') {  // log currents
    sprintf(str, "%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",
        controller.current_controller.i_a_measured,
		controller.current_controller.i_b_measured,
		controller.current_controller.i_c_measured,
        controller.current_controller.i_alpha_measured,
		controller.current_controller.i_beta_measured,
		controller.current_controller.i_q_measured,
		controller.current_controller.i_d_measured);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == 'V') {  // log voltages
    sprintf(str, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",
    	controller.current_controller.v_a_setpoint,
		controller.current_controller.v_b_setpoint,
		controller.current_controller.v_c_setpoint,
		controller.current_controller.v_alpha_setpoint,
		controller.current_controller.v_beta_setpoint,
		controller.current_controller.v_q_setpoint,
		controller.current_controller.v_d_setpoint,
		controller.powerstage.bus_voltage_measured);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == 'p') {  // log position
    sprintf(str, "%f\t%f\r\n",
    	controller.position_controller.position_measured,
		controller.position_controller.position_target
		);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == 'v') {  // log velocity
	sprintf(str, "%f\t%f\r\n",
		controller.position_controller.velocity_measured,
		controller.position_controller.velocity_target
		);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  }
  if (command == 'G') {  // log general
    sprintf(str, "%f\t%f\t%f\t%f\t%f\t%f\r\n",
        controller.position_controller.position_measured,
        controller.position_controller.position_setpoint,
		controller.position_controller.velocity_measured,
		controller.position_controller.velocity_setpoint,
		controller.current_controller.i_q_setpoint,
		controller.position_controller.torque_setpoint
        );
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    return;
  } if (command == 'c') { // config
	sprintf(str, "flux angle offset: %f\t device id: %d\r\n",
			controller.motor.flux_angle_offset,
			DEVICE_CAN_ID);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
  } if (command == 't') { // torques
	sprintf(str, "measured: %f\t setpoint: %f\r\n",
			controller.position_controller.torque_measured,
			controller.position_controller.torque_setpoint);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
  }
}


void APP_init() {
  MotorController_init(&controller);

  controller.position_controller.position_kp = 0.3;
  controller.position_controller.torque_limit_lower = -5;
  controller.position_controller.torque_limit_upper = 5;

  {
    char str[128];
    sprintf(str, "motor pp: %d\r\n", controller.motor.pole_pairs);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
    sprintf(str, "motor offset angle: %f\r\n", controller.motor.flux_angle_offset);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
	sprintf(str, "device id: %d\r\n", DEVICE_CAN_ID);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
  }
}


void APP_main() {
  MotorController_updateService(&controller);

  if (APP_getUserButton(&controller)) {
//    controller.current_controller.i_d_target = 0.;
//    controller.current_controller.i_q_target = 0.5;
//    MotorController_setMode(&controller, MODE_OPEN_IDQ);

    MotorController_setMode(&controller, MODE_CALIBRATION);

//    MotorController_setMode(&controller, MODE_POSITION);
//    MotorController_setMode(&controller, MODE_DISABLED);
  }

  handleHostCommand();

//  char str[128];
//  sprintf(str, "pos:%f\tpos_t:%f\tpos_s:%f\tv:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.position_controller.position_target,
//      controller.position_controller.position_setpoint,
//      user_input_pot);
//  sprintf(str, "pos:%f\tq:%f\tt:%f\tq_set:%f\tt_set:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.i_q_measured,
//      controller.position_controller.torque_measured,
//      controller.current_controller.i_q_setpoint,
//      controller.position_controller.torque_setpoint);
//  sprintf(str, "%d\r\n", val);
//  sprintf(str, "pos:%f\tiq:%f\tid:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.i_q_setpoint,
//      controller.current_controller.i_d_setpoint);
//  sprintf(str, "pos:%f\tvq:%f\tvd:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.v_q_setpoint,
//      controller.current_controller.v_d_setpoint);
//  sprintf(str, "pos:%f\tvalpha:%f\tvbeta:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.v_alpha_setpoint,
//      controller.current_controller.v_beta_setpoint);
//  sprintf(str, "pos:%f\tva:%f\tvb:%f\tvc:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.v_a_setpoint,
//      controller.current_controller.v_b_setpoint,
//      controller.current_controller.v_c_setpoint);
//  sprintf(str, "ia:%f\tib:%f\tic:%f\r\n",
//      controller.current_controller.i_a_measured,
//      controller.current_controller.i_b_measured,
//      controller.current_controller.i_c_measured);
//    sprintf(str, "vbus:%f\tvel:%f\r\n",
//        controller.powerstage.bus_voltage_measured,
//        controller.encoder.velocity);
//  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

  HAL_Delay(10);

}

