/*
 * encoder.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <math.h>

#include "stm32g4xx_hal.h"


typedef struct {
  I2C_HandleTypeDef *hi2c;
  TIM_HandleTypeDef *htim;

  uint8_t i2c_buffer[2];

  int8_t direction;
  uint32_t cpr;
  float position_offset;      // in range (-inf, inf)
  float velocity_filter_alpha;
  float dt;

  int32_t n_rotations;
  float position_relative;    // in range [0, 2PI)
  float position_raw;         // in range (-inf, inf), without offset

  float position;             // in range (-inf, inf), with offset
  float velocity;
} Encoder;


void Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

float Encoder_getOffset(Encoder *encoder);

void Encoder_setOffset(Encoder *encoder, float offset);

void Encoder_triggerUpdate(Encoder *encoder);

void Encoder_update(Encoder *encoder);

float Encoder_getRelativePosition(Encoder *encoder);

float Encoder_getRawPosition(Encoder *encoder);

float Encoder_getPosition(Encoder *encoder);

float Encoder_getVelocity(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
