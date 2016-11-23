/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>

#define ONBOARD_PARAM_NAME_LENGTH 		15
#define BOTTOM_FLOW_IMAGE_HEIGHT		64
#define BOTTOM_FLOW_IMAGE_WIDTH			64
//#define BOTTOM_FLOW_SEARCH_WINDOW_SIZE 	4
//increase window size since running slower in SITL
#define BOTTOM_FLOW_SEARCH_WINDOW_SIZE 	8

/******************************************************************
  * ALL TYPE DEFINITIONS
  */

/**
  * @brief  parameter access
  */
typedef enum
{
  READ_ONLY   = 0,
  READ_WRITE  = 1,
} ParameterAccess_TypeDef;

/**
  * @brief  sensor position enumeration
  */
typedef enum
{
  BOTTOM = 0x00, 	/*!< at bottom position */
  FRONT  = 0x01, 	/*!< at front position */
  TOP    = 0x02, 	/*!< at top position */
  BACK   = 0x03,  	/*!< at back position */
  RIGHT  = 0x04,  	/*!< at right position */
  LEFT   = 0x05  	/*!< at left position */
} SensorPosition_TypeDef;

/**
  * @brief  sensor position enumeration
  */
typedef enum
{
  NO_VIDEO   = 0,
  CAM_VIDEO  = 1,
  FLOW_VIDEO = 2,
} VideoStreamMode_TypeDef;

/**
  * @brief  gyro sensitivity enumeration
  */
typedef enum
{
  DPS250  = 250, 	/*!< 250 dps */
  DPS500  = 500, 	/*!< 500 dps */
  DPS2000 = 2000	/*!< 2000 dps */
} GyroSensitivity_TypeDef;

/******************************************************************
  * ALL SETTINGS VARIABLES
  */

//typedef struct
//{
//	/* nothing here until now */
//
//} SysState_TypeDef;

enum global_param_id_t
{
	PARAM_SYSTEM_ID = 0,
	PARAM_COMPONENT_ID,
	PARAM_SENSOR_ID,
	PARAM_SYSTEM_TYPE,
	PARAM_AUTOPILOT_TYPE,
	PARAM_SW_VERSION,
	PARAM_SYSTEM_SEND_STATE,
	PARAM_SYSTEM_SEND_LPOS,

	PARAM_USART2_BAUD,
	PARAM_USART3_BAUD,
	PARAM_FOCAL_LENGTH_MM,
	PARAM_IMAGE_WIDTH,
	PARAM_IMAGE_HEIGHT,
	PARAM_MAX_FLOW_PIXEL,
	PARAM_IMAGE_LOW_LIGHT,
	PARAM_IMAGE_ROW_NOISE_CORR,
	PARAM_IMAGE_TEST_PATTERN,
	PARAM_GYRO_SENSITIVITY_DPS,
	PARAM_GYRO_COMPENSATION_THRESHOLD,
	PARAM_SONAR_FILTERED,
	PARAM_SONAR_KALMAN_L1,
	PARAM_SONAR_KALMAN_L2,

	PARAM_USB_SEND_VIDEO,
	PARAM_USB_SEND_FLOW,
	PARAM_USB_SEND_GYRO,
	PARAM_USB_SEND_FORWARD,
	PARAM_USB_SEND_DEBUG,

	PARAM_VIDEO_ONLY,
	PARAM_VIDEO_RATE,

	PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD,
	PARAM_BOTTOM_FLOW_VALUE_THRESHOLD,
	PARAM_BOTTOM_FLOW_HIST_FILTER,
	PARAM_BOTTOM_FLOW_GYRO_COMPENSATION,
	PARAM_BOTTOM_FLOW_LP_FILTERED,
	PARAM_BOTTOM_FLOW_WEIGHT_NEW,
	PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR,

	PARAM_SENSOR_POSITION,
	DEBUG_VARIABLE,

	ONBOARD_PARAM_COUNT

};

struct global_struct
{
	//SysState_TypeDef system_state;
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
	ParameterAccess_TypeDef param_access[ONBOARD_PARAM_COUNT];

};

/* global declarations */
extern enum global_param_id_t global_param_id;
extern struct global_struct global_data;

/******************************************************************
  * ALL SETTINGS FUNCTIONS
  */

void global_data_reset_param_defaults(void);
void global_data_reset(void);
void set_sensor_position_settings(uint8_t sensor_position);

#endif /* SETTINGS_H_ */
