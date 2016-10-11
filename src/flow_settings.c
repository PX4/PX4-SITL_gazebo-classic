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

#include "flow_settings.h"
#include "mavlink/v1.0/common/mavlink.h"

enum global_param_id_t global_param_id;
struct global_struct global_data;


/**
 * @brief reset all parameters to default
 */
void global_data_reset_param_defaults(void){

	global_data.param[PARAM_SYSTEM_ID] = 81;
	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	global_data.param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	global_data.param[PARAM_COMPONENT_ID] = 50;
	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	global_data.param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	global_data.param[PARAM_SENSOR_ID] = 77;
	strcpy(global_data.param_name[PARAM_SENSOR_ID], "SYS_SENSOR_ID");
	global_data.param_access[PARAM_SENSOR_ID] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_TYPE] = MAV_TYPE_GENERIC;
	strcpy(global_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");
	global_data.param_access[PARAM_SYSTEM_TYPE] = READ_WRITE;

	global_data.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(global_data.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	global_data.param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

	global_data.param[PARAM_SW_VERSION] = 1300;
	strcpy(global_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	global_data.param_access[PARAM_SW_VERSION] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(global_data.param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	global_data.param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_SEND_LPOS] = 0;
	strcpy(global_data.param_name[PARAM_SYSTEM_SEND_LPOS], "SYS_SEND_LPOS");
	global_data.param_access[PARAM_SYSTEM_SEND_LPOS] = READ_WRITE;

	global_data.param[PARAM_SENSOR_POSITION] = 0; // BOTTOM
	strcpy(global_data.param_name[PARAM_SENSOR_POSITION], "POSITION");
	global_data.param_access[PARAM_SENSOR_POSITION] = READ_WRITE;

	global_data.param[PARAM_USART2_BAUD] = 115200;
	strcpy(global_data.param_name[PARAM_USART2_BAUD], "USART_2_BAUD");
	global_data.param_access[PARAM_USART2_BAUD] = READ_ONLY;

	global_data.param[PARAM_USART3_BAUD] = 115200;
//	global_data.param[PARAM_USART3_BAUD] = 921600;
	strcpy(global_data.param_name[PARAM_USART3_BAUD], "USART_3_BAUD");
	global_data.param_access[PARAM_USART3_BAUD] = READ_ONLY;

	global_data.param[PARAM_FOCAL_LENGTH_MM] = 16.0f;
	strcpy(global_data.param_name[PARAM_FOCAL_LENGTH_MM], "LENS_FOCAL_LEN");
	global_data.param_access[PARAM_FOCAL_LENGTH_MM] = READ_WRITE;

	global_data.param[PARAM_IMAGE_WIDTH] = BOTTOM_FLOW_IMAGE_WIDTH;
	strcpy(global_data.param_name[PARAM_IMAGE_WIDTH], "IMAGE_WIDTH");
	global_data.param_access[PARAM_IMAGE_WIDTH] = READ_ONLY;

	global_data.param[PARAM_IMAGE_HEIGHT] = BOTTOM_FLOW_IMAGE_HEIGHT;
	strcpy(global_data.param_name[PARAM_IMAGE_HEIGHT], "IMAGE_HEIGHT");
	global_data.param_access[PARAM_IMAGE_HEIGHT] = READ_ONLY;

	global_data.param[PARAM_IMAGE_LOW_LIGHT] = 0;
//	global_data.param[PARAM_IMAGE_LOW_LIGHT] = 1;
	strcpy(global_data.param_name[PARAM_IMAGE_LOW_LIGHT], "IMAGE_L_LIGHT");
	global_data.param_access[PARAM_IMAGE_LOW_LIGHT] = READ_WRITE;

	global_data.param[PARAM_IMAGE_ROW_NOISE_CORR] = 1;
	strcpy(global_data.param_name[PARAM_IMAGE_ROW_NOISE_CORR], "IMAGE_NOISE_C");
	global_data.param_access[PARAM_IMAGE_ROW_NOISE_CORR] = READ_WRITE;

	global_data.param[PARAM_IMAGE_TEST_PATTERN] = 0;
	strcpy(global_data.param_name[PARAM_IMAGE_TEST_PATTERN], "IMAGE_TEST_PAT");
	global_data.param_access[PARAM_IMAGE_TEST_PATTERN] = READ_WRITE;

	global_data.param[PARAM_GYRO_SENSITIVITY_DPS] = 250;
	strcpy(global_data.param_name[PARAM_GYRO_SENSITIVITY_DPS], "GYRO_SENS_DPS");
	global_data.param_access[PARAM_GYRO_SENSITIVITY_DPS] = READ_WRITE;

	global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD] = 0.01;
	strcpy(global_data.param_name[PARAM_GYRO_COMPENSATION_THRESHOLD], "GYRO_COMP_THR");
	global_data.param_access[PARAM_GYRO_COMPENSATION_THRESHOLD] = READ_WRITE;

	global_data.param[PARAM_SONAR_FILTERED] = 0;
	strcpy(global_data.param_name[PARAM_SONAR_FILTERED], "SONAR_FILTERED");
	global_data.param_access[PARAM_SONAR_FILTERED] = READ_WRITE;

	global_data.param[PARAM_SONAR_KALMAN_L1] = 0.8461f;
	strcpy(global_data.param_name[PARAM_SONAR_KALMAN_L1], "SONAR_KAL_L1");
	global_data.param_access[PARAM_SONAR_KALMAN_L1] = READ_WRITE;

	global_data.param[PARAM_SONAR_KALMAN_L2] = 6.2034f;
	strcpy(global_data.param_name[PARAM_SONAR_KALMAN_L2], "SONAR_KAL_L2");
	global_data.param_access[PARAM_SONAR_KALMAN_L2] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_VIDEO] = 1; // send video over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_VIDEO], "USB_SEND_VIDEO");
	global_data.param_access[PARAM_USB_SEND_VIDEO] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_FLOW] = 1; // send flow over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_FLOW], "USB_SEND_FLOW");
	global_data.param_access[PARAM_USB_SEND_FLOW] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_GYRO] = 1; // send gyro debug values over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_GYRO], "USB_SEND_GYRO");
	global_data.param_access[PARAM_USB_SEND_GYRO] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_FORWARD] = 0; // send forward flow over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_FORWARD], "USB_SEND_FWD");
	global_data.param_access[PARAM_USB_SEND_FORWARD] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_DEBUG] = 1; // send debug msgs over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_DEBUG], "USB_SEND_DEBUG");
	global_data.param_access[PARAM_USB_SEND_DEBUG] = READ_WRITE;

	global_data.param[PARAM_VIDEO_ONLY] = 0;
	strcpy(global_data.param_name[PARAM_VIDEO_ONLY], "VIDEO_ONLY");
	global_data.param_access[PARAM_VIDEO_ONLY] = READ_WRITE;

	global_data.param[PARAM_VIDEO_RATE] = 150;
	strcpy(global_data.param_name[PARAM_VIDEO_RATE], "VIDEO_RATE");
	global_data.param_access[PARAM_VIDEO_RATE] = READ_WRITE;

	global_data.param[PARAM_MAX_FLOW_PIXEL] = BOTTOM_FLOW_SEARCH_WINDOW_SIZE;
	strcpy(global_data.param_name[PARAM_MAX_FLOW_PIXEL], "BFLOW_MAX_PIX");
	global_data.param_access[PARAM_MAX_FLOW_PIXEL] = READ_ONLY;

//	global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD] = 8 * 8 * 20;
	global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD] = 5000; // threshold is irrelevant with this value
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD], "BFLOW_V_THLD");
	global_data.param_access[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD] = READ_WRITE;

//	global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD] = 100;
	global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD] = 30; // threshold is irrelevant with this value
	global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD] = 10; // lower it even more for SITL

	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD], "BFLOW_F_THLD");
	global_data.param_access[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD] = READ_WRITE;

	global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER] = 0;
//	global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER] = 1;
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_HIST_FILTER], "BFLOW_HIST_FIL");
	global_data.param_access[PARAM_BOTTOM_FLOW_HIST_FILTER] = READ_WRITE;

//	global_data.param[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION] = 0;
	global_data.param[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION] = 0;
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION], "BFLOW_GYRO_COM");
	global_data.param_access[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION] = READ_WRITE;

	global_data.param[PARAM_BOTTOM_FLOW_LP_FILTERED] = 0;
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_LP_FILTERED], "BFLOW_LP_FIL");
	global_data.param_access[PARAM_BOTTOM_FLOW_LP_FILTERED] = READ_WRITE;

	global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW] = 0.3f;
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_WEIGHT_NEW], "BFLOW_W_NEW");
	global_data.param_access[PARAM_BOTTOM_FLOW_WEIGHT_NEW] = READ_WRITE;

	global_data.param[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR] = 10.0f;
	strcpy(global_data.param_name[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR], "BFLOW_THROTT");
	global_data.param_access[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR] = READ_WRITE;

	global_data.param[DEBUG_VARIABLE] = 1;
	strcpy(global_data.param_name[DEBUG_VARIABLE], "DEBUG");
	global_data.param_access[DEBUG_VARIABLE] = READ_WRITE;

}

/**
 * @brief resets the global data struct to all-zero values
 */
void global_data_reset(void)
{
	// not in use anymore
}

/**
 * @brief changes read only settings depending on sensor position
 */
void set_sensor_position_settings(uint8_t sensor_position)
{

	switch(sensor_position)
	{
		case(BOTTOM):
			global_data.param[PARAM_IMAGE_WIDTH] = BOTTOM_FLOW_IMAGE_WIDTH;
			global_data.param[PARAM_IMAGE_HEIGHT] = BOTTOM_FLOW_IMAGE_HEIGHT;
			break;

		default:
			return;
	}

	return;
}
