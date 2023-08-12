/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include <math.h>

#include <unistd.h> // For usleep function (microsecond delay)

// Settings
#define PEDAL_INPUT_TIMEOUT 0.2
#define MAX_MS_WITHOUT_CADENCE 3000
#define MIN_MS_WITHOUT_POWER 500
#define FILTER_SAMPLES 5
#define RPM_FILTER_SAMPLES 8
#define MS_WITHOUT_CADENCE_COOLING_TIME 1000
// Define PID controller variables
#define MAX_MOTOR_RPM 15000

// Threads
static THD_FUNCTION(pas_thread, arg);
static THD_WORKING_AREA(pas_thread_wa, 1024);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
static volatile float max_pulse_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile float throttle_input = 0.0;
static volatile float brake_input = 0.0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// THROTTLE/BRAKE INTEGRATION
static volatile bool pas_use_adc = true;
static volatile bool pas_has_regen = false;

// TORQUE SENSOR
static volatile float torque_ratio = 0.0;
static volatile float min_start_torque = 0.5; // put this later in config
static volatile bool torque_on_adc1 = false;
static volatile bool adc_active = false;
static volatile float adc_throttle;
static volatile float ms_without_cadence = 0.0;
static volatile bool first_start_init = true;
static volatile float ms_without_cadence_cooling_time = 0.0;
static volatile bool min_start_torque_reached = false;

// SPEED LIMIT
static volatile float max_speed;
static float current_speed;
static float threshold_speed;

// TORQUE SENSOR
static volatile uint16_t sample_time = 0;
static volatile float drift_percent = 0;
static volatile float torque_percent = 0;
static volatile uint16_t uptime = 0;
static volatile uint16_t downtime = 0;
static volatile uint16_t print_trigger = 0;
static volatile uint16_t pas_data_trigger = 0;
static volatile bool torque_started = false;
static volatile uint8_t torque_smoothing_trigger = 0; // 2ms loop = 10* 10 = 100ms
static volatile float pas_hall_torque_offset;
static volatile float pas_hall_torque_gain;
static volatile uint8_t pas_hall_torque_samples;

// PID
static volatile float kp = 0.02;				   // Proportional gain
static volatile float ki = 0.01;				   // Integral gain
static volatile float kd = 0.02;				   // Derivative gain
static volatile float prev_error = 0.0;			   // Previous error
static volatile float error = MAX_MOTOR_RPM * 0.2; // proportional error ** 20% from max rpm is the start point
static volatile float error_ki = 0.0;			   // Integral error
static volatile float error_kd = 0.0;			   // Derivative error
static volatile float output_pid = 0.0;
static volatile float current_speed_goal = 0;
static volatile float pas_pid_start_percent;

// DEBUG
static volatile float drift_percent_check;
// static volatile uint16_t debug_2;

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf)
{
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_rpm_end * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);

	// get PID settings
	kp = config.pas_pid_kp;
	ki = config.pas_pid_ki;
	kd = config.pas_pid_kd;
	pas_pid_start_percent = config.pas_pid_start_percent;

	// get initial speed limit
	// max_speed = config.pas_max_speed;

	// Initialize adc rerouting
	pas_use_adc = config.pas_use_adc;

	// check if regen is needed
	pas_has_regen = config.pas_has_regen;

	// init torque sensor
	pas_hall_torque_offset = config.pas_hall_torque_offset;
	pas_hall_torque_gain = config.pas_hall_torque_gain;
	pas_hall_torque_samples = config.pas_hall_torque_samples;
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output)
{
	stop_now = false;
	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	primary_output = is_primary_output;
}

bool app_pas_is_running(void)
{
	return is_running;
}

bool has_torque_on_adc1(void)
{
	return torque_on_adc1;
}

void app_pas_stop(void)
{
	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}

	if (primary_output == true)
	{
		mc_interface_set_current_rel(0.0);
	}
	else
	{
		output_current_rel = 0.0;
	}
}

// APP PAS SET FUNCTIONS
void app_pas_set_current_sub_scaling(float current_sub_scaling)
{
	sub_scaling = current_sub_scaling;
}

void app_pas_set_assist_max_speed(float assist_max_speed)
{
	max_speed = assist_max_speed;
}

// APP PAS CALLING FUNCTIONS
float app_pas_get_current_target_rel(void)
{

	return output_current_rel;
}

float app_pas_get_pas_max_speed(void)
{
	return max_speed;
}

float app_pas_get_pedal_rpm(void)
{
	return pedal_rpm;
}

float app_pas_get_pedal_torque(void)
{
	return torque_percent;
}

float app_pas_get_kp(void)
{
	// return kp * error;
	return drift_percent_check;
}

float app_pas_get_ki(void)
{
	// return ki * error_ki;
	return drift_percent;
}

float app_pas_get_kd(void)
{
	return kd * error_kd;
}

float app_pas_get_adc_used(void)
{
	return pas_use_adc;
}

float app_pas_get_regen_status(void)
{
	return pas_has_regen;
}

void send_pas_data(void) // disabled for now maybe, use it later to send debug datas if needed.
{
	if (pas_data_trigger < 50)
	{
		pas_data_trigger++;
	}
	else
	{
		// commands_printf("print_trigger is 200\n");
		// commands_printf("int_pedal_rpm= %d\n", (int)pedal_rpm);
		static unsigned char pas_app_data[16];
		pas_app_data[0] = (unsigned char)(((int)pedal_rpm >> 8) & 0xFF);
		pas_app_data[1] = (unsigned char)((int)pedal_rpm & 0xFF);
		pas_app_data[2] = (unsigned char)(((int)torque_percent >> 8) & 0xFF);
		pas_app_data[3] = (unsigned char)((int)torque_percent & 0xFF);
		pas_app_data[4] = (unsigned char)(((int)0000 >> 8) & 0xFF);
		pas_app_data[5] = (unsigned char)((int)0000 & 0xFF);
		pas_app_data[6] = (unsigned char)(((int)0000 >> 8) & 0xFF);
		pas_app_data[7] = (unsigned char)((int)0000 & 0xFF);
		pas_app_data[8] = (unsigned char)(((int)(error * 100) >> 8) & 0xFF);
		pas_app_data[9] = (unsigned char)((int)(error * 100) & 0xFF);
		pas_app_data[10] = (unsigned char)(((int)(error_ki * 100) >> 8) & 0xFF);
		pas_app_data[11] = (unsigned char)((int)(error_ki * 100) & 0xFF);
		pas_app_data[12] = (unsigned char)(((int)((kd * (error - prev_error)) * 100) >> 8) & 0xFF);
		pas_app_data[13] = (unsigned char)((int)(((kd * (error - prev_error)) * 100) * 100) & 0xFF);
		pas_app_data[14] = (unsigned char)(((int)(output_pid * 100) >> 8) & 0xFF);
		pas_app_data[15] = (unsigned char)((int)(output_pid * 100) & 0xFF);

		commands_send_app_data(pas_app_data, sizeof(pas_app_data));

		pas_data_trigger = 0;
	}
}

// OUTPUT FUNCTIONS
float apply_pid_speed_limiting(float *input_value, float max_set_speed)
{

	current_speed = mc_interface_get_speed();
	// set the threshold speed for limiting speed limiting interference
	threshold_speed = max_set_speed * 0.8;

	// start PID control after knowing the max rpm

	kp = config.pas_pid_kp;
	ki = config.pas_pid_ki;
	kd = config.pas_pid_kd;
	// Calculate the error (difference between desired speed and current speed)
	error = ((max_set_speed - current_speed) * 100) / (max_set_speed * pas_pid_start_percent); // this should create an error at 100% by 80% of max speed
	error = fmin(fmax(error, -100.0), 100.0);

	// // PRINT DEBUG
	// static uint16_t delay_to_print = 0;
	// if (delay_to_print++ > 100)
	// {
	// 	delay_to_print = 0;
	// 	commands_printf("current_speed %.2f, error %.2f, error_ki %.2f, output_pid %.2f,max_speed_set %.2f, input_value %.2f , torque_ratio %.2f \n", (double) current_speed,  (double) error,  (double) error_ki, (double) output_pid,  (double)max_set_speed, (double) *input_value, (double) torque_ratio);
	// 	//commands_printf("pas_hall_torque_offset %.2f, pas_hall_torque_gain %.2f, pas_hall_torque_samples %d \n", (double)pas_hall_torque_offset, (double)pas_hall_torque_gain, (int)pas_hall_torque_samples);
	// }

	// guard from under threshold actions or too motivated ki
	if (error >= 100)
	{
		error_ki = 0.0;
		return *input_value;
	}
	else if (error <= -100)
	{
		error_ki = 0.0;
		*input_value *= 0.0; // overspeed of a lot we surely stop everything
		return *input_value;
	}

	// Update the integral error and guard for overvalues
	error_ki += ((error / config.update_rate_hz)); // 500Hz = 0.002s

	// Update the derivate error
	error_kd = (error - prev_error) / config.update_rate_hz;

	// Calculate the PID output
	output_pid = kp * error + ki * error_ki + kd * error_kd;
	output_pid = fmin(fmax(output_pid, 0.0), 1.0);

	// Update the previous error
	prev_error = error;

	// Apply the scaling factor to the output
	if (*input_value > output_pid && pedal_rpm > (config.pedal_rpm_start))
		*input_value *= output_pid;

	return *input_value;
}

float apply_ramping(float *input_value, float ramp_time_pos, float ramp_time_neg, uint16_t brakes_on)
{

	static systime_t last_time = 0;
	static float output_ramp = 0.0;

	// Apply smooth ramping at start.
	if (pedal_rpm > 0.01 && pedal_rpm < 40)
	{
		// Adjust the ramp times to be inversely proportional to the pedal RPM
		ramp_time_pos /= (pedal_rpm / 100);
		ramp_time_neg /= (pedal_rpm / 100);
	}

	float ramp_time = fabsf(*input_value) > fabsf(output_ramp) ? ramp_time_pos : ramp_time_neg;

	// slow things up when speed limiting and over speed limit threshold
	if (pedal_rpm > (config.pedal_rpm_start) && current_speed > threshold_speed && brakes_on == 0)
		ramp_time *= 4;
	// add here double ramp time when torque only is detected.
	if (ramp_time > 0.01)
	{
		const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);

		utils_step_towards(&output_ramp, *input_value, ramp_step);

		*input_value = utils_map(output_ramp, 0, 1.0, 0, 1.0);
		last_time = chVTGetSystemTimeX();
	}
	return *input_value;
}

// INPUT FUNCTIONS
float get_throttle_input(float *input_value)
{
	throttle_input = app_adc_get_decoded_level();
	*input_value += throttle_input;
	// guard for values over 1
	*input_value = fmin(fmax(*input_value, 0.0), 1.0);
	return *input_value;
}

float get_brakes_input(float *input_value)
{
	brake_input = ADC_VOLTS(ADC_IND_EXT3);
	*input_value = brake_input;
	return *input_value;
}

void pas_event_handler(void)
{
#ifdef HW_HAS_3_WIRES_PAS_SENSOR
	if (config.sensor_type == PAS_SENSOR_TYPE_STANDARD)
	{
		uint8_t new_state;
		static uint8_t pulse_count = 0;
		static uint8_t old_state = 0;
		static float old_timestamp = 0;
		static float inactivity_time = 0;
		static float period_filtered = 0;

		uint8_t PAS_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
		// Maybe here Set the second port for external sensor
		new_state = PAS_level;
		if (new_state != old_state)
			pulse_count++;

		old_state = new_state;

		const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

		// sensors are poorly placed, so use only one rising edge as reference
		if (pulse_count > 1)
		{
			float period = (timestamp - old_timestamp) * (float)config.magnets;
			old_timestamp = timestamp;

			UTILS_LP_FAST(period_filtered, period, 0.5);

			if (period_filtered < min_pedal_period)
			{ // can't be that short, abort
				return;
			}
			pedal_rpm = 60.0 / period_filtered;
			inactivity_time = 0.0;
			pulse_count = 0;
		}
		else
		{
			inactivity_time += 1.0 / (float)config.update_rate_hz;

			// if no pedal activity, set RPM as zero + taking in account the current RPM speed to give a more natural stop feeling
			if (inactivity_time > 1.0 / ((((config.pedal_rpm_start / 2) + pedal_rpm) / 60.0) * config.magnets) * 2)
			{
				pedal_rpm = 0.0;
			}
		}
	}
#endif

#ifdef HW_HAS_4_WIRES_PAS_SENSOR
	const int8_t QEM[] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix
	int8_t direction_qem;
	uint8_t new_state;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static uint8_t correct_direction_counter = 0;
	static uint8_t correct_direction_counter_last = 100;

	uint8_t PAS1_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t PAS2_level = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);

	// Hall sensor based torque sensor data aquisition
	if (pedal_rpm != 0)
	{
		// If HALL TORQUE SENSOR
		if (PAS1_level == 1 && PAS2_level == 1)
		{
			uptime++;
		}
		else if (uptime != 0 && pedal_rpm != 0)
		{
			downtime++;
		}
		sample_time = uptime + downtime;
		if (sample_time > (config.magnets * 2))
		{
			drift_percent_check = (uptime * 100) / sample_time;
			drift_percent = ((((uptime * 100) / sample_time) - config.pas_hall_torque_offset) * -1) * config.pas_hall_torque_gain; // 5 is a calibration factor to get percentage and 36 is the offset
			uptime = 0;
			downtime = 0;
		}

		// safety guards
		drift_percent = fmin(fmax(drift_percent, 0), 100);

		// reaching for smoother values
		torque_smoothing_trigger++;
		if (torque_smoothing_trigger > pas_hall_torque_samples)
		{

			if (drift_percent > torque_percent)
			{
				torque_percent += (drift_percent - torque_percent) / 2;
			}
			else if (drift_percent < torque_percent)
			{
				torque_percent -= (torque_percent - drift_percent) / 2;
			}
			torque_smoothing_trigger = 0;
			// safety guards
			torque_percent = fmin(fmax(torque_percent, 0), 100);
		}
	}
	else
	{
		torque_percent = 0.0;
	}
	new_state = PAS2_level * 2 + PAS1_level;
	direction_qem = (float)QEM[old_state * 4 + new_state];
	old_state = new_state;

	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	// Require several quadrature events in the right direction to prevent vibrations from
	// engging PAS
	int8_t direction = (direction_conf * direction_qem);

	switch (direction)
	{
	case 1:
		correct_direction_counter++;
		break;
	case -1:
		correct_direction_counter = 0;
		break;
	}

	// sensors are poorly placed, so use only one rising edge as reference
	if ((new_state == 3) && (correct_direction_counter >= 4) && correct_direction_counter_last != correct_direction_counter)
	{
		float period = (timestamp - old_timestamp) * (float)config.magnets;
		old_timestamp = timestamp;

		UTILS_LP_FAST(period_filtered, period, 1.0);

		if (period_filtered < min_pedal_period)
		{ // can't be that short, abort
			return;
		}
		if (correct_direction_counter >= 16)
		{
			correct_direction_counter = 4;
		}
		correct_direction_counter_last = correct_direction_counter;
		pedal_rpm = 60.0 / period_filtered;
		pedal_rpm *= (direction_conf * (float)direction_qem);
		inactivity_time = 0.0;
	}
	else
	{
		inactivity_time += 1.0 / (float)config.update_rate_hz;

		// if no pedal activity, set RPM as zero + taking in account the current RPM speed to give a more natural stop feeling
		if (inactivity_time > 1.0 / ((((config.pedal_rpm_start / 2) + pedal_rpm) / 60.0) * config.magnets) * 2)
		{
			pedal_rpm = 0.0;
		}
	}
#endif
}

// MAIN THREAD
static THD_FUNCTION(pas_thread, arg)
{
	(void)arg;

	float output = 0;
	float brakes = 0;
	chRegSetThreadName("APP_PAS");

#ifdef HW_HAS_PAS_SENSOR
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
#endif

#ifdef HW_HAS_4_WIRES_PAS_SENSOR
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
#endif

	is_running = true;

	for (;;)
	{
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0)
		{
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now)
		{
			is_running = false;
			return;
		}

		pas_event_handler(); // this could happen inside an ISR instead of being polled

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE)
		{
			ms_without_power = 0;
		}

		if (app_is_output_disabled())
		{
			continue;
		}

		switch (config.ctrl_type)
		{

		case PAS_CTRL_TYPE_NONE:
			output = 0.0;
			break;
		case PAS_CTRL_TYPE_BASIC:
			// BASIC PAS INTEGRATION (standard)
			if (first_start_init && pas_use_adc)
			{
				app_adc_detach_adc(1);
				first_start_init = false;
			}

			if (pedal_rpm > (config.pedal_rpm_start))
			{
				output = 1.0;
			}
			else
			{
				output = 0.0;
			}

			break;

		case PAS_CTRL_TYPE_HALL_TORQUE:
			// PAS_CTRL_TYPE_HALL_TORQUE
			if (first_start_init && pas_use_adc)
			{
				app_adc_detach_adc(1);
				first_start_init = false;
			}

			if (torque_percent > 20 && !torque_started) // 20 is a good value
			{
				torque_started = true;
			}
			if (pedal_rpm > (config.pedal_rpm_start) && torque_started)
			{

				output = (utils_throttle_curve((torque_percent / 100), -0.95, 0, 0)) + 0.001; // use exp curving to compensate bad TS and add a minimum 0.001 too
				output = fmin(fmax(output, 0.0), 1.0);
			}
			else
			{
				output = 0.0;
				torque_started = false;
			}

			break;

		case PAS_CTRL_TYPE_CADENCE:
			// Map pedal rpm to assist level
			// NOTE: If the limits are the same a numerical instability is approached, so in that case
			// just use on/off control (which is what setting the limits to the same value essentially means).
			if (first_start_init && pas_use_adc)
			{
				app_adc_detach_adc(1);
				first_start_init = false;
			}

			if (config.pedal_rpm_end > (config.pedal_rpm_start + 1.0))
			{
				output = utils_map(pedal_rpm, config.pedal_rpm_start, config.pedal_rpm_end, 0.0, config.current_scaling * sub_scaling);
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
			}
			else
			{
				if (pedal_rpm > config.pedal_rpm_end)
				{
					output = config.current_scaling * sub_scaling;
				}
				else
				{
					output = 0.0;
				}
			}
			break;

			break;
		case PAS_CTRL_TYPE_TORQUE:
			// Standard way a PAS/Torque sensor (analog) on an E-Bike works.
			// if PAS over min RPM. it starts and gives the max what assist level is * torque ratio (0->1) set at, else, it stops.
			// It uses throttle as Torque sensor Input!

			if (first_start_init && pas_use_adc)
			{
				app_adc_detach_adc(1);
				first_start_init = false;
			}
			//get torque
			torque_ratio = app_adc_get_decoded_level();

			if (pedal_rpm > (config.pedal_rpm_start + 1.0))
			{
				output = utils_map(pedal_rpm, config.pedal_rpm_start, config.pedal_rpm_end, 0.0, config.current_scaling * sub_scaling);
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);	
				output += torque_ratio;
				output = fmin(fmax(output, 0.0), 1.0);
				ms_without_cadence = 0.0;
				ms_without_cadence_cooling_time = 0.0;
				min_start_torque_reached = false;
			}
			// start on pedal press available (1s delay) for 3s, if no PAS signal are detected it cools down for 1 second.
			else
			{
				// if (torque_ratio > min_start_torque)
				// {
				// 	min_start_torque_reached = true;
				// }
				// if (min_start_torque_reached)
				// {
				// 	if (ms_without_cadence_cooling_time > MS_WITHOUT_CADENCE_COOLING_TIME)
				// 	{
				// 		ms_without_cadence += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
				// 		if (ms_without_cadence < MAX_MS_WITHOUT_CADENCE)
				// 		{
				// 			output = torque_ratio;
				// 		}
				// 		else
				// 		{
				// 			output = 0.0;
				// 			ms_without_cadence_cooling_time = 0.0;
				// 			min_start_torque_reached = false;
				// 		}
				// 	}
				// 	else
				// 	{
				// 		output = 0.0;
				// 		ms_without_cadence_cooling_time += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
				// 	}
				// }
				output= 0.0;
			}
				// // PRINT DEBUG
		static uint16_t delay_to_print = 0;
		if (delay_to_print++ > 100)
		{
			delay_to_print = 0;
			commands_printf("output %.2f, config.current_scaling %.2f, sub_scaling, torque_ratio %.2f \n", (double)output, (double) config.current_scaling, (double) sub_scaling, (double)torque_ratio);
			//commands_printf("pas_hall_torque_offset %.2f, pas_hall_torque_gain %.2f, pas_hall_torque_samples %d \n", (double)pas_hall_torque_offset, (double)pas_hall_torque_gain, (int)pas_hall_torque_samples);
		}
			break;

#ifdef HW_HAS_CAN_TORQUE_SENSOR
		// this had to be modified (CAN_TORQUE_SENSOR) because it's an Bafang M600 specific hardware definition (CANbus torque sensor) it may be included but in a other way.
		case PAS_CTRL_TYPE_CAN_TORQUE:
		{
			torque_ratio = hw_get_PAS_torque();
			output = torque_ratio;
		}
		/* fall through */
		case PAS_CTRL_TYPE_CAN_TORQUE_WITH_CADENCE_TIMEOUT:
		{
			// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
			// motor overtemps when the rider is just resting on the pedals
			static float ms_without_cadence_or_torque = 0.0;
			if (output == 0.0 || pedal_rpm > 0)
			{
				ms_without_cadence_or_torque = 0.0;
			}
			else
			{
				ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
				if (ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE)
				{
					output = 0.0;
				}
			}
		}
#endif
		default:
			break;
		}

		// APPLY PAS LIMITATION
		output = output * config.current_scaling * sub_scaling;

		// GET THROTTLE INPUT
		output = get_throttle_input(&output);

		// // PRINT DEBUG
		// static uint16_t delay_to_print = 0;
		// if (delay_to_print++ > 100)
		// {
		// 	delay_to_print = 0;
		// 	commands_printf("output %.2f, config.current_scaling %.2f, sub_scaling %.2f \n", (double)output, (double) config.current_scaling, (double) sub_scaling);
		// 	//commands_printf("pas_hall_torque_offset %.2f, pas_hall_torque_gain %.2f, pas_hall_torque_samples %d \n", (double)pas_hall_torque_offset, (double)pas_hall_torque_gain, (int)pas_hall_torque_samples);
		// }

		// APPLY SPEED LIMITING
		// max_speed = config.pas_max_speed;
		output = apply_pid_speed_limiting(&output, max_speed);

		// BRAKES
		brakes = get_brakes_input(&brakes);

		float ramp_time_pos = config.ramp_time_pos; // Config ramp time for positive ramping
		float ramp_time_neg = config.ramp_time_neg; // Config ramp time for negative ramping

		static uint16_t brakes_on = 0;
		static float brakes_delay_ticks = 0;
		if (config.pas_brake_voltage_inverted && !pas_has_regen)
		{
			if (brakes < config.pas_brake_voltage_trigger)
			{
				output = 0.0;
				brakes_on++;
				ramp_time_pos = config.ramp_time_brakes_pos; // Config ramp time for brake positive ramping
				ramp_time_neg = config.ramp_time_brakes_neg; // Config ramp time for brake negative ramping
			}
			else
			{
				if (brakes_on > 0)
				{

					brakes_delay_ticks += (1000 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if (brakes_delay_ticks < config.pas_brake_delay)
						output = 0.0;
					else if (brakes_delay_ticks < (4 * config.pas_brake_delay))
						ramp_time_pos = config.ramp_time_brakes_pos; // Config ramp time for brake positive ramping
					else
					{
						brakes_delay_ticks = 0;
						brakes_on = 0; // release of the brake ramping time
					}
				}
			}
		}
		else if (!config.pas_brake_voltage_inverted && !pas_has_regen)
		{
			if (brakes > config.pas_brake_voltage_trigger)
			{
				output = 0.0;
				brakes_on++;
				ramp_time_neg = config.ramp_time_brakes_neg; // Config ramp time for brake negative ramping
			}
			else
			{
				if (brakes_on > 0)
				{

					brakes_delay_ticks += (1000 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if (brakes_delay_ticks < config.pas_brake_delay)
						output = 0.0;
					else if (brakes_delay_ticks < (4 * config.pas_brake_delay))
						ramp_time_pos = config.ramp_time_brakes_pos; // Config ramp time for brake positive ramping
					else
					{
						brakes_delay_ticks = 0;
						brakes_on = 0;
					}
				}
			}
		}
		else
		{
			output = 0.0;
			ramp_time_pos = config.ramp_time_brakes_pos; // Config ramp time for brake positive ramping
			ramp_time_neg = config.ramp_time_brakes_neg; // Config ramp time for brake negative ramping
		}

		// // PRINT DEBUG
		// static uint16_t delay_to_print = 0;
		// if (delay_to_print++ > 100)
		// {
		// 	delay_to_print = 0;
		// 	commands_printf("output %.2f, brakes %.2f, config.pas_brake_voltage_trigger %.2f, brakes_on %d \n", (double)output, (double)brakes, (double)config.pas_brake_voltage_trigger, (int)brakes_on);
		// 	// commands_printf("pas_hall_torque_offset %.2f, pas_hall_torque_gain %.2f, pas_hall_torque_samples %d \n", (double)pas_hall_torque_offset, (double)pas_hall_torque_gain, (int)pas_hall_torque_samples);
		// }

		// APPLY RAMPING
		output = apply_ramping(&output, ramp_time_pos, ramp_time_neg, brakes_on);

		// GUARD FOR TORQUE ONLY 
		// if (output < 0.001)
		// {
		// 	ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		// }

		// // Safe start is enabled if the output has not been zero for long enough
		// if (ms_without_power < MIN_MS_WITHOUT_POWER)
		// {
		// 	static int pulses_without_power_before = 0;
		// 	if (ms_without_power == pulses_without_power_before)
		// 	{
		// 		ms_without_power = 0;
		// 	}
		// 	pulses_without_power_before = ms_without_power;
		// 	output_current_rel = 0.0;
		// 	continue;
		// }

		// Reset timeout
		timeout_reset();

		// FORCE SEND PAS CUSTOM DATA
		// send_pas_data();

		// // // DEBUG PRINT
		//  static uint16_t delay_to_print = 0;
		//   if (delay_to_print++ > 100)
		//   {
		//   	delay_to_print = 0;
		//   	commands_printf("brakes: %.2f, max_speed: %.2f,throttle: %.2f, pas_use_adc: %d, output: %.2f,  \n", (double)brakes, (double)max_speed, (double)throttle_input, (int)pas_use_adc, (double)output);
		//  	//commands_printf("pas_hall_torque_offset %.2f, pas_hall_torque_gain %.2f, pas_hall_torque_samples %d \n", (double)pas_hall_torque_offset, (double)pas_hall_torque_gain, (int)pas_hall_torque_samples);

		// }
		if (primary_output == true)
		{
			mc_interface_set_current_rel(output);
		}
		else
		{
			output_current_rel = output;
		}
	}
}
