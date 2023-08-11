/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_MATH_PID_H_
#define ZEPHYR_INCLUDE_MATH_PID_H_

#include <stdint.h>

#if defined(CONFIG_FPU) || defined(CONFIG_FP_SOFTABI)

/**
 * @file
 * @brief Provide pid() function
 */

struct pid_state {
	float kp, ki, kd;

	float integral;
	float prev_error;
};

/**
 *
 * @brief Reset the internal pid state to initial values
 */
static inline void pid_reset_state(struct pid_state *state)
{
	state->integral = 0;
	state->prev_error = 0;
}

/**
 *
 * @brief Set the PID parameters into its state structure
 */
static inline void pid_set_parameters(struct pid_state *state, float kp,
				      float ki, float kd)
{
	state->kp = kp;
	state->ki = ki;
	state->kd = kd;
}

/**
 *
 * @brief Run the PID algorithm
 *
 * Calculate the PID value based on the state and current error.
 * To make the function agnostic to how frequently it's called the dt
 * argument is used to specify the amount of time from the last call.
 *
 * @warning This uses single-precision FPU. It's the callers responsibility
 *        to make sure FPU can be used in the context it was called from.
 *
 * @param state Internal PID state
 * @param error The difference between desired and actual target value
 * @param dt The amount of time that passed from the last time this function was
 * called. Can't be set to 0.
 *
 * @return PID control value
 */
static inline float pid(struct pid_state *state, float error, float dt)
{
	float output;

	state->integral += dt * error;

	output = error * state->kp;
	output += state->integral * state->ki;
	output += (error - state->prev_error) * state->kd / dt;

	state->prev_error = error;

	return output;
}

#else
#error PID can only be used with CONFIG_FP enabled
#endif /* CONFIG_FPU || CONFIG_FP_SOFTABI */
#endif /* ZEPHYR_INCLUDE_MATH_PID_H_ */
