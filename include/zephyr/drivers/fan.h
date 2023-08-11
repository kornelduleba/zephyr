/*
 * Copyright 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public fan Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FAN_H_
#define ZEPHYR_INCLUDE_DRIVERS_FAN_H_

/**
 * @brief Fan Interface
 * @defgroup fan_interface fan Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/toolchain.h>

#include <zephyr/dt-bindings/fan/fan.h>
#include <zephyr/dt-bindings/pwm/pwm.h>

enum fan_mode {
	FAN_MODE_PWM = DT_FAN_MODE_PWM,
	FAN_MODE_RPM = DT_FAN_MODE_RPM,
	FAN_MODE_MAX
};

enum fan_attribute {
	FAN_ATTR_PID,
	FAN_ATTR_PWM_LIMIT,
	FAN_ATTR_MAX
};

enum fan_speed_type {
	FAN_SPEED_TYPE_PWM,
	FAN_SPEED_TYPE_RPM,
	FAN_SPEED_TYPE_TARGET,
	FAN_SPEED_TYPE_MAX
};

/*
 * The PID parameters can be set at initalization, through DT, or by using the
 * set_attr_t API. They're passed as integers, since that's what DT understands.
 * To represent parameters with decimal places the integers are divided by 10^6
 * after being converted to a floats.
 */
#define PID_PARAM_DIVIDER 10e6f

struct fan_attr_data_pid {
	uint32_t kp;
	uint32_t ki;
	uint32_t kd;
};

struct fan_attr_data_pwm_limit {
	uint32_t pwm_min;
	uint32_t pwm_max;
};

union fan_attr_data {
	struct fan_attr_data_pid pid;
	struct fan_attr_data_pwm_limit pwm_limit;
	/*
	 * XXX: Adding new type could increse structure size.
	 * Reserve some space now just in case?
	 */
};

typedef int (*fan_get_mode_t)(const struct device *dev, enum fan_mode *mode);
typedef int (*fan_set_mode_t)(const struct device *dev, enum fan_mode mode);

typedef int (*fan_get_speed_t)(const struct device *dev,
			       enum fan_speed_type type, uint32_t *val);
typedef int (*fan_set_target_t)(const struct device *dev, uint32_t val);

typedef int (*fan_get_attribute_t)(const struct device *dev, enum fan_attribute attr,
				  union fan_attr_data *attr_data);
typedef int (*fan_set_attribute_t)(const struct device *dev, enum fan_attribute attr,
				   const union fan_attr_data *attr_data);

__subsystem struct fan_driver_api {
	fan_get_mode_t fan_get_mode;
	fan_set_mode_t fan_set_mode;

	fan_get_speed_t fan_get_speed;
	fan_set_target_t fan_set_target;

	fan_get_attribute_t fan_get_attribute;
	fan_set_attribute_t fan_set_attribute;
};

__syscall int fan_get_mode(const struct device *dev, enum fan_mode *mode);

static inline int z_impl_fan_get_mode(const struct device *dev, enum fan_mode *mode)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_get_mode(dev, mode);
}

__syscall int fan_set_mode(const struct device *dev, enum fan_mode mode);

static inline int z_impl_fan_set_mode(const struct device *dev, enum fan_mode mode)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_set_mode(dev, mode);
}

__syscall int fan_get_speed(const struct device *dev, enum fan_speed_type type, uint32_t *val);

static inline int z_impl_fan_get_speed(const struct device *dev, enum fan_speed_type type,
				       uint32_t *val)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_get_speed(dev, type, val);
}

__syscall int fan_set_target(const struct device *dev, uint32_t val);

static inline int z_impl_fan_set_target(const struct device *dev, uint32_t val)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_set_target(dev, val);
}

__syscall int fan_get_attribute(const struct device *dev, enum fan_attribute attr,
		      union fan_attr_data *attr_data);

static inline int z_impl_fan_get_attribute(const struct device *dev, enum fan_attribute attr,
				   union fan_attr_data *attr_data)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_get_attribute(dev, attr, attr_data);
}

__syscall int fan_set_attribute(const struct device *dev, enum fan_attribute attr,
		      const union fan_attr_data *attr_data);

static inline int z_impl_fan_set_attribute(const struct device *dev,
				   enum fan_attribute attr,
				   const union fan_attr_data *attr_data)
{
	const struct fan_driver_api *api =
		(const struct fan_driver_api *) dev->api;

	return api->fan_set_attribute(dev, attr, attr_data);
}

#include <syscalls/fan.h>

#endif
