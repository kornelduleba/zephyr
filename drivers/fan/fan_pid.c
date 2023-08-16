/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_fan_pid

#include <zephyr/device.h>

#include <zephyr/drivers/fan.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/math/pid.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephyr_fan_pid, CONFIG_FAN_LOG_LEVEL);

struct fan_pid_config {
	struct pwm_dt_spec pwm;
	const struct device *tach;

	uint32_t pwm_min, pwm_max;
	uint32_t kp, ki, kd;
};

struct fan_pid_data {
	const struct device *dev;
	struct k_work_delayable work;

	struct fan_attr_data_pwm_limit pwm_limit;
	struct pid_state pid_state;

	uint32_t rpm_target;
	enum fan_mode mode;
	uint32_t pwm_val;
};

static int fan_pid_set_pwm(const struct device *dev, uint32_t val)
{
	const struct fan_pid_config *config = dev->config;
	struct fan_pid_data *data = dev->data;
	uint32_t pulse_ns;
	int ret;

	if (val != 0 && val < data->pwm_limit.pwm_min) {
		LOG_WRN("Clamping %d to min value %d", val, data->pwm_limit.pwm_min);
		val = data->pwm_limit.pwm_min;
	}
	if (val > data->pwm_limit.pwm_max) {
		LOG_WRN("Clamping %d to max value %d", val, data->pwm_limit.pwm_max);
		val = data->pwm_limit.pwm_max;
	}

	if (val == data->pwm_val)
		return 0;

	pulse_ns = DIV_ROUND_CLOSEST(config->pwm.period * val, 100);
	ret = pwm_set_pulse_dt(&config->pwm, pulse_ns);
	if (ret != 0) {
		return ret;
	}
	data->pwm_val = DIV_ROUND_CLOSEST(pulse_ns * 100, config->pwm.period);

	return 0;
}

static int fan_pid_fetch_rpm(const struct fan_pid_config *config, uint32_t *val)
{
	struct sensor_value sensor_val = { 0 };
	int ret;

	if (!config->tach) {
		return -ENODEV;
	}

	ret = sensor_sample_fetch_chan(config->tach, SENSOR_CHAN_RPM);
	if (ret) {
		return ret;
	}
	ret = sensor_channel_get(config->tach, SENSOR_CHAN_RPM, &sensor_val);
	if (ret) {
		return ret;
	}

	*val = sensor_val.val1;
	return 0;
}

static int fan_pid_set_rpm(const struct device *dev, uint32_t val)
{
	struct fan_pid_data *data = dev->data;
	struct k_work_sync work_sync;
	int ret = 0;

	data->rpm_target = val;
	if (data->rpm_target == 0) {
		k_work_cancel_delayable_sync(&data->work, &work_sync);
		ret = fan_pid_set_pwm(dev, 0);
		pid_reset_state(&data->pid_state);
	} else {
		k_work_reschedule(&data->work, K_NO_WAIT);
	}

	return ret;
}

static bool fan_pid_rpm_mode_available(const struct device *dev)
{
	const struct fan_pid_config *config = dev->config;
	struct fan_pid_data *data = dev->data;

	if (!config->tach || data->pid_state.kp == 0 || data->pid_state.ki == 0) {
		return false;
	}
	return true;
}

static int fan_pid_get_mode(const struct device *dev, enum fan_mode *mode)
{
	struct fan_pid_data *data = dev->data;

	*mode = data->mode;

	return 0;
}

static int fan_pid_set_mode(const struct device *dev, enum fan_mode mode)
{
	struct fan_pid_data *data = dev->data;
	int ret;

	if (data->mode == mode) {
		return 0;
	}

	switch (mode) {
	case FAN_MODE_PWM:
		ret = fan_pid_set_rpm(dev, 0);
		if (ret != 0) {
			return ret;
		}
		break;
	case FAN_MODE_RPM:
		if (!fan_pid_rpm_mode_available(dev)) {
			return -ENOTSUP;
		}
		ret = fan_pid_set_pwm(dev, 0);
		if (ret != 0) {
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	}

	data->mode = mode;
	LOG_DBG("%s: Setting mode to %d", __func__, mode);
	return 0;
}

static int fan_pid_get_speed(const struct device *dev, enum fan_speed_type type, uint32_t *val)
{
	const struct fan_pid_config *config = dev->config;
	struct fan_pid_data *data = dev->data;
	int ret;

	switch (type) {
	case FAN_SPEED_TYPE_PWM:
		*val = data->pwm_val;
		ret = 0;
		break;
	case FAN_SPEED_TYPE_RPM:
		ret = fan_pid_fetch_rpm(config, val);
		break;
	case FAN_SPEED_TYPE_TARGET:
		switch (data->mode) {
		case FAN_MODE_PWM:
			*val = data->pwm_val;
			ret = 0;
			break;
		case FAN_MODE_RPM:
			*val = data->rpm_target;
			ret = 0;
			break;
		default:
			ret = -ENOTSUP;
			break;
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int fan_pid_get_attribute(const struct device *dev,
				  enum fan_attribute attr,
				  union fan_attr_data *attr_data)
{
	struct fan_pid_data *data = dev->data;

	switch (attr) {
	case FAN_ATTR_PID:
		struct fan_attr_data_pid *pid_data = &attr_data->pid;

		pid_data->kp = (uint32_t)(data->pid_state.kp * PID_PARAM_DIVIDER);
		pid_data->ki = (uint32_t)(data->pid_state.ki * PID_PARAM_DIVIDER);
		pid_data->kd = (uint32_t)(data->pid_state.kd * PID_PARAM_DIVIDER);
		break;
	case FAN_ATTR_PWM_LIMIT:
		struct fan_attr_data_pwm_limit *pwm_limit = &attr_data->pwm_limit;

		*pwm_limit = data->pwm_limit;
		break;
	default:
		return -ENOTSUP;
	};

	return 0;
}

static int fan_pid_set_attribute(const struct device *dev, enum fan_attribute attr,
				  const union fan_attr_data *attr_data)
{
	struct fan_pid_data *data = dev->data;

	switch (attr) {
	case FAN_ATTR_PID:
		const struct fan_attr_data_pid *pid_data = &attr_data->pid;

		pid_set_parameters(&data->pid_state,
				   (float)pid_data->kp / PID_PARAM_DIVIDER,
				   (float)pid_data->ki / PID_PARAM_DIVIDER,
				   (float)pid_data->kd / PID_PARAM_DIVIDER);
		break;
	case FAN_ATTR_PWM_LIMIT:
		const struct fan_attr_data_pwm_limit *pwm_limit = &attr_data->pwm_limit;

		data->pwm_limit = *pwm_limit;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int fan_pid_set_target(const struct device *dev, uint32_t val)
{
	struct fan_pid_data *data = dev->data;

	switch (data->mode) {
	case FAN_MODE_PWM:
		return fan_pid_set_pwm(dev, val);
	case FAN_MODE_RPM:
		return fan_pid_set_rpm(dev, val);
	default:
		return -ENOTSUP;
	}

}

static const struct fan_driver_api fan_pid_driver_api = {
	.fan_get_mode = fan_pid_get_mode,
	.fan_set_mode = fan_pid_set_mode,

	.fan_get_speed = fan_pid_get_speed,
	.fan_set_target = fan_pid_set_target,

	.fan_get_attribute = fan_pid_get_attribute,
	.fan_set_attribute = fan_pid_set_attribute
};

static void fan_pid_work(struct k_work *work)
{
	struct fan_pid_data *data = CONTAINER_OF(k_work_delayable_from_work(work),
						 struct fan_pid_data, work);
	const struct fan_pid_config *config = data->dev->config;
	float dt, new_duty;
	int current_rpm;
	int ret;

	dt = 50;	/* TODO */
	new_duty = data->pwm_limit.pwm_max;
	ret = fan_pid_fetch_rpm(config, &current_rpm);
	if (ret != 0) {
		/* If we can't get a reading, let's just try again later. */
		/* XXX: Switch to PWM mode here instead? */
		LOG_WRN("Failed to read data from tachometer: %d", ret);
		goto out;
	}

	/*
	 * Can't increase the duty more.
	 * There are two possibilities here.
	 * 1. The fan is spinning up, let's wait a bit.
	 * 2. We're either stalled, or the target RPM can't be reached.
	 * Either way don't do the PID calculation to not inflate the integral part.
	 * TODO: Add a mechanism to detect fan stall and print a warning.
	 */
	if (data->pwm_val == data->pwm_limit.pwm_max) {
		goto out;
	}

	new_duty = pid(&data->pid_state, (int)data->rpm_target - current_rpm, dt);
	if (new_duty < 0)
		new_duty = 0;
	if (new_duty > 0 && new_duty < data->pwm_limit.pwm_min)
		new_duty = data->pwm_limit.pwm_min;
	if (new_duty > data->pwm_limit.pwm_max)
		new_duty = data->pwm_limit.pwm_max;

	if ((uint32_t)new_duty != data->pwm_val) {
		LOG_DBG("rpm_target: %d, rpm_actual: %d, rpm_diff: %d, old_duty %d:"
		       " new_duty: %d", data->rpm_target, current_rpm,
		       data->rpm_target - current_rpm, data->pwm_val,
		       (uint32_t)new_duty);
		fan_pid_set_pwm(data->dev, (uint32_t)new_duty);
	}
out:
	k_work_reschedule(&data->work, K_MSEC(50));
}

static int fan_pid_init(const struct device *dev)
{
	const struct fan_pid_config *config = dev->config;
	struct fan_pid_data *data = dev->data;
	struct pid_state *pid_state = &data->pid_state;

	if (!device_is_ready(config->pwm.dev)) {
		LOG_ERR("%s: pwm device is not ready", dev->name);
		return -ENODEV;
	}
	if (config->tach && !device_is_ready(config->tach)) {
		LOG_ERR("%s: tachometer device is not ready", dev->name);
		return -ENODEV;
	}

	k_work_init_delayable(&data->work, fan_pid_work);

	/* Default to PWM mode. */
	data->mode = FAN_MODE_PWM;
	data->dev = dev;
	data->pwm_limit.pwm_min = config->pwm_min;
	data->pwm_limit.pwm_max = config->pwm_max;
	pid_set_parameters(pid_state,
			   (float)config->kp / PID_PARAM_DIVIDER,
			   (float)config->ki / PID_PARAM_DIVIDER,
			   (float)config->kd / PID_PARAM_DIVIDER);
	return 0;
}

#define FAN_PID_INIT(i)								\
	static const struct fan_pid_config fan_pid_config_##i =			\
	{									\
		.pwm    = PWM_DT_SPEC_INST_GET(i),				\
		.tach   = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(i), tach)),	\
		.pwm_min = DT_INST_PROP(i, pwm_min),				\
		.pwm_max = DT_INST_PROP(i, pwm_max),				\
		.kp      = DT_INST_PROP(i, kp),					\
		.ki      = DT_INST_PROP(i, ki),					\
		.kd      = DT_INST_PROP(i, kd),					\
	};									\
	static struct fan_pid_data fan_pid_data_##i;				\
	DEVICE_DT_INST_DEFINE(i, fan_pid_init, NULL, &fan_pid_data_##i,		\
			      &fan_pid_config_##i, APPLICATION,			\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
			      &fan_pid_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FAN_PID_INIT)
