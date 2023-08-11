/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/fan.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/iterable_sections.h>

LOG_MODULE_REGISTER(fan_shell);

/* TODO Help text. */
#define FAN_GET_HELP \
	"TODO"

enum fan_cmd {
	FAN_CMD_NONE = -1,
	FAN_CMD_ATTR,
	FAN_CMD_MODE,
	FAN_CMD_SPEED,
	FAN_CMD_MAX,
};

const char *fan_name_mode[FAN_MODE_MAX] = {
	[FAN_MODE_PWM] = "pwm",
	[FAN_MODE_RPM] = "rpm",
};

const char *fan_name_speed_type[FAN_SPEED_TYPE_MAX] = {
	[FAN_SPEED_TYPE_PWM] = "pwm",
	[FAN_SPEED_TYPE_RPM] = "rpm",
	[FAN_SPEED_TYPE_TARGET] = "target",
};

const char *fan_name_attr[FAN_ATTR_MAX] = {
	[FAN_ATTR_PID] = "pid",
	[FAN_ATTR_PWM_LIMIT] = "pwm_limit",
};

const char *fan_name_cmd_type[FAN_CMD_MAX] = {
	[FAN_CMD_ATTR] = "attr",
	[FAN_CMD_MODE] = "mode",
	[FAN_CMD_SPEED] = "speed",
};

static enum fan_cmd current_cmd_context = FAN_CMD_NONE;
bool current_cmd_isset;

static void subcmd_select_cmd_attr(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_select_cmd_attr, subcmd_select_cmd_attr);
static void subcmd_select_cmd_attr(size_t idx, struct shell_static_entry *entry)
{

	switch (current_cmd_context) {
	case FAN_CMD_ATTR:
		if (idx < ARRAY_SIZE(fan_name_attr)) {
			entry->syntax = fan_name_attr[idx];
			break;
		}
		entry->syntax = NULL;
		break;
	case FAN_CMD_MODE:
		if (idx < ARRAY_SIZE(fan_name_mode)) {
			entry->syntax = fan_name_mode[idx];
			break;
		}
		entry->syntax = NULL;
		break;
	case FAN_CMD_SPEED:
		if (idx < ARRAY_SIZE(fan_name_speed_type)) {
			entry->syntax = fan_name_speed_type[idx];
			break;
		}
		/* fallthrough */
	case FAN_CMD_NONE:
	default:
		entry->syntax = NULL;
		break;
	}
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

static void subcmd_select_device(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_select_device, subcmd_select_device);
static void subcmd_select_device(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;

	if ((current_cmd_isset && current_cmd_context == FAN_CMD_SPEED) ||
	    (!current_cmd_isset && current_cmd_context == FAN_CMD_MODE)) {
		entry->subcmd = NULL;
		return;
	}
	entry->subcmd = &dsub_select_cmd_attr;
}

static void subcmd_parse(size_t idx, struct shell_static_entry *entry)
{
	entry->subcmd = &dsub_select_device;
	entry->handler = NULL;
	entry->help = NULL;
	if (idx < ARRAY_SIZE(fan_name_cmd_type)) {
		entry->syntax = fan_name_cmd_type[idx];
		current_cmd_context = idx;
	} else {
		entry->syntax = NULL;
		current_cmd_context = FAN_CMD_NONE;
	}
}

static void subcmd_get(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_get, subcmd_get);
static void subcmd_get(size_t idx, struct shell_static_entry *entry)
{
	current_cmd_isset = false;
	subcmd_parse(idx, entry);
}

static void subcmd_set(size_t idx, struct shell_static_entry *entry);
SHELL_DYNAMIC_CMD_CREATE(dsub_set, subcmd_set);
static void subcmd_set(size_t idx, struct shell_static_entry *entry)
{
	current_cmd_isset = true;
	subcmd_parse(idx, entry);
}

static int parse_string(char *str, const char *array[], size_t count)
{
	for (int i = 0; i < count; i++) {
		if (!strcmp(str, array[i]))
			return i;
	}
	return -1;
}

static int cmd_get(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	enum fan_cmd cmd;
	int ret;

	cmd = parse_string(argv[1], fan_name_cmd_type, ARRAY_SIZE(fan_name_cmd_type));
	if (cmd == FAN_CMD_NONE) {
		shell_error(sh, "Unknown command %s", argv[1]);
		return -ENOTSUP;
	}
	dev = device_get_binding(argv[2]);
	if (!dev) {
		shell_error(sh, "Device not found %s", argv[2]);
		return -ENODEV;
	}

	switch (cmd) {
	case FAN_CMD_MODE:
		enum fan_mode mode;

		ret = fan_get_mode(dev, &mode);
		if (ret != 0) {
			shell_error(sh, "Failed to read fan mode");
			return ret;
		}
		if (mode > ARRAY_SIZE(fan_name_mode)) {
			shell_error(sh, "Unknown fan mode %d", mode);
			return -ENOTSUP;
		}
		shell_print(sh, "%s", fan_name_mode[mode]);
		break;
	case FAN_CMD_SPEED:
		int speed_type;
		uint32_t speed;

		/* XXX: Print all speeds in this case? */
		if (argc < 4) {
			shell_error(sh, "Fan speed type unspecified");
			return -EINVAL;
		}

		speed_type = parse_string(argv[3], fan_name_speed_type,
					 ARRAY_SIZE(fan_name_speed_type));
		if (speed_type == -1) {
			shell_error(sh, "Unknown fan speed type %s", argv[3]);
			return -EINVAL;
		}
		ret = fan_get_speed(dev, speed_type, &speed);
		if (ret != 0) {
			shell_error(sh, "Failed to read fan speed");
			return ret;
		}
		shell_print(sh, "%d", speed);
		break;
	case FAN_CMD_ATTR:
		/* XXX: Move this to a helper function? */
		union fan_attr_data data;
		int attr;

		if (argc < 4) {
			shell_error(sh, "Fan attribute type unspecified");
			return -EINVAL;
		}
		attr = parse_string(argv[3], fan_name_attr, ARRAY_SIZE(fan_name_attr));
		if (attr == -1) {
			shell_error(sh, "Unknown attribute %s", argv[3]);
			return -EINVAL;
		}
		ret = fan_get_attribute(dev, attr, &data);
		if (ret != 0) {
			shell_error(sh, "Failed to read attribute");
			return ret;
		}
		switch (attr) {
		case FAN_ATTR_PID:
			shell_print(sh, "kp: %d, ki: %d, kd: %d",
				    data.pid.kp, data.pid.ki, data.pid.kd);
			break;
		case FAN_ATTR_PWM_LIMIT:
			shell_print(sh, "pwm_min: %d, pwm_max: %d",
				    data.pwm_limit.pwm_min, data.pwm_limit.pwm_max);
			break;
		default:
			shell_error(sh, "Unsupported attribute %s", argv[3]);
			return -ENOTSUP;
		}
		break;
	default:
		return -ENOTSUP;
	};

	return 0;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	enum fan_cmd cmd;
	char *endptr;
	int ret;

	cmd = parse_string(argv[1], fan_name_cmd_type, ARRAY_SIZE(fan_name_cmd_type));
	if (cmd == FAN_CMD_NONE) {
		shell_error(sh, "Unknown command %s", argv[1]);
		return -ENOTSUP;
	}
	dev = device_get_binding(argv[2]);
	if (!dev) {
		shell_error(sh, "Device not found %s", argv[1]);
		return -ENODEV;
	}

	switch (cmd) {
	case FAN_CMD_MODE:
		int mode;

		mode = parse_string(argv[3], fan_name_mode, ARRAY_SIZE(fan_name_mode));
		if (mode == -1) {
			shell_error(sh, "Unknown mode %s", argv[3]);
			return -ENOTSUP;
		}
		ret = fan_set_mode(dev, mode);
		if (ret != 0) {
			shell_error(sh, "Failed to set fan mode");
			return ret;
		}
		break;
	case FAN_CMD_SPEED:
		uint32_t speed;

		speed = strtol(argv[3], &endptr, 0);
		if (*endptr != '\0') {
			shell_error(sh, "Incorrect fan speed %s", argv[3]);
			return -EINVAL;
		}

		ret = fan_set_target(dev, speed);
		if (ret != 0) {
			shell_error(sh, "Failed to set fan speed");
			return ret;
		}
		break;
	case FAN_CMD_ATTR:
		/* XXX: Move this to a helper function? */
		union fan_attr_data data;
		int attr;

		if (argc < 4) {
			shell_error(sh, "Fan attribute type unspecified");
			return -EINVAL;
		}
		attr = parse_string(argv[3], fan_name_attr, ARRAY_SIZE(fan_name_attr));
		if (attr == -1) {
			shell_error(sh, "Unknown attribute %s", argv[3]);
			return -EINVAL;
		}

		switch (attr) {
		case FAN_ATTR_PID:
			if (argc < 7) {
				shell_error(sh, "Fan attr PID requires 3 data arguments");
				return -EINVAL;
			}
			data.pid.kp = strtol(argv[4], &endptr, 0);
			if (*endptr != '\0') {
				shell_error(sh, "Incorrect kp attribute %s", argv[4]);
				return -EINVAL;
			}
			data.pid.ki = strtol(argv[5], &endptr, 0);
			if (*endptr != '\0') {
				shell_error(sh, "Incorrect ki attribute %s", argv[5]);
				return -EINVAL;
			}
			data.pid.kd = strtol(argv[6], &endptr, 0);
			if (*endptr != '\0') {
				shell_error(sh, "Incorrect kd attribute %s", argv[6]);
				return -EINVAL;
			}
			break;
		case FAN_ATTR_PWM_LIMIT:
			if (argc < 6) {
				shell_error(sh, "Fan attr pwm_limit requires 2 data arguments");
				return -EINVAL;
			}
			data.pwm_limit.pwm_min = strtol(argv[4], &endptr, 0);
			if (*endptr != '\0') {
				shell_error(sh, "Incorrect pwm_min attribute %s", argv[4]);
				return -EINVAL;
			}
			data.pwm_limit.pwm_max = strtol(argv[5], &endptr, 0);
			if (*endptr != '\0') {
				shell_error(sh, "Incorrect pwm_min attribute %s", argv[5]);
				return -EINVAL;
			}
			break;
		}

		ret = fan_set_attribute(dev, attr, &data);
		if (ret != 0) {
			shell_error(sh, "Failed to set fan attribute");
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	};
	return 0;
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_fan,
	SHELL_CMD_ARG(get, &dsub_get, NULL, cmd_get, 3, 1),
	SHELL_CMD_ARG(set, &dsub_set, NULL, cmd_set, 4, 3),
	SHELL_SUBCMD_SET_END
	);
/* clang-format on */

SHELL_CMD_REGISTER(fan, &sub_fan, "Fan commands", NULL);
