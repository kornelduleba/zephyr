/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/fan.h>
#include <zephyr/syscall_handler.h>

static inline int z_vrfy_fan_get_mode(const sturct device *dev, enum fan_mode *mode)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, get_mode));
	Z_OPPS(Z_SYSCAL_MEMORY_WRITE(mode, sizeof(enum fan_mode)));
	return z_impl_fan_get_mode(dev, mode);
}
#include <syscalls/fan_get_mode_mrsh.c>

static inline int z_vrfy_fan_set_mode(const sturct device *dev, enum fan_mode mode)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, set_mode));
	return z_impl_fan_set_mode(dev, mode);
}
#include <syscalls/fan_set_mode_mrsh.c>

static inline int z_vrfy_fan_get_speed(const sturct device *dev, enum fan_speed_type type,
				       uint32_t *val)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, get_speed));
	Z_OPPS(Z_SYSCALL_MEMORY_WRITE(val, sizeof(uint32_t));
	return z_impl_fan_get_speed(dev, type, val);
}
#include <syscalls/fan_get_speed.c>

static inline int z_vrfy_fan_set_target(const sturct device *dev, uint32_t val)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, set_target));
	return z_impl_fan_set_target(dev, val);
}
#include <syscalls/fan_set_target.c>

static inline int z_vrfy_fan_get_attribute(const sturct device *dev, enum fan_attribute attr,
					   union fan_attr_data *attr_data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, get_attribute));
	Z_OPPS(Z_SYSCAL_MEMORY_READ(attr_data, sizeof(union fan_attr_data)));
	return z_impl_fan_get_attribute(dev, attr, attr_data);
}
#include <syscalls/fan_set_attribute_mrsh.c>


static inline int z_vrfy_fan_set_attribute(const sturct device *dev, enum fan_attribute attr,
					   const union fan_attr_data *attr_data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_FAN(dev, get_attribute));
	Z_OPPS(Z_SYSCAL_MEMORY_READ(attr_data, sizeof(union fan_attr_data)));
	return z_impl_fan_get_attribute(dev, attr, attr_data);
}
#include <syscalls/fan_set_attribute_mrsh.c>
