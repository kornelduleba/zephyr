# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/fan.h)

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)

zephyr_library_sources_ifdef(CONFIG_FAN_PID fan_pid.c)
zephyr_library_sources_ifdef(CONFIG_FAN_SHELL fan_shell.c)
