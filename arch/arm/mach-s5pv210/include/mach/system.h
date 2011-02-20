/* linux/arch/arm/mach-s5pv210/include/mach/system.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - system support header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

#include <mach/hardware.h>
#include <plat/watchdog-reset.h>

void (*s5pv2xx_reset_hook)(void);

static void arch_idle(void)
{
	/* nothing here yet */
}

static void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		cpu_reset(0);
	}

	if (s5pv2xx_reset_hook)
		s5pv2xx_reset_hook();

	arch_wdt_reset();

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

#endif /* __ASM_ARCH_SYSTEM_H */
