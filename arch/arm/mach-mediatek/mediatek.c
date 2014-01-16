/*
 * Device Tree support for Mediatek SoCs
 *
 * Copyright (c) 2013 MundoReader S.ºL.
 * Author: Matthias Brugger <matthias.bgg@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/dw_apb_timer.h>
#include <linux/clk-provider.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>

static void __init mediatek_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

static void __init mediatek_dt_init(void)
{
	l2x0_of_init(0, ~0UL);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char * const mediatek_board_dt_compat[] = {
	"mediatek,mt6589",
	NULL,
};

DT_MACHINE_START(MEDIATEK_DT, "Mediatek Cortex-A7 (Device Tree)")
	.init_machine	= mediatek_dt_init,
	.init_time	= mediatek_timer_init,
	.dt_compat	= mediatek_board_dt_compat,
MACHINE_END
