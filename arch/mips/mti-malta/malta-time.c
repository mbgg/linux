/*
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Setting up the clock on the MIPS boards.
 */
#include <linux/types.h>
#include <linux/i8253.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/mc146818rtc.h>

#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/hardirq.h>
#include <asm/irq.h>
#include <asm/div64.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/mc146818-time.h>
#include <asm/msc01_ic.h>
#include <asm/gic.h>

#include <asm/mips-boards/generic.h>
#include <asm/mips-boards/prom.h>

#include <asm/mips-boards/maltaint.h>

unsigned long cpu_khz;
int gic_frequency;

static int mips_cpu_timer_irq;
static int mips_cpu_perf_irq;
extern int cp0_perfcount_irq;

static void mips_timer_dispatch(void)
{
	do_IRQ(mips_cpu_timer_irq);
}

static void mips_perf_dispatch(void)
{
	do_IRQ(mips_cpu_perf_irq);
}

static unsigned int freqround(unsigned int freq, unsigned int amount)
{
	freq += amount;
	freq -= freq % (amount*2);
	return freq;
}

/*
 * Estimate CPU and GIC frequencies.
 */
static void __init estimate_frequencies(void)
{
	unsigned long flags;
	unsigned int count, start;
	unsigned int giccount = 0, gicstart = 0;

#if defined (CONFIG_KVM_GUEST) && defined (CONFIG_KVM_HOST_FREQ)
	/*
	 * XXXKYMA: hardwire the CPU frequency to Host Freq/4
	 */
	count = (CONFIG_KVM_HOST_FREQ * 1000000) >> 3;
	if ((prid != (PRID_COMP_MIPS | PRID_IMP_20KC)) &&
	    (prid != (PRID_COMP_MIPS | PRID_IMP_25KF)))
		count *= 2;

	mips_hpt_frequency = count;
	return count;
#endif

	local_irq_save(flags);

	/* Start counter exactly on falling edge of update flag. */
	while (CMOS_READ(RTC_REG_A) & RTC_UIP);
	while (!(CMOS_READ(RTC_REG_A) & RTC_UIP));

	/* Initialize counters. */
	start = read_c0_count();
	if (gic_present)
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_31_00), gicstart);

	/* Read counter exactly on falling edge of update flag. */
	while (CMOS_READ(RTC_REG_A) & RTC_UIP);
	while (!(CMOS_READ(RTC_REG_A) & RTC_UIP));

	count = read_c0_count();
	if (gic_present)
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_31_00), giccount);

	local_irq_restore(flags);

	count -= start;
	if (gic_present)
		giccount -= gicstart;

	mips_hpt_frequency = count;
	if (gic_present)
		gic_frequency = giccount;
}

void read_persistent_clock(struct timespec *ts)
{
	ts->tv_sec = mc146818_get_cmos_time();
	ts->tv_nsec = 0;
}

static void __init plat_perf_setup(void)
{
#ifdef MSC01E_INT_BASE
	if (cpu_has_veic) {
		set_vi_handler(MSC01E_INT_PERFCTR, mips_perf_dispatch);
		mips_cpu_perf_irq = MSC01E_INT_BASE + MSC01E_INT_PERFCTR;
	} else
#endif
	if (cp0_perfcount_irq >= 0) {
		if (cpu_has_vint)
			set_vi_handler(cp0_perfcount_irq, mips_perf_dispatch);
		mips_cpu_perf_irq = MIPS_CPU_IRQ_BASE + cp0_perfcount_irq;
#ifdef CONFIG_SMP
		irq_set_handler(mips_cpu_perf_irq, handle_percpu_irq);
#endif
	}
}

unsigned int __cpuinit get_c0_compare_int(void)
{
#ifdef MSC01E_INT_BASE
	if (cpu_has_veic) {
		set_vi_handler(MSC01E_INT_CPUCTR, mips_timer_dispatch);
		mips_cpu_timer_irq = MSC01E_INT_BASE + MSC01E_INT_CPUCTR;
	} else
#endif
	{
		if (cpu_has_vint)
			set_vi_handler(cp0_compare_irq, mips_timer_dispatch);
		mips_cpu_timer_irq = MIPS_CPU_IRQ_BASE + cp0_compare_irq;
	}

	return mips_cpu_timer_irq;
}

void __init plat_time_init(void)
{
	unsigned int prid = read_c0_prid() & 0xffff00;
	unsigned int freq;

	estimate_frequencies();

	freq = mips_hpt_frequency;
	if ((prid != (PRID_COMP_MIPS | PRID_IMP_20KC)) &&
	    (prid != (PRID_COMP_MIPS | PRID_IMP_25KF)))
		freq *= 2;
	freq = freqround(freq, 5000);
	pr_debug("CPU frequency %d.%02d MHz\n", freq/1000000,
	       (freq%1000000)*100/1000000);
	cpu_khz = freq / 1000;

	if (gic_present) {
		freq = freqround(gic_frequency, 5000);
		pr_debug("GIC frequency %d.%02d MHz\n", freq/1000000,
		       (freq%1000000)*100/1000000);
		gic_clocksource_init(gic_frequency);
	} else
		init_r4k_clocksource();

#ifdef CONFIG_I8253
	/* Only Malta has a PIT. */
	setup_pit_timer();
#endif

	mips_scroll_message();

	plat_perf_setup();
}
