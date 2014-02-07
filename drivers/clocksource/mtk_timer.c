/*
 * Mediatek SoCs General-Purpose Timer handling.
 *
 * Copyright (C) 2014 Matthias Brugger
 *
 * Matthias Brugger <matthias.bgg@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sched_clock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define GPT_IRQ_EN_REG		0x00
#define GPT_IRQ_ENABLE(val)	BIT(val-1)
#define GPT_IRQ_ST_REG		0x04
#define GPT_IRQ_ACK_REG		0x08
#define GPT_IRQ_ACK(val)	BIT(val-1)

#define TIMER_CTRL_REG(val)	(0x10 * val)
#define TIMER_CTRL_OP(val)	(((val) & 0x3) << 4)
#define TIMER_CTRL_OP_ONESHOT	(0)
#define TIMER_CTRL_OP_REPEAT	(1)
#define TIMER_CTRL_OP_KEEPGO	(2)
#define TIMER_CTRL_OP_FREERUN	(3)
#define TIMER_CTRL_CLEAR	(2)
#define TIMER_CTRL_ENABLE	(1)
#define TIMER_CTRL_DISABLE	(0)

#define TIMER_CLK_REG(val)	(0x04 + (0x10 * val))
#define TIMER_CLK_SRC(val)	(((val) & 0x1) << 4)
#define TIMER_CLK_SRC_SYS13M	(0)
#define TIMER_CLK_SRC_RTC32K	(1)
#define TIMER_CLK_DIV1		(0x0)
#define TIMER_CLK_DIV2		(0x1)
#define TIMER_CLK_DIV3		(0x2)
#define TIMER_CLK_DIV4		(0x3)
#define TIMER_CLK_DIV5		(0x4)
#define TIMER_CLK_DIV6		(0x5)
#define TIMER_CLK_DIV7		(0x6)
#define TIMER_CLK_DIV8		(0x7)
#define TIMER_CLK_DIV9		(0x8)
#define TIMER_CLK_DIV10		(0x9)
#define TIMER_CLK_DIV11		(0xA)
#define TIMER_CLK_DIV12		(0xB)
#define TIMER_CLK_DIV13		(0xC)
#define TIMER_CLK_DIV16		(0xD)
#define TIMER_CLK_DIV32		(0xE)
#define TIMER_CLK_DIV64		(0xF)

#define TIMER_CNT_REG(val)	(0x08 + (0x10 * val))
#define TIMER_CMP_REG(val)	(0x0C + (0x10 * val))

#define GPT_CLK_EVT	1
#define GPT_CLK_SRC	2

static void __iomem *gpt_base;
static u32 ticks_per_jiffy;

static void mtk_clkevt_time_stop(u8 timer)
{
	u32 val = readl(gpt_base + TIMER_CTRL_REG(timer));
	writel(val & ~TIMER_CTRL_ENABLE, gpt_base + TIMER_CTRL_REG(timer));
}

static void mtk_clkevt_time_setup(unsigned long delay, u8 timer)
{
	writel(delay, gpt_base + TIMER_CMP_REG(timer));
}

static void mtk_clkevt_time_start(bool periodic, u8 timer)
{
	u32 val;
	
	/* Acknowledge interrupt */
	writel(GPT_IRQ_ACK(timer), gpt_base + GPT_IRQ_ACK_REG);
       
	val = readl(gpt_base + TIMER_CTRL_REG(timer));

	/* Clear 3 bit timer operation field */
	val &= ~TIMER_CTRL_OP(0x3);

	if (periodic)
		val |= TIMER_CTRL_OP(TIMER_CTRL_OP_REPEAT);
	else
		val |= TIMER_CTRL_OP(TIMER_CTRL_OP_ONESHOT);

	writel(val | TIMER_CTRL_ENABLE | TIMER_CTRL_CLEAR,
	       gpt_base + TIMER_CTRL_REG(timer));
}

static void mtk_clkevt_mode(enum clock_event_mode mode,
			      struct clock_event_device *clk)
{
	mtk_clkevt_time_stop(GPT_CLK_EVT);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		mtk_clkevt_time_setup(ticks_per_jiffy, GPT_CLK_EVT);
		mtk_clkevt_time_start(true, GPT_CLK_EVT);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		mtk_clkevt_time_start(false, GPT_CLK_EVT);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		/* No more interrupts will occur as source is disabled */ 
		break;
	}
}

static int mtk_clkevt_next_event(unsigned long evt,
				   struct clock_event_device *unused)
{
	mtk_clkevt_time_stop(GPT_CLK_EVT);
	mtk_clkevt_time_setup(evt, GPT_CLK_EVT);
	mtk_clkevt_time_start(false, GPT_CLK_EVT);

	return 0;
}

static struct clock_event_device mtk_clockevent = {
	.name = "mtk_tick",
	.rating = 300,
	.shift = 32,
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = mtk_clkevt_mode,
	.set_next_event = mtk_clkevt_next_event,
};


static irqreturn_t mtk_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	/* acknowledge timer0 irq */
	writel(GPT_IRQ_ACK(GPT_CLK_EVT), gpt_base + GPT_IRQ_ACK_REG);
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static void mtk_timer_global_reset(void)
{
	// disable all interrupts
	writel(0x0, gpt_base + GPT_IRQ_EN_REG);
	// acknowledge all interrupts
	writel(0x3f, gpt_base + GPT_IRQ_ACK_REG);
}

static void mtk_timer_reset(u8 timer)
{
	writel(TIMER_CTRL_CLEAR | TIMER_CTRL_DISABLE, gpt_base + TIMER_CTRL_REG(timer));
	writel(0x0, gpt_base + TIMER_CMP_REG(timer));
}

static struct irqaction mtk_timer_irq = {
	.name = "mtk_timer0",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = mtk_timer_interrupt,
	.dev_id = &mtk_clockevent,
};

static u32 mtk_timer_sched_read(void)
{
	return readl(gpt_base + TIMER_CNT_REG(GPT_CLK_SRC));
}


void dump_regs()
{
	u32 new;
	pr_warn("gpt_base = %p\n", gpt_base);
	new = readl(gpt_base + GPT_IRQ_EN_REG);
	pr_warn("GPT_IRQ_EN_REG = %x\n", new & 0x3f);
	new = readl(gpt_base + GPT_IRQ_ST_REG);
	pr_warn("GPT_IRQ_ST_REG = %x\n", new & 0x3f);
	new = readl(gpt_base + GPT_IRQ_ACK_REG);
	pr_warn("GPT_IRQ_ACK_REG = %x\n", new & 0x3f);

	pr_warn("clock source\n");
	new = readl(gpt_base + TIMER_CTRL_REG(GPT_CLK_SRC));
	pr_warn("TIMER_CRTL_REG = %x\n", new & 0x33);
	new = readl(gpt_base + TIMER_CLK_REG(GPT_CLK_SRC));
	pr_warn("TIMER_CLK_REG = %x\n", new & 0x1F);
	new = readl(gpt_base + TIMER_CNT_REG(GPT_CLK_SRC));
	pr_warn("TIMER_CNT_REG = %x\n", new);
	new = readl(gpt_base+ TIMER_CMP_REG(GPT_CLK_SRC));
	pr_warn("TIMER_CMP_REG = %x\n", new);

	pr_warn("clock event\n");
	new = readl(gpt_base + TIMER_CTRL_REG(GPT_CLK_EVT));
	pr_warn("TIMER_CRTL_REG = %x\n", new & 0x33);
	new = readl(gpt_base + TIMER_CLK_REG(GPT_CLK_EVT));
	pr_warn("TIMER_CLK_REG = %x\n", new & 0x1F);
	new = readl(gpt_base + TIMER_CNT_REG(GPT_CLK_EVT));
	pr_warn("TIMER_CNT_REG = %x\n", new);
	new = readl(gpt_base + TIMER_CMP_REG(GPT_CLK_EVT));
	pr_warn("TIMER_CMP_REG = %x\n", new);
}
#include <linux/delay.h>
static void __init mtk_timer_init(struct device_node *node)
{
	unsigned long rate = 0;
	struct clk *clk;
	int ret, irq;
	u32 val;

	gpt_base = of_iomap(node, 0);
	if (!gpt_base)
		panic("Can't map registers");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	clk = of_clk_get_by_name(node, "sys_clk");
	//clk = of_clk_get(node, 0);
	if (IS_ERR(clk))
		panic("Can't get timer clockd");
	clk_prepare_enable(clk);

	rate = clk_get_rate(clk);

	mtk_timer_global_reset();
	
	/* configure clock source */
	mtk_timer_reset(GPT_CLK_SRC);

	writel(TIMER_CLK_SRC(TIMER_CLK_SRC_SYS13M) | TIMER_CLK_DIV1,
			gpt_base + TIMER_CLK_REG(GPT_CLK_SRC));

	writel(TIMER_CTRL_OP(TIMER_CTRL_OP_FREERUN) | TIMER_CTRL_ENABLE,
				gpt_base + TIMER_CTRL_REG(GPT_CLK_SRC));

	clocksource_mmio_init(gpt_base + TIMER_CNT_REG(GPT_CLK_SRC), node->name,
			      rate, 300, 32, clocksource_mmio_readl_up);

	ticks_per_jiffy = DIV_ROUND_UP(rate, HZ);

	/* configure clock event */
	mtk_timer_reset(GPT_CLK_EVT);

	writel(TIMER_CLK_SRC(TIMER_CLK_SRC_SYS13M) | TIMER_CLK_DIV1,
			gpt_base + TIMER_CLK_REG(GPT_CLK_EVT));
	writel(0, gpt_base + TIMER_CMP_REG(GPT_CLK_EVT));

	writel(TIMER_CTRL_OP(TIMER_CTRL_OP_REPEAT) | TIMER_CTRL_ENABLE,
			gpt_base + TIMER_CTRL_REG(GPT_CLK_EVT));

	ret = setup_irq(irq, &mtk_timer_irq);
	if (ret)
		pr_warn("failed to setup irq %d\n", irq);

	/* Enable timer0 interrupt */
	val = readl(gpt_base + GPT_IRQ_EN_REG);
	writel(val | GPT_IRQ_ENABLE(GPT_CLK_EVT), gpt_base + GPT_IRQ_EN_REG);

	mtk_clockevent.cpumask = cpumask_of(0);

	clockevents_config_and_register(&mtk_clockevent, rate, 0x3,
					0xffffffff);

	dump_regs();
	printk(KERN_ERR"test timer implementation\n");
	u32 old, new;
	old = mtk_timer_sched_read();
	mdelay(500);
	new = mtk_timer_sched_read();
	pr_warn("old = %u - new = %u for msleep(500)\n", old, new);
}
CLOCKSOURCE_OF_DECLARE(mtk_mt6589, "mediatek,mtk6589-timer",
			mtk_timer_init);
CLOCKSOURCE_OF_DECLARE(mtk_mt6577, "mediatek,mtk6577-timer",
			mtk_timer_init);
CLOCKSOURCE_OF_DECLARE(mtk_mt6572, "mediatek,mtk6572-timer",
			mtk_timer_init);

