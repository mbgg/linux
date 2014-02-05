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
#define GPT_IRQ_ENABLE(val)	BIT(val)
#define GPT_IRQ_ST_REG		0x04
#define GPT_IRQ_ACK_REG		0x08
#define GPT_IRQ_ACK(val)	BIT(val)

#define TIMER_CTRL_REG		0x00
//0x10
#define TIMER_CTRL_OP(val)	(((val) & 0x3) << 4)
#define TIMER_CTRL_OP_ONESHOT	(0)
#define TIMER_CTRL_OP_REPEAT	(1)
#define TIMER_CTRL_OP_KEEPGO	(2)
#define TIMER_CTRL_OP_FREERUN	(3)
#define TIMER_CTRL_CLEAR	(2)
#define TIMER_CTRL_ENABLE	(1)
#define TIMER_CTRL_DISABLE	(0)

#define TIMER_CLK_REG		0x04
//0x14
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

#define TIMER_CNT_REG		0x08
//0x18
#define TIMER_CMP_REG		0x0C
//0x1C

static void __iomem *gpt_base, *clksource_base, *clkevent_base;
static u32 ticks_per_jiffy;

static void mtk_clkevt_time_stop(void)
{
	u32 val = readl(clkevent_base + TIMER_CTRL_REG);
	writel(val & ~TIMER_CTRL_ENABLE, clkevent_base + TIMER_CTRL_REG);
}

static void mtk_clkevt_time_setup(unsigned long delay)
{
	writel(delay, clkevent_base + TIMER_CMP_REG);
}

static void mtk_clkevt_time_start(bool periodic)
{
	u32 val;
	
	/* Acknowledge interrupt */
	writel(GPT_IRQ_ACK(0), gpt_base + GPT_IRQ_ACK_REG);
       
	val = readl(clkevent_base + TIMER_CTRL_REG);

	/* Clear 3 bit timer operation field */
	val &= ~TIMER_CTRL_OP(0x3);

	if (periodic)
		val |= TIMER_CTRL_OP(TIMER_CTRL_OP_REPEAT);
	else
		val |= TIMER_CTRL_OP(TIMER_CTRL_OP_ONESHOT);

	writel(val | TIMER_CTRL_ENABLE | TIMER_CTRL_CLEAR,
	       clkevent_base + TIMER_CTRL_REG);
}

static void mtk_clkevt_mode(enum clock_event_mode mode,
			      struct clock_event_device *clk)
{
	mtk_clkevt_time_stop();

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		mtk_clkevt_time_setup(ticks_per_jiffy);
		mtk_clkevt_time_start(true);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		mtk_clkevt_time_start(false);
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
	mtk_clkevt_time_stop();
	mtk_clkevt_time_setup(evt);
	mtk_clkevt_time_start(false);

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
	writel(GPT_IRQ_ACK(0), gpt_base + GPT_IRQ_ACK_REG);
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

static void mtk_timer_reset(void __iomem *base)
{

	// TODO we can do this in one step?
	//writel(TIMER_CTRL_CLEAR | TIMER_CTRL_DISABLE, base + TIMER_CTRL_REG);
	writel(TIMER_CTRL_DISABLE, base + TIMER_CTRL_REG);

	// TODO we might not need write to TIMER_CLK_REG
	/* Use system clock and no divider */
	writel(0x0, base + TIMER_CLK_REG);

	writel(TIMER_CTRL_CLEAR, base + TIMER_CTRL_REG);

	writel(0x0, base + TIMER_CMP_REG);

}

static struct irqaction mtk_timer_irq = {
	.name = "mtk_timer0",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = mtk_timer_interrupt,
	.dev_id = &mtk_clockevent,
};

static u32 mtk_timer_sched_read(void)
{
	return readl(clksource_base + TIMER_CNT_REG);
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

	pr_warn("clksource_base = %p\n", clksource_base);
	new = readl(clksource_base + TIMER_CTRL_REG);
	pr_warn("TIMER_CRTL_REG = %x\n", new & 0x33);
	new = readl(clksource_base + TIMER_CLK_REG);
	pr_warn("TIMER_CLK_REG = %x\n", new & 0x1F);
	new = readl(clksource_base + TIMER_CNT_REG);
	pr_warn("TIMER_CNT_REG = %x\n", new);
	new = readl(clksource_base + TIMER_CMP_REG);
	pr_warn("TIMER_CMP_REG = %x\n", new);

	pr_warn("clkevent_base = %p\n", clkevent_base);
	new = readl(clkevent_base + TIMER_CTRL_REG);
	pr_warn("TIMER_CRTL_REG = %x\n", new & 0x33);
	new = readl(clkevent_base + TIMER_CLK_REG);
	pr_warn("TIMER_CLK_REG = %x\n", new & 0x1F);
	new = readl(clkevent_base + TIMER_CNT_REG);
	pr_warn("TIMER_CNT_REG = %x\n", new);
	new = readl(clkevent_base + TIMER_CMP_REG);
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
		panic("Can't map registers gpt_base");

	clkevent_base = of_iomap(node, 1);
	if (!clkevent_base)
		panic("Can't map registers clkevent_base");

	clksource_base = of_iomap(node, 2);
	if (!clksource_base)
		panic("Can't map registers clksource_base");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");
	/*
	clk = of_clk_get_by_name(node, "system13m");
	if (IS_ERR(clk))
		panic("Can't get timer clock");
	clk_prepare_enable(clk);

	// clockrate is arlead set to 13MHz
	rate = clk_get_rate(clk);
	*/
	rate = 15000000;

	mtk_timer_global_reset();
	
	/* configure clock source */
	mtk_timer_reset(clksource_base);
	//writel(TIMER_CTRL_OP(TIMER_CTRL_OP_FREERUN), clksource_base + TIMER_CTRL_REG);

	writel(TIMER_CLK_SRC(TIMER_CLK_SRC_SYS13M) | TIMER_CLK_DIV1,
			clksource_base + TIMER_CLK_REG);

	//writel(TIMER_CTRL_OP(TIMER_CTRL_ENABLE), clksource_base + TIMER_CTRL_REG);
	writel(TIMER_CTRL_OP(TIMER_CTRL_OP_FREERUN) | TIMER_CTRL_ENABLE,
				clksource_base + TIMER_CTRL_REG);

	setup_sched_clock(mtk_timer_sched_read, 32, rate);
	clocksource_mmio_init(clksource_base + TIMER_CNT_REG, node->name,
			      rate, 300, 32, clocksource_mmio_readl_down);

	ticks_per_jiffy = DIV_ROUND_UP(rate, HZ);

	/* configure clock event */
	mtk_timer_reset(clkevent_base);
	//writel(TIMER_CTRL_OP(TIMER_CTRL_OP_REPEAT), clkevent_base + TIMER_CTRL_REG);

	writel(TIMER_CLK_SRC(TIMER_CLK_SRC_SYS13M) | TIMER_CLK_DIV1,
			clkevent_base + TIMER_CLK_REG);
	writel(0, clkevent_base + TIMER_CMP_REG);

	/* TODO really enable IRQ before setup_irq????
	 * Enable timer0 interrupt */
	val = readl(gpt_base + GPT_IRQ_EN_REG);
	writel(val | GPT_IRQ_ENABLE(0), gpt_base + GPT_IRQ_EN_REG);

	//writel(TIMER_CTRL_OP(TIMER_CTRL_ENABLE), clkevent_base + TIMER_CTRL_REG);
	writel(TIMER_CTRL_OP(TIMER_CTRL_OP_REPEAT) | TIMER_CTRL_ENABLE,
			clkevent_base + TIMER_CTRL_REG);

	ret = setup_irq(irq, &mtk_timer_irq);
	if (ret)
		pr_warn("failed to setup irq %d\n", irq);

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

