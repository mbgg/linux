// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 Pengutronix, Sascha Hauer <kernel@pengutronix.de>
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of_clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/mediatek/infracfg.h>

#include <dt-bindings/power/mt2701-power.h>
#include <dt-bindings/power/mt2712-power.h>
#include <dt-bindings/power/mt6797-power.h>
#include <dt-bindings/power/mt7622-power.h>
#include <dt-bindings/power/mt7623a-power.h>
#include <dt-bindings/power/mt8173-power.h>

#define MTK_POLL_DELAY_US   10
#define MTK_POLL_TIMEOUT    USEC_PER_SEC

#define MTK_SCPD_ACTIVE_WAKEUP		BIT(0)
#define MTK_SCPD_FWAIT_SRAM		BIT(1)
#define MTK_SCPD_CAPS(_scpd, _x)	((_scpd)->data->caps & (_x))

#define SPM_VDE_PWR_CON			0x0210
#define SPM_MFG_PWR_CON			0x0214
#define SPM_VEN_PWR_CON			0x0230
#define SPM_ISP_PWR_CON			0x0238
#define SPM_DIS_PWR_CON			0x023c
#define SPM_CONN_PWR_CON		0x0280
#define SPM_VEN2_PWR_CON		0x0298
#define SPM_AUDIO_PWR_CON		0x029c	/* MT8173, MT2712 */
#define SPM_BDP_PWR_CON			0x029c	/* MT2701 */
#define SPM_ETH_PWR_CON			0x02a0
#define SPM_HIF_PWR_CON			0x02a4
#define SPM_IFR_MSC_PWR_CON		0x02a8
#define SPM_MFG_2D_PWR_CON		0x02c0
#define SPM_MFG_ASYNC_PWR_CON		0x02c4
#define SPM_USB_PWR_CON			0x02cc
#define SPM_USB2_PWR_CON		0x02d4	/* MT2712 */
#define SPM_ETHSYS_PWR_CON		0x02e0	/* MT7622 */
#define SPM_HIF0_PWR_CON		0x02e4	/* MT7622 */
#define SPM_HIF1_PWR_CON		0x02e8	/* MT7622 */
#define SPM_WB_PWR_CON			0x02ec	/* MT7622 */

#define SPM_PWR_STATUS			0x060c
#define SPM_PWR_STATUS_2ND		0x0610

#define PWR_RST_B_BIT			BIT(0)
#define PWR_ISO_BIT			BIT(1)
#define PWR_ON_BIT			BIT(2)
#define PWR_ON_2ND_BIT			BIT(3)
#define PWR_CLK_DIS_BIT			BIT(4)

#define PWR_STATUS_CONN			BIT(1)
#define PWR_STATUS_DISP			BIT(3)
#define PWR_STATUS_MFG			BIT(4)
#define PWR_STATUS_ISP			BIT(5)
#define PWR_STATUS_VDEC			BIT(7)
#define PWR_STATUS_BDP			BIT(14)
#define PWR_STATUS_ETH			BIT(15)
#define PWR_STATUS_HIF			BIT(16)
#define PWR_STATUS_IFR_MSC		BIT(17)
#define PWR_STATUS_USB2			BIT(19)	/* MT2712 */
#define PWR_STATUS_VENC_LT		BIT(20)
#define PWR_STATUS_VENC			BIT(21)
#define PWR_STATUS_MFG_2D		BIT(22)	/* MT8173 */
#define PWR_STATUS_MFG_ASYNC		BIT(23)	/* MT8173 */
#define PWR_STATUS_AUDIO		BIT(24)	/* MT8173, MT2712 */
#define PWR_STATUS_USB			BIT(25)	/* MT8173, MT2712 */
#define PWR_STATUS_ETHSYS		BIT(24)	/* MT7622 */
#define PWR_STATUS_HIF0			BIT(25)	/* MT7622 */
#define PWR_STATUS_HIF1			BIT(26)	/* MT7622 */
#define PWR_STATUS_WB			BIT(27)	/* MT7622 */

/**
 * struct scp_domain_data - scp domain data for power on/off flow
 * @name: The domain name.
 * @sta_mask: The mask for power on/off status bit.
 * @ctl_offs: The offset for main power control register.
 * @sram_pdn_bits: The mask for sram power control bits.
 * @sram_pdn_ack_bits: The mask for sram power control acked bits.
 * @bus_prot_mask: The mask for single step bus protection.
 * @clk_id: The basic clocks required by this power domain.
 * @caps: The flag for active wake-up action.
 */
struct scp_domain_data {
	u32 sta_mask;
	int ctl_offs;
	u32 sram_pdn_bits;
	u32 sram_pdn_ack_bits;
	u32 bus_prot_mask;
	u8 caps;
};

struct scp_domain {
	struct generic_pm_domain genpd;
	const struct scp_domain_data *data;
	struct scp *scp;
	int num_clks;
	struct clk_bulk_data *clks;
	// TODO regmap should go here I think
};

struct scp_soc_data {
	const struct scp_domain_data *domains;
	int num_domains;
	int pwr_sta_offs;
	int pwr_sta2nd_offs;
	bool bus_prot_reg_update;
};

struct scp {
	struct device *dev;
	void __iomem *base;
	struct regmap *infracfg;
	const struct scp_soc_data *soc_data;
	struct genpd_onecell_data pd_data;
	struct generic_pm_domain *domains[];
};

static int scpsys_domain_is_on(struct scp_domain *pd)
{
	struct scp *scp = pd->scp;

	u32 status = readl(scp->base + scp->soc_data->pwr_sta_offs) &
						pd->data->sta_mask;
	u32 status2 = readl(scp->base + scp->soc_data->pwr_sta2nd_offs) &
						pd->data->sta_mask;

	/*
	 * A domain is on when both status bits are set. If only one is set
	 * return an error. This happens while powering up a domain
	 */

	if (status && status2)
		return true;
	if (!status && !status2)
		return false;

	return -EINVAL;
}

static int scpsys_sram_enable(struct scp_domain *pd, void __iomem *ctl_addr)
{
	u32 val;
	u32 pdn_ack = pd->data->sram_pdn_ack_bits;
	int tmp;

	val = readl(ctl_addr);
	val &= ~pd->data->sram_pdn_bits;
	writel(val, ctl_addr);

	/* Either wait until SRAM_PDN_ACK all 0 or have a force wait */
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_FWAIT_SRAM)) {
		/*
		 * Currently, MTK_SCPD_FWAIT_SRAM is necessary only for
		 * MT7622_POWER_DOMAIN_WB and thus just a trivial setup
		 * is applied here.
		 */
		usleep_range(12000, 12100);
	} else {
		/* Either wait until SRAM_PDN_ACK all 1 or 0 */
		int ret = readl_poll_timeout(ctl_addr, tmp,
				(tmp & pdn_ack) == 0,
				MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int scpsys_sram_disable(struct scp_domain *pd, void __iomem *ctl_addr)
{
	u32 val;
	u32 pdn_ack = pd->data->sram_pdn_ack_bits;
	int tmp;

	val = readl(ctl_addr);
	val |= pd->data->sram_pdn_bits;
	writel(val, ctl_addr);

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	return readl_poll_timeout(ctl_addr, tmp,
			(tmp & pdn_ack) == pdn_ack,
			MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_enable(struct scp_domain *pd)
{
	struct scp *scp = pd->scp;

	if (!pd->data->bus_prot_mask)
		return 0;

	return mtk_infracfg_set_bus_protection(scp->infracfg,
			pd->data->bus_prot_mask,
			scp->soc_data->bus_prot_reg_update);
}

static int scpsys_bus_protect_disable(struct scp_domain *pd)
{
	struct scp *scp = pd->scp;

	if (!pd->data->bus_prot_mask)
		return 0;

	return mtk_infracfg_clear_bus_protection(scp->infracfg,
			pd->data->bus_prot_mask,
			scp->soc_data->bus_prot_reg_update);
}

static int scpsys_power_on(struct generic_pm_domain *genpd)
{
	struct scp_domain *pd = container_of(genpd, struct scp_domain, genpd);
	struct scp *scp = pd->scp;
	void __iomem *ctl_addr = scp->base + pd->data->ctl_offs;
	u32 val;
	int ret, tmp;

	ret = clk_bulk_enable(pd->num_clks, pd->clks);
	if (ret)
		goto err_clk;

	/* subsys power on */
	val = readl(ctl_addr);
	val |= PWR_ON_BIT;
	writel(val, ctl_addr);
	val |= PWR_ON_2ND_BIT;
	writel(val, ctl_addr);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp > 0,
				 MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		goto err_pwr_ack;

	val &= ~PWR_CLK_DIS_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ISO_BIT;
	writel(val, ctl_addr);

	val |= PWR_RST_B_BIT;
	writel(val, ctl_addr);

	ret = scpsys_sram_enable(pd, ctl_addr);
	if (ret < 0)
		goto err_pwr_ack;

	ret = scpsys_bus_protect_disable(pd);
	if (ret < 0)
		goto err_pwr_ack;

	// TODO can't we turn off the clocks here?

	return 0;

err_pwr_ack:
	clk_bulk_disable(pd->num_clks, pd->clks);
err_clk:
	dev_err(scp->dev, "Failed to power on domain %s\n", genpd->name);

	return ret;
}

static int scpsys_power_off(struct generic_pm_domain *genpd)
{
	struct scp_domain *pd = container_of(genpd, struct scp_domain, genpd);
	struct scp *scp = pd->scp;
	void __iomem *ctl_addr = scp->base + pd->data->ctl_offs;
	u32 val;
	int ret, tmp;

	ret = scpsys_bus_protect_enable(pd);
	if (ret < 0)
		goto out;

	ret = scpsys_sram_disable(pd, ctl_addr);
	if (ret < 0)
		goto out;

	/* subsys power off */
	val = readl(ctl_addr);
	val |= PWR_ISO_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_RST_B_BIT;
	writel(val, ctl_addr);

	val |= PWR_CLK_DIS_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ON_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ON_2ND_BIT;
	writel(val, ctl_addr);

	/* wait until PWR_ACK = 0 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp == 0,
				 MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		goto out;

	clk_bulk_disable(pd->num_clks, pd->clks);

	return 0;

out:
	dev_err(scp->dev, "Failed to power off domain %s\n", genpd->name);

	return ret;
}

//static void mtk_register_power_domains(struct platform_device *pdev,
//				struct scp *scp, int num)
static int scpsys_add_one_domain(struct scp *scp, struct device_node *node)
{
	const struct scp_domain_data *domain_data;
	struct scp_domain *pd;
	int i, ret;
	u32 id;

	ret = of_property_read_u32(node, "reg", &id);
	if (ret) {
		dev_err(scp->dev,
			"%pOFn: failed to retrive domain id from reg: %d\n",
			node, ret);
		return -EINVAL;
	}

	if (id >= scp->soc_data->num_domains) {
		dev_err(scp->dev, "%pOFn: invalid domain id %d\n",
			node, id);
		return -EINVAL;
	}
	
	
	domain_data = &scp->soc_data->domains[id];
	if (!domain_data) {
		dev_err(scp->dev, "%pOFn: undefined domain id %d\n",
			node, id);
		return -EINVAL;
	}
	
	pd = devm_kzalloc(scp->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->data = domain_data;
	pd->scp = scp;
	pd->num_clks = of_clk_get_parent_count(node);
	if (pd->num_clks > 0) {
		pd->clks = devm_kcalloc(scp->dev, pd->num_clks,
					sizeof(*pd->clks), GFP_KERNEL);
		if (!pd->clks)
			return -ENOMEM;
	} else {
		pd->num_clks = 0;
	}

	for (i = 0; i < pd->num_clks; i++) {
		pd->clks[i].clk = of_clk_get(node, i);
		if (IS_ERR(pd->clks[i].clk)) {
			ret = PTR_ERR(pd->clks[i].clk);
			dev_err(scp->dev,
				"%pOFn: failed to get clk at index %d: %d\n",
				node, i, ret);
			return ret;
		}
	}

	ret = clk_bulk_prepare(pd->num_clks, pd->clks);
	if (ret)
		goto error_put_clocks;

	// TODO get regmap here!

	/*
	 * Initially turn on all domains to make the domains usable
	 * with !CONFIG_PM and to get the hardware in sync with the
	 * software.  The unused domains will be switched off during
	 * late_init time.
	 */
	ret = scpsys_power_on(&pd->genpd);
	if (ret < 0) {
		dev_err(scp->dev,
			"failed to power on domain %pOFN with error %d\n",
			node, ret);
		goto error_unprepare_clocks;
	}


	pd->genpd.name = node->name;
	pd->genpd.power_off = scpsys_power_off;
	pd->genpd.power_on = scpsys_power_on;

	//TODO check if necessary
	// pd->genpd.flags = GENPD_FLAG_PM_CLK;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_ACTIVE_WAKEUP))
		pd->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;

	pm_genpd_init(&pd->genpd, NULL, false);

	scp->domains[id] = &pd->genpd;
	return 0;

error_unprepare_clocks:
	clk_bulk_unprepare(pd->num_clks, pd->clks);
error_put_clocks:
	clk_bulk_put(pd->num_clks, pd->clks);
	return ret;
}

static int scpsys_add_subdomain(struct scp *scp, struct device_node *parent)
{
	struct device_node *child;
	struct generic_pm_domain *child_pd, *parent_pd;
	int ret;

	for_each_child_of_node(parent, child) {
		u32 id;

		ret = of_property_read_u32(parent, "reg", &id);
		if (ret) {
			dev_err(scp->dev,
				"%pOFn: failed to get parent domain id: %d\n",
				child, ret);
			goto error;
		}
		parent_pd = scp->pd_data.domains[id];

		ret = scpsys_add_one_domain(scp, child);
		if (ret) {
			dev_err(scp->dev,
				"error adding power domain for %pOFn: %d\n",
				child, ret);
			goto error;
		}

		ret = of_property_read_u32(child, "reg", &id);
		if (ret) {
			dev_err(scp->dev,
				"%pOFn: failed to get child domain id: %d\n",
				child, ret);
			goto error;
		}
		child_pd = scp->pd_data.domains[id];

		ret = pm_genpd_add_subdomain(parent_pd, child_pd);
		if (ret) {
			dev_err(scp->dev, "failed to add %s subdomain to parent %s\n",
				child_pd->name, parent_pd->name);
			goto error;
		} else {
			dev_dbg(scp->dev, "%s add subdomain: %s\n",
				parent_pd->name, child_pd->name);
		}

		/* recursive call to add all subdomains */
		ret = scpsys_add_subdomain(scp, child);
		if (ret)
			goto error;
	}

	return 0;

error:
	of_node_put(child);
	return ret;
}

/*
 * MT8173 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt8173[] = {
	[MT8173_POWER_DOMAIN_VDEC] = {
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT8173_POWER_DOMAIN_VENC] = {
		.sta_mask = PWR_STATUS_VENC,
		.ctl_offs = SPM_VEN_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
	},
	[MT8173_POWER_DOMAIN_ISP] = {
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
	},
	[MT8173_POWER_DOMAIN_MM] = {
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bus_prot_mask = MT8173_TOP_AXI_PROT_EN_MM_M0 |
			MT8173_TOP_AXI_PROT_EN_MM_M1,
	},
	[MT8173_POWER_DOMAIN_VENC_LT] = {
		.sta_mask = PWR_STATUS_VENC_LT,
		.ctl_offs = SPM_VEN2_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
	},
	[MT8173_POWER_DOMAIN_AUDIO] = {
		.sta_mask = PWR_STATUS_AUDIO,
		.ctl_offs = SPM_AUDIO_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
	},
	[MT8173_POWER_DOMAIN_USB] = {
		.sta_mask = PWR_STATUS_USB,
		.ctl_offs = SPM_USB_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8173_POWER_DOMAIN_MFG_ASYNC] = {
		.sta_mask = PWR_STATUS_MFG_ASYNC,
		.ctl_offs = SPM_MFG_ASYNC_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = 0,
	},
	[MT8173_POWER_DOMAIN_MFG_2D] = {
		.sta_mask = PWR_STATUS_MFG_2D,
		.ctl_offs = SPM_MFG_2D_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
	},
	[MT8173_POWER_DOMAIN_MFG] = {
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(13, 8),
		.sram_pdn_ack_bits = GENMASK(21, 16),
		.bus_prot_mask = MT8173_TOP_AXI_PROT_EN_MFG_S |
			MT8173_TOP_AXI_PROT_EN_MFG_M0 |
			MT8173_TOP_AXI_PROT_EN_MFG_M1 |
			MT8173_TOP_AXI_PROT_EN_MFG_SNOOP_OUT,
	},
};

static const struct scp_soc_data mt8173_ng_data = {
	.domains = scp_domain_data_mt8173,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt8173),
	.pwr_sta_offs = SPM_PWR_STATUS,
	.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
	.bus_prot_reg_update = true,
};

/*
 * scpsys driver init
 */

static const struct of_device_id of_scpsys_ng_match_tbl[] = {
	{
		.compatible = "mediatek,mt8173-scpsys-ng",
		.data = &mt8173_ng_data,
	}, {
		/* sentinel */
	}
};

static int scpsys_ng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	const struct scp_soc_data *soc;
	struct scp *scp;
	struct resource *res;
	int ret;

	soc = of_device_get_match_data(&pdev->dev);
	if (!soc) {
		dev_err(&pdev->dev, "no power contoller data\n");
		return -EINVAL;
	}

	scp = devm_kzalloc(dev,
			   struct_size(scp, domains, soc->num_domains),
			   GFP_KERNEL);
	if (!scp)
		return -ENOMEM;

	scp->dev = dev;
	scp->soc_data = soc;

	scp->pd_data.domains = scp->domains;
	scp->pd_data.num_domains = soc->num_domains;

	/* get ressource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	scp->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(scp->base))
		return -ENODEV;

	/* get regmap. TODO do this on a per domain basis? */
	scp->infracfg = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"infracfg");
	if (IS_ERR(scp->infracfg)) {
		dev_err(&pdev->dev, "Cannot find infracfg controller: %ld\n",
				PTR_ERR(scp->infracfg));
		return -ENODEV;
	}

	ret = -ENODEV;

	for_each_available_child_of_node(np, node) {
		ret = scpsys_add_one_domain(scp, node);
		if (ret) {
			dev_err(dev, "failed to handle node %pOFN: %d\n",
				node, ret);
			of_node_put(node);
			goto error;
		}
		
		ret = scpsys_add_subdomain(scp, node);
		if (ret < 0) {
			dev_err(dev, "failed to add subdomain node %pOFn: %d\n",
				node, ret);
			of_node_put(node);
			goto error;
		}
	}

	if (ret) {
		dev_dbg(dev, "no power domains present\n");
		goto error;
	}

	ret = of_genpd_add_provider_onecell(np, &scp->pd_data);

	return 0;
error:
	//TODO cleanup?
	return ret;
}

static struct platform_driver scpsys_ng_drv = {
	.probe = scpsys_ng_probe,
	.driver = {
		.name = "mtk-scpsys-ng",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_scpsys_ng_match_tbl),
	},
};
builtin_platform_driver(scpsys_ng_drv);
