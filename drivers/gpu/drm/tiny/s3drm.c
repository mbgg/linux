/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * linux/drivers/gpu/drm/tiny/s3drm.c -- DRM driver for S3 Trio
 *
 * Copyright 2024 Marvin Friedrich <contact@marvinf.com>
 *
 * This code has been adapted from the fbdev version in
 * linux/drivers/video/fbdev/s3fb.c
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/svga.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <drm/drm_module.h>
#include <drm/drm_aperture.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode.h>
#include <drm/drm_gem.h>
#include <drm/drm_drv.h>
#include <drm/drm_simple_kms_helper.h>

#include <video/vga.h>

/* ------------------------------------------------------------------------- */
/* Defines                                                                   */

#define DRIVER_NAME		"s3drm"
#define DRIVER_DESC		"DRM driver for S3 Trio"
#define MMIO_OFFSET		0x1000000
#define MMIO_SIZE		0x10000

/* ------------------------------------------------------------------------- */
/* Module parameters                                                         */

static int s3drm_modeset = -1;
static char *mode_option;
static int mtrr = 1;
static int fasttext = 1;

module_param_named(modeset, s3drm_modeset, int, 0444);
MODULE_PARM_DESC(modeset, "Enable/disable kernel modesetting");

module_param(mode_option, charp, 0444);
MODULE_PARM_DESC(mode_option, "Default video mode ('640x480-8@60', etc)");

module_param_named(mode, mode_option, charp, 0444);
MODULE_PARM_DESC(mode, "Default video mode ('640x480-8@60', etc) (deprecated)");

module_param(mtrr, int, 0444);
MODULE_PARM_DESC(mtrr, "Enable write-combining with MTRR (1=enable, 0=disable, default=1)");

module_param(fasttext, int, 0644);
MODULE_PARM_DESC(fasttext, "Enable S3 fast text mode (1=enable, 0=disable, default=1)");

/* ------------------------------------------------------------------------- */
/* Device                                                                    */
#define CONFIG_DRM_S3_DDC

enum s3drm_chip {
	CHIP_UNKNOWN		= 0x00,
	CHIP_732_TRIO32		= 0x01,
	CHIP_764_TRIO64		= 0x02,
	CHIP_765_TRIO64VP	= 0x03,
	CHIP_767_TRIO64UVP	= 0x04,
	CHIP_775_TRIO64V2_DX	= 0x05,
	CHIP_785_TRIO64V2_GX	= 0x06,
	CHIP_551_PLATO_PX	= 0x07,
	CHIP_M65_AURORA64VP	= 0x08,
	CHIP_325_VIRGE		= 0x09,
	CHIP_988_VIRGE_VX	= 0x0A,
	CHIP_375_VIRGE_DX	= 0x0B,
	CHIP_385_VIRGE_GX	= 0x0C,
	CHIP_357_VIRGE_GX2	= 0x0D,
	CHIP_359_VIRGE_GX2P	= 0x0E,
	CHIP_360_TRIO3D_1X	= 0x10,
	CHIP_362_TRIO3D_2X	= 0x11,
	CHIP_368_TRIO3D_2X	= 0x12,
	CHIP_365_TRIO3D		= 0x13,
	CHIP_260_VIRGE_MX	= 0x14,

	CHIP_UNDECIDED_FLAG	= 0x80,

	CHIP_XXX_TRIO		= CHIP_UNDECIDED_FLAG | 0x00,
	CHIP_XXX_TRIO64V2_DXGX	= CHIP_UNDECIDED_FLAG | 0x01,
	CHIP_XXX_VIRGE_DXGX	= CHIP_UNDECIDED_FLAG | 0x02,
	CHIP_36X_TRIO3D_1X_2X	= CHIP_UNDECIDED_FLAG | 0x03,

	CHIP_MASK = 0xFF,
};

static const struct vga_regset s3_h_total_regs[]        = {{0x00, 0, 7}, {0x5D, 0, 0}, VGA_REGSET_END};
static const struct vga_regset s3_h_display_regs[]      = {{0x01, 0, 7}, {0x5D, 1, 1}, VGA_REGSET_END};
static const struct vga_regset s3_h_blank_start_regs[]  = {{0x02, 0, 7}, {0x5D, 2, 2}, VGA_REGSET_END};
static const struct vga_regset s3_h_blank_end_regs[]    = {{0x03, 0, 4}, {0x05, 7, 7}, VGA_REGSET_END};
static const struct vga_regset s3_h_sync_start_regs[]   = {{0x04, 0, 7}, {0x5D, 4, 4}, VGA_REGSET_END};
static const struct vga_regset s3_h_sync_end_regs[]     = {{0x05, 0, 4}, VGA_REGSET_END};

static const struct vga_regset s3_v_total_regs[]        = {{0x06, 0, 7}, {0x07, 0, 0}, {0x07, 5, 5}, {0x5E, 0, 0}, VGA_REGSET_END};
static const struct vga_regset s3_v_display_regs[]      = {{0x12, 0, 7}, {0x07, 1, 1}, {0x07, 6, 6}, {0x5E, 1, 1}, VGA_REGSET_END};
static const struct vga_regset s3_v_blank_start_regs[]  = {{0x15, 0, 7}, {0x07, 3, 3}, {0x09, 5, 5}, {0x5E, 2, 2}, VGA_REGSET_END};
static const struct vga_regset s3_v_blank_end_regs[]    = {{0x16, 0, 7}, VGA_REGSET_END};
static const struct vga_regset s3_v_sync_start_regs[]   = {{0x10, 0, 7}, {0x07, 2, 2}, {0x07, 7, 7}, {0x5E, 4, 4}, VGA_REGSET_END};
static const struct vga_regset s3_v_sync_end_regs[]     = {{0x11, 0, 3}, VGA_REGSET_END};

static const struct vga_regset s3_line_compare_regs[]   = {{0x18, 0, 7}, {0x07, 4, 4}, {0x09, 6, 6}, {0x5E, 6, 6}, VGA_REGSET_END};
static const struct vga_regset s3_start_address_regs[]  = {{0x0d, 0, 7}, {0x0c, 0, 7}, {0x69, 0, 4}, VGA_REGSET_END};
static const struct vga_regset s3_offset_regs[]         = {{0x13, 0, 7}, {0x51, 4, 5}, VGA_REGSET_END}; /* set 0x43 bit 2 to 0 */

static const struct vga_regset s3_dtpc_regs[]		= {{0x3B, 0, 7}, {0x5D, 6, 6}, VGA_REGSET_END};

static const struct svga_timing_regs s3_timing_regs     = {
	s3_h_total_regs, s3_h_display_regs, s3_h_blank_start_regs,
	s3_h_blank_end_regs, s3_h_sync_start_regs, s3_h_sync_end_regs,
	s3_v_total_regs, s3_v_display_regs, s3_v_blank_start_regs,
	s3_v_blank_end_regs, s3_v_sync_start_regs, s3_v_sync_end_regs,
};

static const char * const s3_names[] = {
	"S3 Unknown", "S3 Trio32", "S3 Trio64", "S3 Trio64V+",
	"S3 Trio64UV+", "S3 Trio64V2/DX", "S3 Trio64V2/GX",
	"S3 Plato/PX", "S3 Aurora64V+", "S3 Virge",
	"S3 Virge/VX", "S3 Virge/DX", "S3 Virge/GX",
	"S3 Virge/GX2", "S3 Virge/GX2+", "",
	"S3 Trio3D/1X", "S3 Trio3D/2X", "S3 Trio3D/2X",
	"S3 Trio3D", "S3 Virge/MX"
};

struct s3drm_device {
	enum s3drm_chip chip;
	int rev;
	int mclk_freq;
	int wc_cookie;
	struct vgastate state;
	struct mutex open_lock;
	unsigned int ref_count;
	u32 pseudo_palette[16];
#ifdef CONFIG_DRM_S3_DDC
	u8 __iomem *mmio;
	bool ddc_registered;
	struct i2c_adapter ddc_adapter;
	struct i2c_algo_bit_data ddc_algo;
#endif

	struct drm_device *dev;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
};

/* ------------------------------------------------------------------------- */
/* Utils                                                                     */

/* Identifies the chip. */
static int s3_identification(struct s3drm_device *par)
{
	int chip = par->chip;

	if (chip == CHIP_XXX_TRIO) {
		u8 cr30 = vga_rcrt(par->state.vgabase, 0x30);
		u8 cr2e = vga_rcrt(par->state.vgabase, 0x2e);
		u8 cr2f = vga_rcrt(par->state.vgabase, 0x2f);

		if ((cr30 == 0xE0) || (cr30 == 0xE1)) {
			if (cr2e == 0x10)
				return CHIP_732_TRIO32;
			if (cr2e == 0x11) {
				if (! (cr2f & 0x40))
					return CHIP_764_TRIO64;
				else
					return CHIP_765_TRIO64VP;
			}
		}
	}

	if (chip == CHIP_XXX_TRIO64V2_DXGX) {
		u8 cr6f = vga_rcrt(par->state.vgabase, 0x6f);

		if (! (cr6f & 0x01))
			return CHIP_775_TRIO64V2_DX;
		else
			return CHIP_785_TRIO64V2_GX;
	}

	if (chip == CHIP_XXX_VIRGE_DXGX) {
		u8 cr6f = vga_rcrt(par->state.vgabase, 0x6f);

		if (! (cr6f & 0x01))
			return CHIP_375_VIRGE_DX;
		else
			return CHIP_385_VIRGE_GX;
	}

	if (chip == CHIP_36X_TRIO3D_1X_2X) {
		switch (vga_rcrt(par->state.vgabase, 0x2f)) {
		case 0x00:
			return CHIP_360_TRIO3D_1X;
		case 0x01:
			return CHIP_362_TRIO3D_2X;
		case 0x02:
			return CHIP_368_TRIO3D_2X;
		}
	}

	return CHIP_UNKNOWN;
}

/* ------------------------------------------------------------------------- */
/* DDC                                                                       */

#ifdef CONFIG_DRM_S3_DDC

#define DDC_REG		0xaa		/* Trio 3D/1X/2X */
#define DDC_MMIO_REG	0xff20		/* all other chips */
#define DDC_SCL_OUT	(1 << 0)
#define DDC_SDA_OUT	(1 << 1)
#define DDC_SCL_IN	(1 << 2)
#define DDC_SDA_IN	(1 << 3)
#define DDC_DRIVE_EN	(1 << 4)

static bool s3drm_ddc_needs_mmio(int chip)
{
	return !(chip == CHIP_360_TRIO3D_1X  ||
		 chip == CHIP_362_TRIO3D_2X  ||
		 chip == CHIP_368_TRIO3D_2X);
}

static u8 s3drm_ddc_read(struct s3drm_device *par)
{
	if (s3drm_ddc_needs_mmio(par->chip))
		return readb(par->mmio + DDC_MMIO_REG);
	else
		return vga_rcrt(par->state.vgabase, DDC_REG);
}

static void s3drm_ddc_write(struct s3drm_device *par, u8 val)
{
	if (s3drm_ddc_needs_mmio(par->chip))
		writeb(val, par->mmio + DDC_MMIO_REG);
	else
		vga_wcrt(par->state.vgabase, DDC_REG, val);
}

static void s3drm_ddc_setscl(void *data, int val)
{
	struct s3drm_device *par = data;
	unsigned char reg;

	reg = s3drm_ddc_read(par) | DDC_DRIVE_EN;
	if (val)
		reg |= DDC_SCL_OUT;
	else
		reg &= ~DDC_SCL_OUT;
	s3drm_ddc_write(par, reg);
}

static void s3drm_ddc_setsda(void *data, int val)
{
	struct s3drm_device *par = data;
	unsigned char reg;

	reg = s3drm_ddc_read(par) | DDC_DRIVE_EN;
	if (val)
		reg |= DDC_SDA_OUT;
	else
		reg &= ~DDC_SDA_OUT;
	s3drm_ddc_write(par, reg);
}

static int s3drm_ddc_getscl(void *data)
{
	struct s3drm_device *par = data;

	return !!(s3drm_ddc_read(par) & DDC_SCL_IN);
}

static int s3drm_ddc_getsda(void *data)
{
	struct s3drm_device *par = data;

	return !!(s3drm_ddc_read(par) & DDC_SDA_IN);
}

// TODO
static int s3drm_setup_ddc_bus(struct s3drm_device *info)
{
	strscpy(info->ddc_adapter.name, info->fix.id,
		sizeof(info->ddc_adapter.name));
	info->ddc_adapter.owner		= THIS_MODULE;
	info->ddc_adapter.algo_data	= &info->ddc_algo;
	info->ddc_adapter.dev.parent	= info->dev->dev;
	info->ddc_algo.setsda		= s3drm_ddc_setsda;
	info->ddc_algo.setscl		= s3drm_ddc_setscl;
	info->ddc_algo.getsda		= s3drm_ddc_getsda;
	info->ddc_algo.getscl		= s3drm_ddc_getscl;
	info->ddc_algo.udelay		= 10;
	info->ddc_algo.timeout		= 20;
	info->ddc_algo.data		= info;

	i2c_set_adapdata(&info->ddc_adapter, info);

	/*
	 * some Virge cards have external MUX to switch chip I2C bus between
	 * DDC and extension pins - switch it do DDC
	 */
/*	vga_wseq(par->state.vgabase, 0x08, 0x06); - not needed, already unlocked */
	if (info->chip == CHIP_357_VIRGE_GX2 ||
	    info->chip == CHIP_359_VIRGE_GX2P ||
	    info->chip == CHIP_260_VIRGE_MX)
		svga_wseq_mask(info->state.vgabase, 0x0d, 0x01, 0x03);
	else
		svga_wseq_mask(info->state.vgabase, 0x0d, 0x00, 0x03);
	/* some Virge need this or the DDC is ignored */
	svga_wcrt_mask(info->state.vgabase, 0x5c, 0x03, 0x03);

	return i2c_bit_add_bus(&info->ddc_adapter);
}
#endif /* CONFIG_FB_S3_DDC */

/* ------------------------------------------------------------------------- */
/* DRM                                                                       */

// TODO
static void s3drm_plane_update(struct s3drm_device *dev, struct drm_plane_state *state)
{

}

// TODO
static void s3drm_pipe_enable(struct drm_simple_display_pipe *pipe,
		       struct drm_crtc_state *crtc_state,
		       struct drm_plane_state *plane_state)
{
	struct s3drm_device *s3drm = pipe->crtc.dev->dev_private;
	s3drm_plane_update(s3drm, pipe->plane.state);
}

static void s3drm_pipe_update(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *old_state)
{
	struct s3drm_device *dev = pipe->crtc.dev->dev_private;
	s3drm_plane_update(dev, pipe->plane.state);
}

// TODO
static const struct drm_simple_display_pipe_funcs s3drm_pipe_funcs = {
	.enable = s3drm_pipe_enable,
	.update = s3drm_pipe_update,
};

static const struct drm_driver s3drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= "20240412",
	.major			= 1,
	.minor			= 0,
};

/* ------------------------------------------------------------------------- */
/* PCI                                                                       */

// TODO
static int s3_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct pci_bus_region bus_reg;
	struct resource vga_res;
	struct drm_device *info;
	struct s3drm_device *par;
	int rc;
	u8 regval, cr38, cr39;
	bool found = false;

	/* Ignore secondary VGA device because there is no VGA arbitration */
	if (! svga_primary_device(dev)) {
		dev_info(&(dev->dev), "ignoring secondary device\n");
		return -ENODEV;
	}

	rc = drm_aperture_remove_conflicting_pci_framebuffers(dev, &s3drm_driver);
	if (rc)
		return rc;

	/* Allocate and fill driver data structure */
	info = drm_dev_alloc(&s3drm_driver, &(dev->dev));
	if (!info)
		return -ENOMEM;

	par = info->dev;
	mutex_init(&par->open_lock);

	info->flags = FBINFO_PARTIAL_PAN_OK | FBINFO_HWACCEL_YPAN;
	info->fbops = &s3drm_ops;

	/* Prepare PCI device */
	rc = pci_enable_device(dev);
	if (rc < 0) {
		dev_err(info->dev, "cannot enable PCI device\n");
		goto err_enable_device;
	}

	rc = pci_request_regions(dev, DRIVER_NAME);
	if (rc < 0) {
		dev_err(info->dev, "cannot reserve framebuffer region\n");
		goto err_request_regions;
	}


	info->fix.smem_start = pci_resource_start(dev, 0);
	info->fix.smem_len = pci_resource_len(dev, 0);

	/* Map physical IO memory address into kernel space */
	info->screen_base = pci_iomap_wc(dev, 0, 0);
	if (! info->screen_base) {
		rc = -ENOMEM;
		dev_err(info->device, "iomap for framebuffer failed\n");
		goto err_iomap;
	}

	bus_reg.start = 0;
	bus_reg.end = 64 * 1024;

	vga_res.flags = IORESOURCE_IO;

	pcibios_bus_to_resource(dev->bus, &vga_res, &bus_reg);

	par->state.vgabase = (void __iomem *) (unsigned long) vga_res.start;

	/* Unlock regs */
	cr38 = vga_rcrt(par->state.vgabase, 0x38);
	cr39 = vga_rcrt(par->state.vgabase, 0x39);
	vga_wseq(par->state.vgabase, 0x08, 0x06);
	vga_wcrt(par->state.vgabase, 0x38, 0x48);
	vga_wcrt(par->state.vgabase, 0x39, 0xA5);

	/* Identify chip type */
	par->chip = id->driver_data & CHIP_MASK;
	par->rev = vga_rcrt(par->state.vgabase, 0x2f);
	if (par->chip & CHIP_UNDECIDED_FLAG)
		par->chip = s3_identification(par);

	/* Find how many physical memory there is on card */
	/* 0x36 register is accessible even if other registers are locked */
	regval = vga_rcrt(par->state.vgabase, 0x36);
	if (par->chip == CHIP_360_TRIO3D_1X ||
	    par->chip == CHIP_362_TRIO3D_2X ||
	    par->chip == CHIP_368_TRIO3D_2X ||
	    par->chip == CHIP_365_TRIO3D) {
		switch ((regval & 0xE0) >> 5) {
		case 0: /* 8MB -- only 4MB usable for display */
		case 1: /* 4MB with 32-bit bus */
		case 2:	/* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 4: /* 2MB on 365 Trio3D */
		case 6: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		}
	} else if (par->chip == CHIP_357_VIRGE_GX2 ||
		   par->chip == CHIP_359_VIRGE_GX2P ||
		   par->chip == CHIP_260_VIRGE_MX) {
		switch ((regval & 0xC0) >> 6) {
		case 1: /* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 3: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		}
	} else if (par->chip == CHIP_988_VIRGE_VX) {
		switch ((regval & 0x60) >> 5) {
		case 0: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		case 1: /* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 2: /* 6MB */
			info->screen_size = 6 << 20;
			break;
		case 3: /* 8MB */
			info->screen_size = 8 << 20;
			break;
		}
		/* off-screen memory */
		regval = vga_rcrt(par->state.vgabase, 0x37);
		switch ((regval & 0x60) >> 5) {
		case 1: /* 4MB */
			info->screen_size -= 4 << 20;
			break;
		case 2: /* 2MB */
			info->screen_size -= 2 << 20;
			break;
		}
	} else
		info->screen_size = s3_memsizes[regval >> 5] << 10;
	info->fix.smem_len = info->screen_size;

	/* Find MCLK frequency */
	regval = vga_rseq(par->state.vgabase, 0x10);
	par->mclk_freq = ((vga_rseq(par->state.vgabase, 0x11) + 2) * 14318) / ((regval & 0x1F)  + 2);
	par->mclk_freq = par->mclk_freq >> (regval >> 5);

	/* Restore locks */
	vga_wcrt(par->state.vgabase, 0x38, cr38);
	vga_wcrt(par->state.vgabase, 0x39, cr39);

	strcpy(info->fix.id, s3_names [par->chip]);
	info->fix.mmio_start = 0;
	info->fix.mmio_len = 0;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	info->fix.ypanstep = 0;
	info->fix.accel = FB_ACCEL_NONE;
	info->pseudo_palette = (void*) (par->pseudo_palette);
	info->var.bits_per_pixel = 8;

#ifdef CONFIG_FB_S3_DDC
	/* Enable MMIO if needed */
	if (s3drm_ddc_needs_mmio(par->chip)) {
		par->mmio = ioremap(info->fix.smem_start + MMIO_OFFSET, MMIO_SIZE);
		if (par->mmio)
			svga_wcrt_mask(par->state.vgabase, 0x53, 0x08, 0x08);	/* enable MMIO */
		else
			dev_err(info->device, "unable to map MMIO at 0x%lx, disabling DDC",
				info->fix.smem_start + MMIO_OFFSET);
	}
	if (!s3drm_ddc_needs_mmio(par->chip) || par->mmio)
		if (s3drm_setup_ddc_bus(info) == 0) {
			u8 *edid = fb_ddc_read(&par->ddc_adapter);
			par->ddc_registered = true;
			if (edid) {
				fb_edid_to_monspecs(edid, &info->monspecs);
				kfree(edid);
				if (!info->monspecs.modedb)
					dev_err(info->device, "error getting mode database\n");
				else {
					const struct fb_videomode *m;

					fb_videomode_to_modelist(info->monspecs.modedb,
								 info->monspecs.modedb_len,
								 &info->modelist);
					m = fb_find_best_display(&info->monspecs, &info->modelist);
					if (m) {
						fb_videomode_to_var(&info->var, m);
						/* fill all other info->var's fields */
						if (s3drm_check_var(&info->var, info) == 0)
							found = true;
					}
				}
			}
		}
#endif
	if (!mode_option && !found)
		mode_option = "640x480-8@60";

	/* Prepare startup mode */
	if (mode_option) {
		rc = fb_find_mode(&info->var, info, mode_option,
				   info->monspecs.modedb, info->monspecs.modedb_len,
				   NULL, info->var.bits_per_pixel);
		if (!rc || rc == 4) {
			rc = -EINVAL;
			dev_err(info->device, "mode %s not found\n", mode_option);
			fb_destroy_modedb(info->monspecs.modedb);
			info->monspecs.modedb = NULL;
			goto err_find_mode;
		}
	}

	fb_destroy_modedb(info->monspecs.modedb);
	info->monspecs.modedb = NULL;

	/* maximize virtual vertical size for fast scrolling */
	info->var.yres_virtual = info->fix.smem_len * 8 /
			(info->var.bits_per_pixel * info->var.xres_virtual);
	if (info->var.yres_virtual < info->var.yres) {
		dev_err(info->device, "virtual vertical size smaller than real\n");
		rc = -EINVAL;
		goto err_find_mode;
	}

	rc = fb_alloc_cmap(&info->cmap, 256, 0);
	if (rc < 0) {
		dev_err(info->device, "cannot allocate colormap\n");
		goto err_alloc_cmap;
	}

	rc = drm_dev_register(info, 0);
	if (rc < 0) {
		dev_err(info->device, "cannot register framebuffer\n");
		goto err_reg_fb;
	}

	fb_info(info, "%s on %s, %d MB RAM, %d MHz MCLK\n",
		info->fix.id, pci_name(dev),
		info->fix.smem_len >> 20, (par->mclk_freq + 500) / 1000);

	if (par->chip == CHIP_UNKNOWN)
		fb_info(info, "unknown chip, CR2D=%x, CR2E=%x, CRT2F=%x, CRT30=%x\n",
			vga_rcrt(par->state.vgabase, 0x2d),
			vga_rcrt(par->state.vgabase, 0x2e),
			vga_rcrt(par->state.vgabase, 0x2f),
			vga_rcrt(par->state.vgabase, 0x30));

	/* Record a reference to the driver data */
	pci_set_drvdata(dev, info);

	if (mtrr)
		par->wc_cookie = arch_phys_wc_add(info->fix.smem_start,
						  info->fix.smem_len);

	return 0;

	/* Error handling */
err_reg_fb:
	fb_dealloc_cmap(&info->cmap);
err_alloc_cmap:
err_find_mode:
#ifdef CONFIG_FB_S3_DDC
	if (par->ddc_registered)
		i2c_del_adapter(&par->ddc_adapter);
	if (par->mmio)
		iounmap(par->mmio);
#endif
	pci_iounmap(dev, info->screen_base);
err_iomap:
	pci_release_regions(dev);
err_request_regions:
/*	pci_disable_device(dev); */
err_enable_device:
	framebuffer_release(info);
	return rc;
}

// TODO
static void s3_pci_remove(struct pci_dev *pdev)
{
	struct drm_driver *dev = pci_get_drvdata(pdev);

	if (dev) {
		par = info->par;
		arch_phys_wc_del(par->wc_cookie);
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);

#ifdef CONFIG_FB_S3_DDC
		if (par->ddc_registered)
			i2c_del_adapter(&par->ddc_adapter);
		if (par->mmio)
			iounmap(par->mmio);
#endif

		pci_iounmap(dev, info->screen_base);
		pci_release_regions(dev);
/*		pci_disable_device(dev); */

		framebuffer_release(info);
	}
}

static const struct pci_device_id s3_devices[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8810), .driver_data = CHIP_XXX_TRIO },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8811), .driver_data = CHIP_XXX_TRIO },
// TODO: Unsupported because untested.
/*
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8812), .driver_data = CHIP_M65_AURORA64VP },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8814), .driver_data = CHIP_767_TRIO64UVP },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8901), .driver_data = CHIP_XXX_TRIO64V2_DXGX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8902), .driver_data = CHIP_551_PLATO_PX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x5631), .driver_data = CHIP_325_VIRGE },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x883D), .driver_data = CHIP_988_VIRGE_VX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A01), .driver_data = CHIP_XXX_VIRGE_DXGX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A10), .driver_data = CHIP_357_VIRGE_GX2 },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A11), .driver_data = CHIP_359_VIRGE_GX2P },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A12), .driver_data = CHIP_359_VIRGE_GX2P },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A13), .driver_data = CHIP_36X_TRIO3D_1X_2X },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8904), .driver_data = CHIP_365_TRIO3D },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8C01), .driver_data = CHIP_260_VIRGE_MX },
*/
	{ /* End of list */ }
};

// TODO
static struct pci_driver s3drm_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= s3_devices,
	.probe		= s3_pci_probe,
	/*
	.remove		= s3_pci_remove,
	.driver.pm	= &s3_pci_pm_ops,
	*/
};

/* ------------------------------------------------------------------------- */
/* Module                                                                    */

#ifndef MODULE
/* Parse user specified options */
static int  __init s3drm_setup(char *options)
{
	char *opt;

	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {

		if (!*opt)
			continue;
		else if (!strncmp(opt, "mtrr:", 5))
			mtrr = simple_strtoul(opt + 5, NULL, 0);
		else if (!strncmp(opt, "fasttext:", 9))
			fasttext = simple_strtoul(opt + 9, NULL, 0);
		else
			mode_option = opt;
	}

	return 0;
}
#endif

// TODO
static int __init s3drm_init(void)
{
	if (s3drm_modeset == -1)
		return -ENODEV;

#ifndef MODULE
	char *option = NULL;
	if (fb_get_options(DRIVER_NAME, &option))
		return -ENODEV;
	s3drm_setup(option);
#endif

	pr_debug(DRIVER_NAME ": initializing\n");
	return pci_register_driver(&s3drm_pci_driver);
}

static void __exit s3drm_cleanup(void)
{
	pr_debug(DRIVER_NAME ": cleaning up\n");
	pci_unregister_driver(&s3drm_pci_driver);
}

/* ------------------------------------------------------------------------- */

drm_module_pci_driver_if_modeset(s3drm_pci_driver, s3drm_modeset);
module_init(s3drm_init);
module_exit(s3drm_cleanup);

MODULE_DEVICE_TABLE(pci, s3_devices);
MODULE_AUTHOR("Marvin Friedrich <contact@marvinf.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
