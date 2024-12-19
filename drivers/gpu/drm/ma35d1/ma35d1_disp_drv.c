// SPDX-License-Identifier: GPL-2.0+
/*
 * linux/drivers/gpu/drm/ma35d1/ma35d1_disp_drv.c
 *
 * Copyright (c) 2024 Nuvoton technology corporation.
 * Copyright (c) 2024 Nuvoton (Shanghai) corporation.
 *
 * Author: Jiang Tianwen  <twjiang@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/dma-map-ops.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "ma35d1_regs.h"

#define REG_CLK_CLKDIV0         (0x2C)

struct ma35d1_drm_private {
	struct drm_device			drm;
	struct drm_crtc				crtc;
	struct drm_pending_vblank_event		*event;
	struct drm_encoder			encoder;
	struct drm_connector			connector;
	struct drm_plane			plane[2];
	void __iomem				*reg_base;
	struct clk				*dcuclk;
	struct clk				*pixclk;
	struct reset_control			*rst;
	struct drm_display_mode			display_mode;
	u32 					bus_flags;
	u32					dpiConfig;
	u32					bits_per_pixel;
	u32					pixel_fmt;
	u32					swizzle;
};

static inline struct ma35d1_drm_private * to_private_drm_device(struct drm_device *dev)
{
	 return container_of(dev, struct ma35d1_drm_private, drm);
}

static int ma35d1_gem_cma_dumb_create(struct drm_file *file, struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), 128);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	return drm_gem_cma_dumb_create_internal(file, dev, args);
}

DEFINE_DRM_GEM_CMA_FOPS(ma35d1_drm_gem_cma_fops);

static struct drm_driver ma35d1_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name            = "ma35d1_kms",
	.desc            = "MA35D1 display DRM/KMS driver",
	.date            = "20241213",
	.major           = 1,
	.minor           = 0,
	.patchlevel      = 0,
	.fops            = &ma35d1_drm_gem_cma_fops,
	DRM_GEM_CMA_DRIVER_OPS_WITH_DUMB_CREATE(ma35d1_gem_cma_dumb_create),
};

static const struct of_device_id ma35d1_drm_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-drm" },
	{ }
};

MODULE_DEVICE_TABLE(of, ma35d1_drm_dt_ids);

static irqreturn_t ma35d1_drm_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct ma35d1_drm_private *priv = to_private_drm_device(drm);
	u32 status;
	unsigned long flags;

	status = readl(priv->reg_base + REG_dcregDisplayIntr);

	if (status & 0x1) {
		
		drm_crtc_handle_vblank(&priv->crtc); 

		spin_lock_irqsave(&drm->event_lock, flags);
		if (priv->event) {
			drm_crtc_send_vblank_event(&priv->crtc, priv->event);
			drm_crtc_vblank_put(&priv->crtc);
			priv->event = NULL;
		}
		spin_unlock_irqrestore(&drm->event_lock, flags);
		
		return IRQ_HANDLED;
	}
	
	return IRQ_NONE;
}

static const struct drm_mode_config_funcs ma35d1_drm_mode_config_funcs = {
	.fb_create           = drm_gem_fb_create,
	.output_poll_changed = drm_fb_helper_output_poll_changed,
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = drm_atomic_helper_commit,
};

void ma35d1_drm_mode_config_helper_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

static const struct drm_mode_config_helper_funcs ma35d1_drm_mode_config_helpers_funcs = {
	.atomic_commit_tail = ma35d1_drm_mode_config_helper_atomic_commit_tail,
};

bool ma35d1_drm_plane_format_mod_supported(struct drm_plane *plane, uint32_t format, uint64_t modifier)
{
	return modifier == DRM_FORMAT_MOD_LINEAR;
}

static const struct drm_plane_funcs ma35d1_drm_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.format_mod_supported	= ma35d1_drm_plane_format_mod_supported,
};

static int ma35d1_drm_plane_helper_atomic_check(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct drm_framebuffer *fb;
	struct device *dev = plane->dev->dev;

	if (!state->fb || !state->crtc) {
		return 0;
	}

	fb = state->fb;

	switch (plane->type) {
		case DRM_PLANE_TYPE_PRIMARY:
			return 0;
		case DRM_PLANE_TYPE_OVERLAY:
			return 0;
		case DRM_PLANE_TYPE_CURSOR:
			return 0;
		default:
			break;
	}

	if ((fb->flags & DRM_MODE_FB_MODIFIERS) &&
	    ! plane->funcs->format_mod_supported(plane, fb->format->format, fb->modifier)) {
		dev_err(dev, "Invalid modifier: %llx", fb->modifier);
		return -EINVAL;
	}

	return 0;
}

static void ma35d1_drm_plane_helper_atomic_disable(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	if (!old_state || !old_state->crtc) {
		return;
	}
}

static void ma35d1_drm_plane_helper_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct drm_device *dev = plane->dev;
	struct ma35d1_drm_private *priv = to_private_drm_device(dev);
	dma_addr_t scanout_start;
	
	if (!state->fb || !state->crtc) {
		return;
	}

	scanout_start = drm_fb_cma_get_gem_addr(state->fb, state, 0);
	writel(scanout_start, priv->reg_base + REG_dcregFrameBufferAddress0);
}

static const struct drm_plane_helper_funcs ma35d1_drm_plane_helper_funcs = {
	.prepare_fb     = drm_gem_fb_prepare_fb,
	.atomic_check   = ma35d1_drm_plane_helper_atomic_check,
	.atomic_update  = ma35d1_drm_plane_helper_atomic_update,
	.atomic_disable = ma35d1_drm_plane_helper_atomic_disable,
};

static const uint32_t drm_plane_formats[] = {
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
};

enum drm_cursor_format {
 	DRM_CURSOR_FORMAT_DISABLED,
	DRM_CURSOR_FORMAT_MASKED,
	DRM_CURSOR_FORMAT_ARGB8888
};

static const uint32_t drm_overlay_plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const uint64_t drm_plane_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

int ma35d1_drm_plane_create(struct drm_device *dev, struct drm_plane *plane, enum drm_plane_type type)
{
	int ret;
	
	ret = drm_universal_plane_init(dev, plane, 1,
			&ma35d1_drm_plane_funcs,
			drm_plane_formats,
			ARRAY_SIZE(drm_plane_formats),
			NULL,
			type,
			NULL);
	
	if (ret != 0) {
		return ret;
	}

	drm_plane_helper_add(plane, &ma35d1_drm_plane_helper_funcs);

	return 0;
}

static int ma35d1_drm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct ma35d1_drm_private *priv = container_of(crtc->dev, struct ma35d1_drm_private, drm);
	struct drm_crtc_state *state = crtc->state;

	if (state->enable) {
		writel(1, priv->reg_base + REG_dcregDisplayIntrEnable);
	} else {
		WARN_ON(1);
		return -EAGAIN;
	}

	return 0;
}

static void ma35d1_drm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct ma35d1_drm_private *priv = container_of(crtc->dev, struct ma35d1_drm_private, drm);
	writel(0, priv->reg_base + REG_dcregDisplayIntrEnable);
}

static u32 ma35d1_drm_crtc_get_vblank_counter(struct drm_crtc *crtc)
{
	struct ma35d1_drm_private *priv = container_of(crtc->dev, struct ma35d1_drm_private, drm);
	u32 counter;

	writel(5, priv->reg_base + REG_dcregDebugCounterSelect0);
	counter = readl(priv->reg_base + REG_dcregDebugCounterValue0);

	return counter;
}

static const struct drm_crtc_funcs ma35d1_drm_crtc_funcs = {
	.reset                  = drm_atomic_helper_crtc_reset,
	.destroy                = drm_crtc_cleanup,
	.set_config             = drm_atomic_helper_set_config,
	.page_flip              = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_crtc_destroy_state,
	.get_vblank_counter     = ma35d1_drm_crtc_get_vblank_counter,
	.enable_vblank          = ma35d1_drm_crtc_enable_vblank,
	.disable_vblank         = ma35d1_drm_crtc_disable_vblank,
	.get_vblank_timestamp   = drm_crtc_vblank_helper_get_vblank_timestamp,
};

static void ma35d1_drm_crtc_helper_atomic_begin(struct drm_crtc *crtc, struct drm_crtc_state *old_s)
{
	struct ma35d1_drm_private *priv = container_of(crtc->dev, struct ma35d1_drm_private, drm);

	if (crtc->state->event) {
		crtc->state->event->pipe = drm_crtc_index(crtc);

		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		priv->event = crtc->state->event;
		crtc->state->event = NULL;
	}
}

static int ma35d1_drm_crtc_helper_atomic_check(struct drm_crtc *crtc, struct drm_crtc_state *s)
{
	return 0;
}

static enum drm_mode_status ma35d1_drm_crtc_helper_mode_valid(struct drm_crtc *c, const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void ma35d1_drm_crtc_helper_atomic_flush(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{

}

static void ma35d1_drm_crtc_helper_mode_set_nofb(struct drm_crtc *c)
{

}

static void ma35d1_drm_crtc_helper_atomic_enable(struct drm_crtc *c, struct drm_crtc_state *old_state)
{
	struct drm_display_mode *mode = &c->state->adjusted_mode;
	struct drm_display_mode *old_mode = &old_state->adjusted_mode;
	struct videomode vm;
	int vrefresh;
	u32 data, paddr;
	u32 h_end, h_total, hsync_start, hsync_end;
	u32 v_end, v_total, vsync_start, vsync_end;
	struct ma35d1_drm_private *priv = container_of(c->dev, struct ma35d1_drm_private, drm);

	drm_display_mode_to_videomode(mode, &vm);
	vm.pixelclock = mode->crtc_clock * 1000;

	h_end = vm.hactive;
	h_total = vm.hactive + vm.hfront_porch + vm.hback_porch + vm.hsync_len;
	hsync_start = vm.hactive + vm.hfront_porch;
	hsync_end = hsync_start + vm.hsync_len;

	v_end = vm.vactive;
	v_total = vm.vactive + vm.vfront_porch + vm.vback_porch + vm.vsync_len;
	vsync_start = vm.vactive + vm.vfront_porch;
	vsync_end = vsync_start + vm.vsync_len;

	if (!drm_mode_equal(mode, old_mode) || !old_state->active) {
			
		data = (h_total << HSYNC_TOTAL_POS) | h_end;
		writel(data, priv->reg_base + REG_dcregHDisplay0);

		data = HSYNC_PULSE_POLARITY_POS | HSYNC_PULSE_ENABLE | (hsync_end << HSYNC_PULSE_END_POS) | (hsync_start);
		writel(data, priv->reg_base + REG_dcregHSync0);

		data = (v_total << VSYNC_TOTAL_POS) | v_end;
		writel(data, priv->reg_base + REG_dcregVDisplay0);

		data = VSYNC_PULSE_POLARITY_POS | VSYNC_PULSE_ENABLE | (vsync_end << VSYNC_PULSE_END_POS) | (vsync_start);
		writel(data, priv->reg_base + REG_dcregVSync0);

		data = priv->dpiConfig;
		writel(data, priv->reg_base + REG_dcregDpiConfig0);

		data = PANELCONFIG_DE_DATA_ENABLE | PANELCONFIG_DE_POLARITY_POS |
			PANELCONFIG_DATA_ENABLE | PANELCONFIG_DATA_POLARITY_POS |
			PANELCONFIG_CLOCK_ENABLE | PANELCONFIG_CLOCK_POLARITY_POS;
		writel(data, priv->reg_base + REG_dcregPanelConfig0);

		data = (vm.vactive << FRAMEBUFFER_HEIGHT_POS) | (vm.hactive);
		writel(data, priv->reg_base + REG_dcregFrameBufferSize0);

		data = vm.hactive * (priv->bits_per_pixel >> 3);
		writel(data, priv->reg_base + REG_dcregFrameBufferStride0);

		data = (priv->pixel_fmt << FRAMEBUFFER_FORMAT_POS) | (priv->swizzle << 23) | YUV_709_BT709 | RESET_ENABLE | OUTPUT_ENABLE;
		writel(data, priv->reg_base + REG_dcregFrameBufferConfig0);

	}

	drm_crtc_vblank_on(c);
}

static void ma35d1_drm_crtc_helper_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	struct drm_pending_vblank_event *event;
	
	drm_crtc_vblank_off(crtc);

	if (!crtc->state->active) {
		event = crtc->state->event;
		crtc->state->event = NULL;
		if (event) {
			spin_lock_irq(&crtc->dev->event_lock);
			drm_crtc_send_vblank_event(crtc, event);
			spin_unlock_irq(&crtc->dev->event_lock);
		}
	}
}

bool ma35d1_drm_crtc_helper_get_scanout_position(struct drm_crtc *crtc,
	bool in_vblank_irq, int *vpos, int *hpos,
	ktime_t *stime, ktime_t *etime,
	const struct drm_display_mode *mode)
{
	bool ret = true;
	u32 cur_loc;
	struct ma35d1_drm_private *priv = to_private_drm_device(crtc->dev);
	
	/* Get optional system timestamp before query. */
	if (stime)
		*stime = ktime_get();

	cur_loc = readl(priv->reg_base + REG_dcregDisplayCurrentLocation0);

	*vpos = ((cur_loc & 0xFFFF0000) >> 16);
	*hpos = (cur_loc & 0x0000FFFF);

	/* Get optional system timestamp after query. */
	if (etime)
		*etime = ktime_get();

	/* preempt_enable_rt() should go right here in PREEMPT_RT patchset. */

	return ret;
}

static const struct drm_crtc_helper_funcs ma35d1_drm_crtc_helper_funcs = {
	.mode_valid           = ma35d1_drm_crtc_helper_mode_valid,
	.mode_set_nofb        = ma35d1_drm_crtc_helper_mode_set_nofb,
	.atomic_check         = ma35d1_drm_crtc_helper_atomic_check,
	.atomic_begin         = ma35d1_drm_crtc_helper_atomic_begin,
	.atomic_flush         = ma35d1_drm_crtc_helper_atomic_flush,
	.atomic_enable        = ma35d1_drm_crtc_helper_atomic_enable,
	.atomic_disable       = ma35d1_drm_crtc_helper_atomic_disable,
	.get_scanout_position = ma35d1_drm_crtc_helper_get_scanout_position,
};

const uint32_t DivideFactor[] = {2, 4, 6, 8, 10, 12, 14, 16};
u32 clock_target, dcpdiv;
u32 retclock[1]= {0};

static u32 calc_pixclk_div_params(u32 dcupsrcFreg, struct regmap *regmap, u32 *TargetFreq)
{
	u32 val, div_val, retFreq = 0, FactorIdx, tmpFreq;

	for (FactorIdx = 0; FactorIdx < (sizeof(DivideFactor)/4); FactorIdx++) {
		tmpFreq = dcupsrcFreg / DivideFactor[FactorIdx];
		if (tmpFreq > TargetFreq[0])
			continue;
		else if (tmpFreq < TargetFreq[0]) {
			if (tmpFreq > retFreq) {
				retFreq = tmpFreq;
				div_val = FactorIdx;
			}
        	} else {
			retFreq = tmpFreq;
			div_val = FactorIdx;
			break;
		}
	}

	TargetFreq[0] = retFreq;
	regmap_read(regmap, REG_CLK_CLKDIV0, &val);
	val = (val &~(0x7<<16)) | (div_val<<16);
	regmap_write(regmap, REG_CLK_CLKDIV0, div_val);
	regmap_read(regmap, REG_CLK_CLKDIV0, &val);

	return div_val;
}

static int ma35d1_drm_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	struct device_node *np = drm->dev->of_node;
	struct ma35d1_drm_private *priv = container_of(drm, struct ma35d1_drm_private, drm);
	int num_modes;

	if (np) {
		/* fallback to display-timings node */
		struct drm_display_mode *mode = drm_mode_create(connector->dev);
		int ret;

		if (!mode) {
			return -EINVAL;
		}

		ret = of_get_drm_display_mode(np, &priv->display_mode, &priv->bus_flags, OF_USE_NATIVE_MODE);
		if (ret) {
			drm_mode_destroy(connector->dev, mode);
			return ret;
		}

		drm_mode_copy(mode, &priv->display_mode);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		drm_mode_probed_add(connector, mode);
		num_modes++;
	}

	return num_modes;
}

static const struct drm_connector_helper_funcs ma35d1_drm_connector_helper_funcs = {
	.get_modes = ma35d1_drm_connector_get_modes,
};

static const struct drm_connector_funcs ma35d1_drm_connector_funcs = {
	.fill_modes             = drm_helper_probe_single_connector_modes,
	.destroy                = drm_connector_cleanup,
	.reset                  = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_encoder_helper_funcs ma35d1_drm_encoder_helper_funcs = {
	.disable	= 0,
	.enable		= 0,
	.mode_valid	= 0,
};

static int ma35d1_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(ma35d1_drm_dt_ids, dev);
	int ret;
	struct ma35d1_drm_private *priv;
	struct drm_device *drm;
	struct drm_connector *connector;
	struct resource res; 
	struct device_node *np, *mp;
	struct regmap *regmap;
	struct display_timings *disp_timing;
	struct videomode vm;

#if 0
	if (of_device_get_match_data(dev) == NULL)
		return -EINVAL;
#endif

	if (!dev->of_node) {
		dev_err(dev, "Failed to get device tree node\n");
		return -ENODEV;
	}

	disp_timing = of_get_display_timings(dev->of_node);
	if (!disp_timing) {
		dev_err(dev, "Failed to get display timings\n");
		return -EINVAL;
	}

	ret = videomode_from_timings(disp_timing, &vm, disp_timing->native_mode);
	if (ret) {
		dev_err(&pdev->dev, "videomode_from_timings() failed: %d\n",ret);
		return -EINVAL;
	}

	/* gulpixclkHz = vm.pixelclock; */

	priv = devm_drm_dev_alloc(dev, &ma35d1_drm_driver, struct ma35d1_drm_private, drm);
	if (IS_ERR(priv)) {
		dev_err(dev, "failed to alloc a drm device with devm_drm_dev_alloc()\n");
		return PTR_ERR(priv);
	}

	drm = &priv->drm;

	ret = of_property_read_u32(dev->of_node, "bits-per-pixel", &priv->bits_per_pixel);

	ret = of_property_read_u32(dev->of_node, "pixel-fmt", &priv->pixel_fmt);

	ret = of_property_read_u32(dev->of_node, "swizzle", &priv->swizzle);

	ret = of_property_read_u32(dev->of_node, "dpi-config", &priv->dpiConfig);

	priv->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg_base)) {
		dev_err(dev, "failed to get register address\n");
		return PTR_ERR(priv->reg_base);
	}

	mp = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (mp) {
		ret = of_address_to_resource(mp, 0, &res);
		if (ret) {
			dev_err(&pdev->dev, "of_address_to_resource() failed!\n");
			return ret;
		}
		of_node_put(mp);

		np = of_find_compatible_node(NULL, NULL, "shared-dma-pool");
		if (!np) {
			ret = dma_declare_coherent_memory(&pdev->dev, res.start, res.start, resource_size(&res));
			if (ret == 0) {
				dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
			} else {
				return -ENOMEM;
			}	
		} else {
			of_node_put(np);
			ret = of_reserved_mem_device_init(dev);
			if (ret && ret != -ENODEV) {
				dev_err(dev, "Couldn't claim display memory region\n");
				return ret;
			}
		}
	}

	/* DCUltra Core Clock */
	priv->dcuclk = devm_clk_get(dev, "dcu_gate");
	if (IS_ERR(priv->dcuclk)) {
		dev_err(dev, "Failed to get display core clock\n");
		return PTR_ERR(priv->dcuclk);
	}

	ret = clk_prepare_enable(priv->dcuclk);
	if (ret) {
		dev_err(dev, "Failed to enable display core clock\n");
		return ret;
	}

	dev_info(dev, "dcu_clk = epll/2 = %ld Hz\n", clk_get_rate(priv->dcuclk));

	/* DCUltra Display Pixel Clock */
	np = of_parse_phandle(dev->of_node, "nuvoton,clk", 0);
	if (np) {
		regmap = syscon_node_to_regmap(np);
		of_node_put(np);
		if (IS_ERR(regmap)) {
			dev_err(dev, "Failed to get regmap\n");
			return PTR_ERR(regmap);
		}
	}

	priv->pixclk = devm_clk_get(dev, "dcup_div");
	if (IS_ERR(priv->pixclk)) {
		dev_err(dev, "Failed to get display pixel clock\n");
		return PTR_ERR(priv->pixclk);
	}

	dev_info(dev, "DCUltra Display Pixel Clock: %ld Hz\n", clk_get_rate(priv->pixclk) * 2);

	clock_target = clk_get_rate(priv->pixclk) * 2;
	retclock[0] = vm.pixelclock;
	dcpdiv = calc_pixclk_div_params(clock_target, regmap, retclock);
	clk_set_rate(priv->pixclk, retclock[0]);
	clk_prepare_enable(priv->pixclk);
	if (IS_ERR(priv->pixclk)) {
		dev_err(dev, "Failed to enable display pixel clock\n");
		return -ENOENT;
	}

	priv->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_info(&pdev->dev, "no reset control found\n");
		ret = PTR_ERR(priv->rst);
		goto reserved_memory_fail;
	}
	ret = reset_control_assert(priv->rst);
	udelay(100);
	ret = reset_control_deassert(priv->rst);

#if 0
	printk("chip id: %u\n", readl(priv->reg_base + REG_GCChipId));
	printk("chip revision: %u\n", readl(priv->reg_base + REG_GCChipRev));
	printk("chip patch revision: %u\n", readl(priv->reg_base + REG_gcregHICHIPatchRev));
	printk("chip info: %u\n", readl(priv->reg_base + REG_AQxiConfig));
	printk("product id: %u\n", readl(priv->reg_base + REG_GCProductId));
	printk("eco id: %u\n", readl(priv->reg_base + REG_GCECOId));
	printk("customer id: %u\n", readl(priv->reg_base + REG_GCCustomerId));
	printk("chip date: %u\n", readl(priv->reg_base + REG_GCChipDate));
	printk("chip time: %u\n", readl(priv->reg_base + REG_GCChipTime));
#endif

	writel(RESET_ENABLE, priv->reg_base + REG_dcregFrameBufferConfig0);
	udelay(10);
	writel(RESET_DISABLE, priv->reg_base + REG_dcregFrameBufferConfig0);

	drm->mode_config.min_width = 16;
	drm->mode_config.min_height = 16;
	drm->mode_config.max_width = 1920;
	drm->mode_config.max_height = 1080;
	drm->mode_config.allow_fb_modifiers = true;
	drm->mode_config.normalize_zpos = true;
	drm->mode_config.funcs = &ma35d1_drm_mode_config_funcs;
	drm->mode_config.helper_private = &ma35d1_drm_mode_config_helpers_funcs;
	
	ret = drmm_mode_config_init(drm);
	if (ret ) {
		dev_err(dev, "drmm_mode_config_init() failed\n");
		return ret;
	}

	ret = drm_vblank_init(drm, 1);
	if (ret) {
		dev_err(dev, "drm_vblank_init() failed\n");
		return ret;
	}
	
	/*
	 * CRTC init
	 */
	ma35d1_drm_plane_create(drm, &priv->plane[0], DRM_PLANE_TYPE_PRIMARY);
	ma35d1_drm_plane_create(drm, &priv->plane[1], DRM_PLANE_TYPE_CURSOR);

	ret = drm_crtc_init_with_planes(drm, &priv->crtc,
		&priv->plane[0],       /* primary */
		&priv->plane[1],       /* cursor  */
		&ma35d1_drm_crtc_funcs,
		NULL);
	
	if (ret != 0) {
		dev_err(dev, "ma35d1_drm_probe() failed to initialize crtc with planes\n");
		return ret;
	}

	drm_crtc_helper_add(&priv->crtc, &ma35d1_drm_crtc_helper_funcs);

	priv->encoder.possible_crtcs = 1;
	ret = drm_simple_encoder_init(drm, &priv->encoder, DRM_MODE_ENCODER_NONE);
	if (ret) {
		dev_err(dev, "Couldn't initialise the encoder\n");
		return ret;
	}

#if 0
	drm_encoder_helper_add(&priv->encoder, &ma35d1_drm_encoder_helper_funcs);
#endif

	connector = &priv->connector;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm, connector, &ma35d1_drm_connector_funcs, DRM_MODE_CONNECTOR_Unknown);
	if (ret) {
		dev_err(dev, "failed to initialize connector with drm\n");
		drm_encoder_cleanup(&priv->encoder);
		return ret;
	}

	drm_connector_helper_add(connector, &ma35d1_drm_connector_helper_funcs);

	ret = drm_connector_attach_encoder(connector, &priv->encoder);
	if (ret) {
		dev_err(dev, "failed to attach connector and encoder\n");
		drm_connector_cleanup(connector);
		return ret;
	}

	pm_runtime_get_sync(dev);
	ret = devm_request_irq(dev, platform_get_irq(pdev, 0), ma35d1_drm_irq_handler, 0, "ma35d1 drm", drm);
	pm_runtime_put_sync(dev);
	
	if (ret) {
		dev_err(dev, "failed to request IRQ%u: %d\n", platform_get_irq(pdev, 0), ret);
		return ret;
	}

	platform_set_drvdata(pdev, drm);

	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err(dev, "drm_dev_register() failed\n");
		return ret;
	}

	drm_fbdev_generic_setup(drm, 32);

	return 0;

reserved_memory_fail:
	of_reserved_mem_device_release(dev);

	return ret;
}

static int ma35d1_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void ma35d1_drm_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = dev_get_drvdata(&pdev->dev);

	if (!drm)
		return;

	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
}

static struct platform_driver ma35d1_display_platform_driver = {
	.probe		= ma35d1_drm_probe,
	.remove		= ma35d1_drm_remove,
	/* .shutdown 	= ma35d1_drm_shutdown, */
	.driver	= {
		.name		= "ma35d1_drm",
		.of_match_table	= ma35d1_drm_dt_ids,
	},
};

module_platform_driver(ma35d1_display_platform_driver);

MODULE_AUTHOR("Nuvoton Corporation <twjiang@nuvoton.com>");
MODULE_DESCRIPTION("MA35D1 DRM/KMS driver");
MODULE_LICENSE("GPL");

