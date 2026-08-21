// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 * Merged bridge+panel driver for imx93 — self-contained, no separate
 * panel-dpi DT node needed. Combines DSI host attach logic from the
 * original NXP waveshare-dsi.c bridge with the proven mode-table /
 * register-control model from panel-waveshare-dsi.c (sl1680, working).
 *
 * Based on panel-raspberrypi-touchscreen by Broadcom.
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

struct ws_bridge {
	struct drm_bridge bridge;
	struct backlight_device *backlight;
	struct device *dev;
	struct regmap *reg_map;
	const struct drm_display_mode *mode;
	int lanes;
	unsigned long mode_flags;
};

struct ws_bridge_data {
	const struct drm_display_mode *mode;
	int lanes;
	unsigned long mode_flags;
};

static const struct regmap_config ws_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

/* 7.0inch C 1024x600
 * https://www.waveshare.com/product/raspberry-pi/displays/lcd-oled/7inch-dsi-lcd-c-with-case-a.htm
 */
static const struct drm_display_mode ws_bridge_7_0_c_mode = {
	.clock = 50000,
	.hdisplay = 1024,
	.hsync_start = 1024 + 100,
	.hsync_end = 1024 + 100 + 100,
	.htotal = 1024 + 100 + 100 + 100,
	.vdisplay = 600,
	.vsync_start = 600 + 10,
	.vsync_end = 600 + 10 + 10,
	.vtotal = 600 + 10 + 10 + 10,
};

static const struct ws_bridge_data ws_bridge_7_0_c_data = {
	.mode = &ws_bridge_7_0_c_mode,
	.lanes = 2,
	.mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
};

/* Waveshare 7inch-C physical panel, 800x480 landscape orientation
 * (as measured/derived on this board: clock 28.03MHz,
 *  hfp 70 hbp 26 hsync 20, vfp 7 vbp 21 vsync 10)
 */
static const struct drm_display_mode ws_bridge_7_0_800x480_mode = {
	.clock = 28030,
	.hdisplay = 800,
	.hsync_start = 800 + 70,
	.hsync_end = 800 + 70 + 20,
	.htotal = 800 + 70 + 20 + 26,
	.vdisplay = 480,
	.vsync_start = 480 + 7,
	.vsync_end = 480 + 7 + 10,
	.vtotal = 480 + 7 + 10 + 21,
};

static const struct ws_bridge_data ws_bridge_7_0_800x480_data = {
	.mode = &ws_bridge_7_0_800x480_mode,
	.lanes = 2,
	.mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
};

/* 4.0inch 480x800
 * https://www.waveshare.com/product/raspberry-pi/displays/4inch-dsi-lcd.htm
 */
static const struct drm_display_mode ws_bridge_4_0_mode = {
	.clock = 50000,
	.hdisplay = 480,
	.hsync_start = 480 + 150,
	.hsync_end = 480 + 150 + 100,
	.htotal = 480 + 150 + 100 + 150,
	.vdisplay = 800,
	.vsync_start = 800 + 20,
	.vsync_end = 800 + 20 + 100,
	.vtotal = 800 + 20 + 100 + 20,
};

static const struct ws_bridge_data ws_bridge_4_0_data = {
	.mode = &ws_bridge_4_0_mode,
	.lanes = 2,
	.mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
};

static struct ws_bridge *bridge_to_ws_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ws_bridge, bridge);
}

static void ws_bridge_i2c_write(struct ws_bridge *ws, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(ws->reg_map, reg, val);
	if (ret)
		dev_err(ws->dev, "I2C write failed reg 0x%02x: %d\n", reg, ret);
}

static int ws_bridge_attach_dsi(struct ws_bridge *ws)
{
	const struct mipi_dsi_device_info info = {
		.type = "ws-bridge",
		.channel = 0,
		.node = NULL,
	};
	struct device_node *dsi_host_node;
	struct device *dev = ws->dev;
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_host *host;
	int ret;

	dsi_host_node = of_graph_get_remote_node(dev->of_node, 0, 0);
	if (!dsi_host_node) {
		dev_err(dev, "Failed to get remote port\n");
		return -ENODEV;
	}
	host = of_find_mipi_dsi_host_by_node(dsi_host_node);
	of_node_put(dsi_host_node);
	if (!host)
		return dev_err_probe(dev, -EPROBE_DEFER, "Failed to find dsi_host\n");

	dsi = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(dsi))
		return dev_err_probe(dev, PTR_ERR(dsi), "Failed to create dsi device\n");

	dsi->mode_flags = ws->mode_flags;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = ws->lanes;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to attach dsi to host\n");

	return 0;
}

static int ws_bridge_bridge_attach(struct drm_bridge *bridge,
				   struct drm_encoder *encoder,
				   enum drm_bridge_attach_flags flags)
{
	struct ws_bridge *ws = bridge_to_ws_bridge(bridge);

	return ws_bridge_attach_dsi(ws);
}

static void ws_bridge_bridge_enable(struct drm_bridge *bridge)
{
	struct ws_bridge *ws = bridge_to_ws_bridge(bridge);

	ws_bridge_i2c_write(ws, 0xad, 0x01);
	backlight_enable(ws->backlight);
}

static void ws_bridge_bridge_disable(struct drm_bridge *bridge)
{
	struct ws_bridge *ws = bridge_to_ws_bridge(bridge);

	backlight_disable(ws->backlight);
	ws_bridge_i2c_write(ws, 0xad, 0x00);
}

static int ws_bridge_bridge_get_modes(struct drm_bridge *bridge,
				      struct drm_connector *connector)
{
	static const u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct ws_bridge *ws = bridge_to_ws_bridge(bridge);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ws->mode);
	if (!mode) {
		dev_err(ws->dev, "failed to add mode %ux%u\n",
			ws->mode->hdisplay, ws->mode->vdisplay);
		return 0;
	}

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.bpc = 8;
	drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);

	return 1;
}

static const struct drm_bridge_funcs ws_bridge_bridge_funcs = {
	.attach = ws_bridge_bridge_attach,
	.enable = ws_bridge_bridge_enable,
	.disable = ws_bridge_bridge_disable,
	.get_modes = ws_bridge_bridge_get_modes,
};

static int ws_bridge_bl_update_status(struct backlight_device *bl)
{
	struct ws_bridge *ws = bl_get_data(bl);

	ws_bridge_i2c_write(ws, 0xab, 0xff - backlight_get_brightness(bl));
	ws_bridge_i2c_write(ws, 0xaa, 0x01);

	return 0;
}

static const struct backlight_ops ws_bridge_bl_ops = {
	.update_status = ws_bridge_bl_update_status,
};

static struct backlight_device *ws_bridge_create_backlight(struct ws_bridge *ws)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};
	struct device *dev = ws->dev;

	return devm_backlight_device_register(dev, dev_name(dev), dev, ws,
					      &ws_bridge_bl_ops, &props);
}

static int ws_bridge_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	const struct ws_bridge_data *data;
	struct ws_bridge *ws;
	int ret;

	ws = devm_drm_bridge_alloc(dev, struct ws_bridge, bridge, &ws_bridge_bridge_funcs);
	if (IS_ERR(ws))
		return PTR_ERR(ws);

	ws->dev = dev;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	ws->mode = data->mode;
	ws->lanes = data->lanes;
	ws->mode_flags = data->mode_flags;

	ws->reg_map = devm_regmap_init_i2c(i2c, &ws_regmap_config);
	if (IS_ERR(ws->reg_map))
		return dev_err_probe(dev, PTR_ERR(ws->reg_map), "Failed to allocate regmap\n");

	ws->backlight = ws_bridge_create_backlight(ws);
	if (IS_ERR(ws->backlight)) {
		ret = PTR_ERR(ws->backlight);
		dev_err(dev, "Failed to create backlight: %d\n", ret);
		return ret;
	}

	ws_bridge_i2c_write(ws, 0xc0, 0x01);
	ws_bridge_i2c_write(ws, 0xc2, 0x01);
	ws_bridge_i2c_write(ws, 0xac, 0x01);

	ws->bridge.type = DRM_MODE_CONNECTOR_DSI;
	ws->bridge.of_node = dev->of_node;
	devm_drm_bridge_add(dev, &ws->bridge);

	return 0;
}

static const struct of_device_id ws_bridge_of_ids[] = {
	{
		.compatible = "waveshare,7.0inch-c-panel",
		.data = &ws_bridge_7_0_c_data,
	}, {
		.compatible = "waveshare,7.0inch-800x480-panel",
		.data = &ws_bridge_7_0_800x480_data,
	}, {
		.compatible = "waveshare,4.0inch-panel",
		.data = &ws_bridge_4_0_data,
	},
	{ }
};

MODULE_DEVICE_TABLE(of, ws_bridge_of_ids);

static struct i2c_driver ws_bridge_driver = {
	.driver = {
		.name = "ws_dsi2dpi",
		.of_match_table = ws_bridge_of_ids,
	},
	.probe = ws_bridge_probe,
};
module_i2c_driver(ws_bridge_driver);

MODULE_AUTHOR("Joseph Guo <qijian.guo@nxp.com>");
MODULE_DESCRIPTION("Waveshare DSI panel/bridge combined driver for imx93");
MODULE_LICENSE("GPL");
