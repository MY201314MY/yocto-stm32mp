// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Amarula Solutions.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL		0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL		0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		    0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL		0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		    0xC2 /* Inversion selection, Frame Rate Control */
#define DSI_CMD2_BK0_PORCTRL_B0(m)	((m)->vtotal - (m)->vsync_end)
#define DSI_CMD2_BK0_PORCTRL_B1(m)	((m)->vsync_start - (m)->vdisplay)

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD		0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1		0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2		0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		0xC2 /* Source EQ2 Setting */
#define DSI_CMD2_BK1_MIPISET1		0xD0 /* MIPI Setting 1 */

/*
 * Command2 with BK function selection.
 *
 * BIT[4].....CN2
 * BIT[1:0]...BKXSEL
 * 1:00 = CMD2BK0, Command2 BK0
 * 1:01 = CMD2BK1, Command2 BK1
 * 1:11 = CMD2BK3, Command2 BK3
 * 0:00 = Command2 disable
 */
#define DSI_CMD2BK0_SEL			0x10
#define DSI_CMD2BK1_SEL			0x11
#define DSI_CMD2BK3_SEL			0x13
#define DSI_CMD2BKX_SEL_NONE    0x00

struct st7701_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned int panel_sleep_delay;
};

struct st7701 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct st7701_panel_desc *desc;

	struct gpio_desc *reset;
	unsigned int sleep_delay;
};

static inline struct st7701 *panel_to_st7701(struct drm_panel *panel)
{
	return container_of(panel, struct st7701, panel);
}

static inline int st7701_dsi_write(struct st7701 *st7701, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(st7701->dsi, seq, len);
}

#define ST7701_DSI(st7701, seq...)				\
	{							\
		const u8 d[] = { seq };				\
		st7701_dsi_write(st7701, d, ARRAY_SIZE(d));	\
	}

static void st7701_init_sequence(struct st7701 *st7701)
{
	const struct st7701_panel_desc *desc = st7701->desc;
	const struct drm_display_mode *mode = desc->mode;

	uint8_t rxbuffer[1] = {0};
	dev_info(&st7701->dsi->dev, "panel init sequence.");
	if(mipi_dsi_dcs_read(st7701->dsi, 0xDA, rxbuffer, 1))
	{
		dev_info(&st7701->dsi->dev, "ID1 : 0x%02X", rxbuffer[0]);
	}
	if(mipi_dsi_dcs_read(st7701->dsi, 0xDB, rxbuffer, 1))
	{
		dev_info(&st7701->dsi->dev, "ID2 : 0x%02X", rxbuffer[0]);
	}
	if(mipi_dsi_dcs_read(st7701->dsi, 0xDC, rxbuffer, 1))
	{
		dev_info(&st7701->dsi->dev, "ID3 : 0x%02X", rxbuffer[0]);
	}

	ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);

	/* We need to wait 5ms before sending new commands */
	msleep(5);

	ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);

	/* Command2, BK0 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BK0_SEL);

	ST7701_DSI(st7701, 0xC0, 0x63, 0x00);

	ST7701_DSI(st7701, DSI_CMD2_BK0_PORCTRL,
		   DSI_CMD2_BK0_PORCTRL_B0(mode),
		   DSI_CMD2_BK0_PORCTRL_B1(mode));

	ST7701_DSI(st7701, 0xC2, 0x37, 0x08);
	ST7701_DSI(st7701, 0xCC, 0x18);

	ST7701_DSI(st7701, DSI_CMD2_BK0_PVGAMCTRL, 0x40, 0xC9, 0x91, 0x0D,
		   0x12, 0x07, 0x02, 0x09, 0x09, 0x1F, 0x04, 0x50, 0x0F,
		   0xE4, 0x29, 0xDF);
	ST7701_DSI(st7701, DSI_CMD2_BK0_NVGAMCTRL, 0x40, 0xCB, 0xD0, 0x11,
		   0x92, 0x07, 0x00, 0x08, 0x07, 0x1C, 0x06, 0x53, 0x12,
		   0x63, 0xEB, 0xDF);
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, 0x11);
	
	ST7701_DSI(st7701, 0xB0, 0x65);
	ST7701_DSI(st7701, 0xB1, 0x6A);
	ST7701_DSI(st7701, 0xB2, 0x87);
	ST7701_DSI(st7701, 0xB3, 0x80);
	ST7701_DSI(st7701, 0xB5, 0x49);
	ST7701_DSI(st7701, 0xB7, 0x85);
	ST7701_DSI(st7701, 0xB8, 0x20);
	ST7701_DSI(st7701, 0xB9, 0x10);
	ST7701_DSI(st7701, 0xC1, 0x78);
	ST7701_DSI(st7701, 0xC2, 0x78);
	ST7701_DSI(st7701, 0xD0, 0x88);
	msleep(100);

	/**
	 * ST7701_SPEC_V1.2 is unable to provide enough information above this
	 * specific command sequence, so grab the same from vendor BSP driver.
	 */
	ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
	ST7701_DSI(st7701, 0xE1, 0x08, 0x00, 0x0A, 0x00, 0x07, 0x00, 0x09,
		   0x00, 0x00, 0x33, 0x33);
	ST7701_DSI(st7701, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
	ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
	ST7701_DSI(st7701, 0xE5, 0x0E, 0x60, 0xA0, 0xA0, 0x10, 0x60, 0xA0,
		   0xA0, 0x0A, 0x60, 0xA0, 0xA0, 0x0C, 0x60, 0xA0, 0xA0);
	ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
	ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
	ST7701_DSI(st7701, 0xE8, 0x0D, 0x60, 0xA0, 0xA0, 0x0F, 0x60, 0xA0,
		   0xA0, 0x09, 0x60, 0xA0, 0xA0, 0x0B, 0x60, 0xA0, 0xA0);
	ST7701_DSI(st7701, 0xEB, 0x02, 0x01, 0xE4, 0xE4, 0x44, 0x00, 0x40);
	ST7701_DSI(st7701, 0xEC, 0x02, 0x01);
	ST7701_DSI(st7701, 0xED, 0xAB, 0x89, 0x76, 0x54, 0x01, 0xFF, 0xFF,
		   0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x45, 0x67, 0x98, 0xBA);

	/* disable Command2 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE);
	ST7701_DSI(st7701, 0x36, 0x00);
	ST7701_DSI(st7701, 0x3A, 0x70);
	ST7701_DSI(st7701, 0x29);
	msleep(25);
}

static int st7701_prepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	gpiod_set_value(st7701->reset, 0);

	msleep(20);

	gpiod_set_value(st7701->reset, 1);
	msleep(150);

	gpiod_set_value_cansleep(st7701->reset, 0);
	msleep(50);

	st7701_init_sequence(st7701);

	/* Disable Command2 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE);

	return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON, 0x00);

	return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_OFF, 0x00);

	return 0;
}

static int st7701_unprepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	ST7701_DSI(st7701, MIPI_DCS_ENTER_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);

	gpiod_set_value(st7701->reset, 0);

	/**
	 * During the Resetting period, the display will be blanked
	 * (The display is entering blanking sequence, which maximum
	 * time is 120 ms, when Reset Starts in Sleep Out –mode. The
	 * display remains the blank state in Sleep In –mode.) and
	 * then return to Default condition for Hardware Reset.
	 *
	 * So we need wait sleep_delay time to make sure reset completed.
	 */
	msleep(st7701->sleep_delay);

	return 0;
}

static int st7701_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	const struct drm_display_mode *desc_mode = st7701->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		dev_err(&st7701->dsi->dev, "failed to add mode %ux%u@%u\n",
			desc_mode->hdisplay, desc_mode->vdisplay,
			drm_mode_vrefresh(desc_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = desc_mode->width_mm;
	connector->display_info.height_mm = desc_mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs st7701_funcs = {
	.disable	= st7701_disable,
	.unprepare	= st7701_unprepare,
	.prepare	= st7701_prepare,
	.enable		= st7701_enable,
	.get_modes	= st7701_get_modes,
};

static const struct drm_display_mode ts8550b_mode = {
	.clock		= 27056,

	.hdisplay	= 480,
	.hsync_start	= 480 + 50,
	.hsync_end	= 480 + 50 + 2,
	.htotal		= 480 + 50 + 2 + 10,

	.vdisplay	= 800,
	.vsync_start	= 800 + 20,
	.vsync_end	= 854 + 20 + 2,
	.vtotal		= 854 + 20 + 2 + 10,

	.width_mm	= 69,
	.height_mm	= 139,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct st7701_panel_desc ts8550b_desc = {
	.mode = &ts8550b_mode,
	.lanes = 2,
	.format = MIPI_DSI_FMT_RGB888,
	.panel_sleep_delay = 120, /* panel need extra 80ms for sleep out cmd */
};

static int st7701_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct st7701_panel_desc *desc;
	struct st7701 *st7701;
	int ret;

	st7701 = devm_kzalloc(&dsi->dev, sizeof(*st7701), GFP_KERNEL);
	if (!st7701)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	if (ret < 0)
		return ret;

	st7701->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st7701->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(st7701->reset);
	}

	dev_info(&dsi->dev, "sitronix st7701 panel.");
	drm_panel_init(&st7701->panel, &dsi->dev, &st7701_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	/**
	 * Once sleep out has been issued, ST7701 IC required to wait 120ms
	 * before initiating new commands.
	 *
	 * On top of that some panels might need an extra delay to wait, so
	 * add panel specific delay for those cases. As now this panel specific
	 * delay information is referenced from those panel BSP driver, example
	 * ts8550b and there is no valid documentation for that.
	 */
	st7701->sleep_delay = 120 + desc->panel_sleep_delay;

	ret = drm_panel_of_backlight(&st7701->panel);
	if (ret)
		return ret;

	drm_panel_add(&st7701->panel);

	mipi_dsi_set_drvdata(dsi, st7701);
	st7701->dsi = dsi;
	st7701->desc = desc;

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_attach;

	dev_info(&dsi->dev, "--- End ---");

	return 0;

err_attach:
	drm_panel_remove(&st7701->panel);
	return ret;
}

static void st7701_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct st7701 *st7701 = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&st7701->panel);
}

static const struct of_device_id st7701_of_match[] = {
	{ .compatible = "techstar,ts8550b", .data = &ts8550b_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, st7701_of_match);

static struct mipi_dsi_driver st7701_dsi_driver = {
	.probe		= st7701_dsi_probe,
	.remove		= st7701_dsi_remove,
	.driver = {
		.name		= "st7701",
		.of_match_table	= st7701_of_match,
	},
};
module_mipi_dsi_driver(st7701_dsi_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Sitronix ST7701 LCD Panel Driver");
MODULE_LICENSE("GPL");
