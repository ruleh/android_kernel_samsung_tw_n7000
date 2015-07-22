/* linux/drivers/video/samsung/s6e8aa1.c
 *
 * MIPI-DSI based AMS529HA01 AMOLED lcd panel driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *THIS FIXES BLACK CRUSH
 *However it isn't really complete. However that incompleteness is the fix.
 *
 *Not implemented:
 *--brightness settings
 *--correct calibration (instead we do this in mdnie)
 *--fancy stuff like pwm, cabc, auto-brightness.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-dsim.h>
#include <mach/dsim.h>
#include <mach/mipi_ddi.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "s5p-dsim.h"
#include "s3cfb.h"

#include "s6e8aa3_param.h"

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define MAX_GAMMA			300
#define DEFAULT_BRIGHTNESS		160
#define DEFAULT_GAMMA_LEVEL		0

#define LDI_ID_REG			0xD1
#define LDI_ID_LEN			3

#define LDI_MTP_ADDR			0xCB
#define LDI_MTP_LENGTH		63

extern void (*lcd_early_suspend)(void);
extern void (*lcd_late_resume)(void);


struct lcd_info {
	unsigned int			bl;
	unsigned int			auto_brightness;
	unsigned int			acl_enable;
	unsigned int			current_acl;
	unsigned int			current_bl;
	unsigned int			current_elvss;

	unsigned int			ldi_enable;
	unsigned int			power;
	struct mutex			lock;
	struct mutex			bl_lock;

	struct device			*dev;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct lcd_platform_data	*lcd_pd;
	struct early_suspend		early_suspend;

	unsigned char			id[LDI_ID_LEN];

	unsigned char			**gamma_table;
	unsigned char			**elvss_table;

	unsigned int			irq;
	unsigned int			connected;

	struct dsim_global		*dsim;
};

static const unsigned int candela_table[GAMMA_MAX] = {
	 30,  40,  50,  60,  70,  80,  90, 100, 110, 120,
	130, 140, 150, 160, 170, 180, 190, 200, 210, 220,
	230, 240, 250, MAX_GAMMA
};


static int s6e8ax0_write(struct lcd_info *lcd, const unsigned char *seq, int len)
{
	int size;
	const unsigned char *wbuf;

	if (!lcd->connected)
		return 0;

	mutex_lock(&lcd->lock);

	size = len;
	wbuf = seq;

	if (size == 1)
		lcd->dsim->ops->cmd_write(lcd->dsim, DCS_WR_NO_PARA, wbuf[0], 0);
	else if (size == 2)
		lcd->dsim->ops->cmd_write(lcd->dsim, DCS_WR_1_PARA, wbuf[0], wbuf[1]);
	else
		lcd->dsim->ops->cmd_write(lcd->dsim, DCS_LONG_WR, (unsigned int)wbuf, size);

	mutex_unlock(&lcd->lock);

	return 0;
}

static int _s6e8ax0_read(struct lcd_info *lcd, const u8 addr, u16 count, u8 *buf)
{
	int ret = 0;

	if (!lcd->connected)
		return ret;

	mutex_lock(&lcd->lock);

	if (lcd->dsim->ops->cmd_read)
		ret = lcd->dsim->ops->cmd_read(lcd->dsim, addr, count, buf);

	mutex_unlock(&lcd->lock);

	return ret;
}

static int s6e8ax0_read(struct lcd_info *lcd, const u8 addr, u16 count, u8 *buf, u8 retry_cnt)
{
	int ret = 0;

read_retry:
	ret = _s6e8ax0_read(lcd, addr, count, buf);
	if (!ret) {
		if (retry_cnt) {
			printk(KERN_WARNING "[WARN:LCD] %s : retry cnt : %d\n", __func__, retry_cnt);
			retry_cnt--;
			goto read_retry;
		} else
			printk(KERN_ERR "[ERROR:LCD] %s : 0x%02x read failed\n", __func__, addr);
	}

	return ret;
}
/**/
static int get_backlight_level_from_brightness(int brightness)
{
	int backlightlevel;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is only supported from 0 to 24 */

	switch (brightness) {
	case 0 ... 29:
		backlightlevel = GAMMA_30CD;
		break;
	case 30 ... 254:
		backlightlevel = (brightness - candela_table[0]) / 10;
		break;
	case 255:
		backlightlevel = ARRAY_SIZE(candela_table) - 1;
		break;
	default:
		backlightlevel = DEFAULT_GAMMA_LEVEL;
		break;
	}
	return backlightlevel;
}

static int s6e8ax0_gamma_ctl(struct lcd_info *lcd)
{
	s6e8ax0_write(lcd, lcd->gamma_table[lcd->bl], GAMMA_PARAM_SIZE);

	/* Gamma Set Update */
	s6e8ax0_write(lcd, SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));

	return 0;
}

static int s6e8ax0_set_acl(struct lcd_info *lcd, u8 force)
{
	int ret = 0, enable, level;
	u32 candela = candela_table[lcd->bl];

	switch (candela) {
	case 0 ... 49:
		level = ACL_STATUS_0P;
		break;
	case 50 ... 59:
		level = ACL_STATUS_20P;
		break;
	case 60 ... 69:
		level = ACL_STATUS_33P;
		break;
	case 70 ... 79:
		level = ACL_STATUS_43P;
		break;
	case 80 ... 89:
		level = ACL_STATUS_45P_80CD;
		break;
	case 90 ... 159:
		level = ACL_STATUS_45P;
		break;
	case 160 ... 169:
		level = ACL_STATUS_46P_160CD;
		break;
	case 170 ... 250:
		level = ACL_STATUS_46P;
		break;
	default:
		level = ACL_STATUS_50P;
		break;
	}

	if (!lcd->acl_enable)
		level = ACL_STATUS_0P;

	enable = !!level;

	//if (force || lcd->acl_enable != enable) {
		dev_dbg(&lcd->ld->dev, "acl turn %s\n", enable ? "on" : "off");
		if (enable)
			ret = s6e8ax0_write(lcd, SEQ_ACL_ON, ARRAY_SIZE(SEQ_ACL_ON));
		else {
			ret = s6e8ax0_write(lcd, SEQ_ACL_OFF, ARRAY_SIZE(SEQ_ACL_OFF));
			goto exit;
		}
	//}

	//if (force || lcd->current_acl != level) {
		ret = s6e8ax0_write(lcd, ACL_CUTOFF_TABLE[level], ACL_PARAM_SIZE);
		lcd->current_acl = level;
		dev_dbg(&lcd->ld->dev, "current_acl = %d\n", lcd->current_acl);
	//}

	if (ret)
		ret = -EPERM;

exit:
	return ret;
}


static int update_brightness(struct lcd_info *lcd, u8 force)
{
	u32 brightness;

	mutex_lock(&lcd->bl_lock);

	brightness = lcd->bd->props.brightness;

	if (unlikely(!lcd->auto_brightness && brightness > 250))
		brightness = 250;

	lcd->bl = get_backlight_level_from_brightness(brightness);

	if ((force) || ((lcd->ldi_enable) && (lcd->current_bl != lcd->bl))) {
		s6e8ax0_gamma_ctl(lcd);

#ifdef CONFIG_AID_DIMMING
		s6e8ax0_aid_parameter_ctl(lcd, force);
#endif
		s6e8ax0_set_acl(lcd, force);

		lcd->current_bl = lcd->bl;

		dev_info(&lcd->ld->dev, "brightness=%d, bl=%d, candela=%d\n", brightness, lcd->bl, candela_table[lcd->bl]);
	}

	mutex_unlock(&lcd->bl_lock);

	return 0;
}

/**/
static int s6e8ax0_ldi_init(struct lcd_info *lcd)
{
	int ret = 0;

	s6e8ax0_write(lcd, SEQ_APPLY_MTP_KEY_ENABLE, ARRAY_SIZE(SEQ_APPLY_MTP_KEY_ENABLE));
	s6e8ax0_write(lcd, SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	msleep(22);

	s6e8ax0_write(lcd, SEQ_PANEL_CONDITION_SET, ARRAY_SIZE(SEQ_PANEL_CONDITION_SET)); 
	s6e8ax0_write(lcd, SEQ_DISPLAY_CONDITION_SET, ARRAY_SIZE(SEQ_DISPLAY_CONDITION_SET));
	s6e8ax0_write(lcd, SEQ_GAMMA_CONDITION_SET, ARRAY_SIZE(SEQ_GAMMA_CONDITION_SET)); 
/*	s6e8ax0_write(lcd, SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE)); */

	s6e8ax0_write(lcd, SEQ_ETC_SOURCE_CONTROL, ARRAY_SIZE(SEQ_ETC_SOURCE_CONTROL));
	s6e8ax0_write(lcd, SEQ_ETC_PENTILE_CONTROL, ARRAY_SIZE(SEQ_ETC_PENTILE_CONTROL));
	s6e8ax0_write(lcd, SEQ_ELVSS_CONTROL, ARRAY_SIZE(SEQ_ELVSS_CONTROL));
	s6e8ax0_write(lcd, SEQ_ETC_NVM_SETTING, ARRAY_SIZE(SEQ_ETC_NVM_SETTING)); 

	return ret;
}

static int s6e8ax0_ldi_enable(struct lcd_info *lcd)
{
	int ret = 0;

	s6e8ax0_write(lcd, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));

	return ret;
}

static int s6e8ax0_ldi_disable(struct lcd_info *lcd)
{
	int ret = 0;

	s6e8ax0_write(lcd, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	s6e8ax0_write(lcd, SEQ_STANDBY_ON, ARRAY_SIZE(SEQ_STANDBY_ON));

	return ret;
}

static int s6e8ax0_power_on(struct lcd_info *lcd)
{
	int ret = 0;
	struct lcd_platform_data *pd = NULL;
	pd = lcd->lcd_pd;

	/* dev_info(&lcd->ld->dev, "%s\n", __func__); */

	ret = s6e8ax0_ldi_init(lcd);
	if (ret) {
		dev_err(&lcd->ld->dev, "failed to initialize ldi.\n");
		goto err;
	}

	msleep(120);

	ret = s6e8ax0_ldi_enable(lcd);
	if (ret) {
		dev_err(&lcd->ld->dev, "failed to enable ldi.\n");
		goto err;
	}

	lcd->ldi_enable = 1;

	/* update_brightness(lcd, 1); */
err:
	return ret;
}

static int s6e8ax0_power_off(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	lcd->ldi_enable = 0;

	ret = s6e8ax0_ldi_disable(lcd);

	msleep(135);

	return ret;
}

static int s6e8ax0_power(struct lcd_info *lcd, int power)
{
	int ret = 0;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		ret = s6e8ax0_power_on(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
		ret = s6e8ax0_power_off(lcd);

	if (!ret)
		lcd->power = power;

	return ret;
}

static int s6e8ax0_set_power(struct lcd_device *ld, int power)
{
	struct lcd_info *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(&lcd->ld->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return s6e8ax0_power(lcd, power);
}

static int s6e8ax0_get_power(struct lcd_device *ld)
{
	struct lcd_info *lcd = lcd_get_data(ld);

	return lcd->power;
}

static int s6e8ax0_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	int brightness = bd->props.brightness;
	struct lcd_info *lcd = bl_get_data(bd);

	/* dev_info(&lcd->ld->dev, "%s: brightness=%d\n", __func__, brightness); */

	if (brightness < MIN_BRIGHTNESS ||
		brightness > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d. now %d\n",
			MIN_BRIGHTNESS, MAX_BRIGHTNESS, brightness);
		return -EINVAL;
	}

	if (lcd->ldi_enable) {
		/* ret = update_brightness(lcd, 0); */
		if (ret < 0) {
			dev_err(lcd->dev, "err in %s\n", __func__);
			return -EINVAL;
		}
	}

	return ret;
}
/**/
static int s6e8ax0_check_fb(struct lcd_device *ld, struct fb_info *fb)
{
	struct s3cfb_window *win = fb->par;
	struct lcd_info *lcd = lcd_get_data(ld);

	dev_info(&lcd->ld->dev, "%s, fb%d\n", __func__, win->id);

	return 0;
}
/** this seems irrelevant. Maybe someone will use it to add brightness settings.**/
static int s6e8ax0_get_brightness(struct backlight_device *bd)
{
	struct lcd_info *lcd = bl_get_data(bd);

	return candela_table[lcd->bl];
}


static struct lcd_ops s6e8ax0_lcd_ops = {
	.set_power = s6e8ax0_set_power,
	.get_power = s6e8ax0_get_power,
	.check_fb  = s6e8ax0_check_fb,
};

static const struct backlight_ops s6e8ax0_backlight_ops  = {
	.get_brightness = s6e8ax0_get_brightness, /**relevence?**/
	.update_status = s6e8ax0_set_brightness,
};

static ssize_t power_reduce_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd->acl_enable);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t power_reduce_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->acl_enable != value) {
			dev_info(dev, "%s - %d, %d\n", __func__, lcd->acl_enable, value);
			mutex_lock(&lcd->bl_lock);
			lcd->acl_enable = value;
			/* if (lcd->ldi_enable)
				s6e8ax0_set_acl(lcd); */
			mutex_unlock(&lcd->bl_lock);
		}
	}
	return size;
}

static DEVICE_ATTR(power_reduce, 0664, power_reduce_show, power_reduce_store);
/**/
static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char temp[15];
	sprintf(temp, "SMD_AMS480GZ01-0\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);

static ssize_t auto_brightness_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd->auto_brightness);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t auto_brightness_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->auto_brightness != value) {
			dev_info(dev, "%s - %d, %d\n", __func__, lcd->auto_brightness, value);
			mutex_lock(&lcd->bl_lock);
			lcd->auto_brightness = value;
			mutex_unlock(&lcd->bl_lock);
			/* if (lcd->ldi_enable)
				update_brightness(lcd, 0); */
		}
	}
	return size;
}

static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);


struct lcd_info *g_lcd;

void s6e8ax0_early_suspend(void)
{
	struct lcd_info *lcd = g_lcd;

	set_dsim_lcd_enabled(0);

	dev_info(&lcd->ld->dev, "+%s\n", __func__);
	s6e8ax0_power(lcd, FB_BLANK_POWERDOWN);
	dev_info(&lcd->ld->dev, "-%s\n", __func__);

	return ;
}

void s6e8ax0_late_resume(void)
{
	struct lcd_info *lcd = g_lcd;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);
	s6e8ax0_power(lcd, FB_BLANK_UNBLANK);

	s3c_gpio_cfgpin(GPIO_OLED_DET, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_OLED_DET, S3C_GPIO_PULL_NONE);
	enable_irq(lcd->irq);

	dev_info(&lcd->ld->dev, "-%s\n", __func__);

	set_dsim_lcd_enabled(1);

	return ;
}

static int s6e8ax0_read_mtp(struct lcd_info *lcd, u8 *mtp_data)
{
	int ret, i;

	for (i = 0; i < 3; i++)
		ret = s6e8ax0_read(lcd, LDI_MTP_ADDR+i, LDI_MTP_LENGTH, mtp_data, 0);

	return ret;
}

static void s6e8ax0_read_id(struct lcd_info *lcd, u8 *buf)
{
	int ret = 0;

	ret = s6e8ax0_read(lcd, LDI_ID_REG, LDI_ID_LEN, buf, 3);
	if (!ret) {
		lcd->connected = 0;
		dev_info(&lcd->ld->dev, "panel is not connected well\n");
	}
}

static int s6e8ax0_probe(struct device *dev)
{
	int ret = 0;
	struct lcd_info *lcd;
	u8 mtp_data[LDI_MTP_LENGTH] = {0,};

	lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	g_lcd = lcd;

	lcd->ld = lcd_device_register("panel", dev, lcd, &s6e8ax0_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		pr_err("failed to register lcd device\n");
		ret = PTR_ERR(lcd->ld);
		goto out_free_lcd;
	}

	lcd->bd = backlight_device_register("panel", dev, lcd, &s6e8ax0_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("failed to register backlight device\n");
		ret = PTR_ERR(lcd->bd);
		goto out_free_backlight;
	}

	lcd->dev = dev;
	lcd->dsim = (struct dsim_global *)dev_get_drvdata(dev->parent);
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
	lcd->bl = DEFAULT_GAMMA_LEVEL;
	lcd->current_bl = lcd->bl;

	lcd->acl_enable = 0;
	lcd->current_acl = 0;

	lcd->power = FB_BLANK_UNBLANK;
	lcd->ldi_enable = 1;
	lcd->connected = 1;
	lcd->auto_brightness = 0;

	ret = device_create_file(&lcd->ld->dev, &dev_attr_power_reduce);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add sysfs entries, %d\n", __LINE__);

	ret = device_create_file(&lcd->ld->dev, &dev_attr_lcd_type);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add sysfs entries, %d\n", __LINE__);

/**cut line**/
/* I was thinking of adding this but I forgot how or why.
*	ret = device_create_file(&lcd->ld->dev, &dev_attr_read_acl);
*	if (ret < 0)
*		dev_err(&lcd->ld->dev, "failed to add sysfs entries, %d\n", __LINE__);
*/
/**cut line**/

	ret = device_create_file(&lcd->bd->dev, &dev_attr_auto_brightness);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add sysfs entries, %d\n", __LINE__);

	dev_set_drvdata(dev, lcd);

	mutex_init(&lcd->lock);
	mutex_init(&lcd->bl_lock);

	s6e8ax0_read_id(lcd, lcd->id);

	dev_info(&lcd->ld->dev, "ID: %x, %x, %x\n", lcd->id[0], lcd->id[1], lcd->id[2]);

	dev_info(&lcd->ld->dev, "lcd panel driver has been probed.\n");

	lcd_early_suspend = s6e8ax0_early_suspend;
	lcd_late_resume = s6e8ax0_late_resume;


	return 0;

out_free_backlight:
	lcd_device_unregister(lcd->ld);
	kfree(lcd);
	return ret;

out_free_lcd:
	kfree(lcd);
	return ret;

err_alloc:
	return ret;
}

static int __devexit s6e8ax0_remove(struct device *dev)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	s6e8ax0_power(lcd, FB_BLANK_POWERDOWN);
	lcd_device_unregister(lcd->ld);
	backlight_device_unregister(lcd->bd);
	kfree(lcd);

	return 0;
}

/* Power down all displays on reboot, poweroff or halt. */
static void s6e8ax0_shutdown(struct device *dev)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	s6e8ax0_power(lcd, FB_BLANK_POWERDOWN);
}

static struct mipi_lcd_driver s6e8ax0_mipi_driver = {
	.name = "s6e8aa0",
	.probe			= s6e8ax0_probe,
	.remove			= __devexit_p(s6e8ax0_remove),
	.shutdown		= s6e8ax0_shutdown,
};

static int s6e8ax0_init(void)
{
	return s5p_dsim_register_lcd_driver(&s6e8ax0_mipi_driver);
}

static void s6e8ax0_exit(void)
{
	return;
}

module_init(s6e8ax0_init);
module_exit(s6e8ax0_exit);

MODULE_DESCRIPTION("MIPI-DSI S6E8AA0:AMS529HA01 (800x1280) Panel Driver");
MODULE_LICENSE("GPL");
