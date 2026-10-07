// SPDX-License-Identifier: GPL-2.0-only
/*
 * xpg_fusion.c — hwmon driver for the XPG Fusion 1600W Titanium PSU
 * (Delta Electronics, GPS-1600AB, VID 0x125F / PID 0x7900).
 *
 * The device is a virtual HID with no endpoints (bNumEndpoints=0); all I/O
 * goes over control transfers on EP0: HID Feature Report ID 7, exactly a
 * 257-byte buffer.
 *
 * Protocol:
 *   request:  SET_REPORT  bmReq=0x21 bReq=0x09 wValue=0x0307 len=257
 *             [0]=0x07 [1]=0x00 [2]=command [3..254]=0 [255]=PEC
 *   response: GET_REPORT  bmReq=0xA1 bReq=0x01 wValue=0x0307 wLength=257
 *             [0]=0x07 [1]=0x00 [2]=echo [3..254]=data [255]=PEC
 *   PEC: CRC-8 (PMBus) over bytes 0..254. Without a correct PEC the MCU
 *        silently drops the command (USB ACK, all-zero report) — we validate
 *        the echo + PEC.
 *
 * Read-only commands only: 0xDF (FULLPACKAGE), 0x96 (POUT), 0x99/0x9A/0xDC (MFR).
 * Nothing is written to the device.
 *
 * Tested: kernel 7.0.0-34-generic (Ubuntu 26.04).
 */

#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#define DRIVER_NAME	"xpg_fusion"
#define XPG_VID		0x125F
#define XPG_PID		0x7900

#define REPORT_LEN	257		/* 1 (Report ID) + 256 (data) */
#define W_VALUE		0x0307		/* Feature Report, Report ID 7 */
#define PEC_INDEX	255

#define BM_SET_REPORT	0x21		/* OUT | Standard | Device */
#define BM_GET_REPORT	0xA1		/* IN  | Standard | Device */
#define BREQ_SET_REPORT	0x09
#define BREQ_GET_REPORT	0x01

#define CMD_FULLPACKAGE	0xDF
#define CMD_READ_POUT	0x96
#define CMD_MFR_ID	0x99
#define CMD_MFR_MODEL	0x9A
#define CMD_MFR_FW	0xDC

#define XPG_TIMEOUT_MS	2000
#define DELAY_AFTER_SET_MS 50
#define DELAY_AFTER_GET_MS 30

#define INTERVAL_MIN_SEC	1
#define INTERVAL_MAX_SEC	60
#define INTERVAL_DEF_SEC	1

static int interval_sec = INTERVAL_DEF_SEC;
module_param(interval_sec, int, 0444);
MODULE_PARM_DESC(interval_sec, "Poll interval in seconds (default 1, range 1..60)");

static const u8 pec_hi_crc[16] = {
	0x00, 0x70, 0xE0, 0x90, 0xC7, 0xB7, 0x27, 0x57,
	0x89, 0xF9, 0x69, 0x19, 0x4E, 0x3E, 0xAE, 0xDE,
};
static const u8 pec_lo_crc[16] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
};

static u8 pec8(u8 pec, u8 data)
{
	pec ^= data;
	return pec_hi_crc[pec >> 4] ^ pec_lo_crc[pec & 0x0F];
}

static u8 report_pec(const u8 *buf)
{
	u8 p = 0;
	int i;

	for (i = 0; i < PEC_INDEX; i++)
		p = pec8(p, buf[i]);
	return p;
}

/* --- Linear11 (Delta packing) -----------
 * Returns X * mult in integer units (mV/mA/m°C/µW/RPM).
 */
static s64 l11_scaled(u8 lo, u8 hi, s64 mult)
{
	int n = (hi & 0x78) >> 3;
	s64 m, v;

	if (hi & 0x80)
		n -= 16;
	m = lo + (s64)(hi & 0x03) * 256;
	if (hi & 0x04)
		m -= 1024;
	v = m * mult;
	if (n >= 0)
		return v << n;
	if (v >= 0)
		return (v + (1 << -n) / 2) >> -n;
	return -((-v + (1 << -n) / 2) >> -n);
}

/* --- Device state ------------------------------------------------------------ */

struct xpg_priv {
	struct usb_interface *intf;
	struct usb_device *udev;
	struct delayed_work poll;
	struct mutex io_lock;
	bool stopping;
	u8 buf[REPORT_LEN];
	bool valid;
	s64 vin_mv;
	s64 vout_mv;
	s64 v5_mv;
	s64 v33_mv;
	s64 vn12_mv;		/* |-12V|, magnitude is positive (label "-12V") */
	s64 iin_ma;
	s64 rails_ma[10];	/* 12V1..6, -12V, +5V, +3.3V, +12V total */
	s64 pin_uw;
	s64 pout_uw;
	s64 temp_millic;
	s64 fan_rpm;
	s64 eff_pct;
	char mfr[64];
	char model[64];
	char fw[40];
};

static int xpg_interval_ms(void)
{
	int ms = clamp(interval_sec, INTERVAL_MIN_SEC, INTERVAL_MAX_SEC) * 1000;

	return ms;
}

static void xpg_sanity_check(struct xpg_priv *priv, struct device *dev)
{
#define XPG_WARN_IF(cond, fmt, ...) do {		\
	if (cond)					\
		dev_warn_ratelimited(dev, fmt, ##__VA_ARGS__); \
} while (0)

	XPG_WARN_IF(priv->vin_mv < 80000 || priv->vin_mv > 300000,
		    "VIN out of range: %lld mV\n", priv->vin_mv);
	XPG_WARN_IF(priv->vout_mv < 11000 || priv->vout_mv > 13500,
		    "VOUT out of range: %lld mV\n", priv->vout_mv);
	XPG_WARN_IF(priv->temp_millic < -10000 || priv->temp_millic > 90000,
		    "TEMP out of range: %lld mC\n", priv->temp_millic);
	XPG_WARN_IF(priv->fan_rpm < 0 || priv->fan_rpm > 3000,
		    "FAN out of range: %lld RPM\n", priv->fan_rpm);
	XPG_WARN_IF(priv->pout_uw < 0 || priv->pout_uw > 2000000000LL,
		    "POUT out of range: %lld uW\n", priv->pout_uw);
#undef XPG_WARN_IF
}

/* --- Transfer ---------------------------------------------------------------- */

/* One full cycle: SET(cmd) -> 50ms -> GET -> 30ms -> validation.
 * The response is left in priv->buf. Must be called with io_lock held. */
static int xpg_transfer(struct xpg_priv *priv, u8 cmd)
{
	u8 *b = priv->buf;
	int rc;

	memset(b, 0, REPORT_LEN);
	b[0] = 0x07;
	b[1] = 0x00;
	b[2] = cmd;
	b[PEC_INDEX] = report_pec(b);

	rc = usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
			     BREQ_SET_REPORT, BM_SET_REPORT,
			     W_VALUE, 0, b, REPORT_LEN, XPG_TIMEOUT_MS);
	if (rc < 0)
		return rc;
	msleep(DELAY_AFTER_SET_MS);

	rc = usb_control_msg(priv->udev, usb_rcvctrlpipe(priv->udev, 0),
			     BREQ_GET_REPORT, BM_GET_REPORT,
			     W_VALUE, 0, b, REPORT_LEN, XPG_TIMEOUT_MS);
	if (rc != REPORT_LEN)
		return rc < 0 ? rc : -EIO;
	msleep(DELAY_AFTER_GET_MS);

	/* The MCU silently drops commands with a bad PEC / unknown command */
	if (b[2] != cmd || report_pec(b) != b[PEC_INDEX])
		return -EPROTO;
	return 0;
}

static void xpg_decode_full(struct xpg_priv *priv, const u8 *b)
{
	priv->vin_mv = l11_scaled(b[47], b[48], 1000);
	priv->vout_mv = l11_scaled(b[49], b[50], 1000);
	priv->v5_mv = l11_scaled(b[17], b[18], 1000);
	priv->v33_mv = l11_scaled(b[19], b[20], 1000);
	priv->vn12_mv = l11_scaled(b[15], b[16], 1000);
	if (priv->vn12_mv < 0)
		priv->vn12_mv = -priv->vn12_mv;

	priv->iin_ma = l11_scaled(b[51], b[52], 1000);
	priv->rails_ma[0] = l11_scaled(b[25], b[26], 1000);	/* 12V1 */
	priv->rails_ma[1] = l11_scaled(b[27], b[28], 1000);	/* 12V2 */
	priv->rails_ma[2] = l11_scaled(b[29], b[30], 1000);	/* 12V3 */
	priv->rails_ma[3] = l11_scaled(b[31], b[32], 1000);	/* 12V4 */
	priv->rails_ma[4] = l11_scaled(b[33], b[34], 1000);	/* 12V5 */
	priv->rails_ma[5] = l11_scaled(b[35], b[36], 1000);	/* 12V6 */
	priv->rails_ma[6] = l11_scaled(b[37], b[38], 1000);	/* -12V I */
	priv->rails_ma[7] = l11_scaled(b[39], b[40], 1000);	/* +5V I */
	priv->rails_ma[8] = l11_scaled(b[41], b[42], 1000);	/* +3.3V I */
	priv->rails_ma[9] = l11_scaled(b[43], b[44], 1000);	/* +12V I */

	priv->pin_uw = l11_scaled(b[55], b[56], 1000000);
	priv->temp_millic = l11_scaled(b[76], b[77], 1000);
	priv->fan_rpm = l11_scaled(b[94], b[95], 1);
	priv->eff_pct = l11_scaled(b[112], b[113], 1);
}

/* FULLPACKAGE + POUT, cache is up to date. Must be called with io_lock held. */
static int xpg_refresh(struct xpg_priv *priv)
{
	int rc;

	rc = xpg_transfer(priv, CMD_FULLPACKAGE);
	if (rc)
		return rc;
	xpg_decode_full(priv, priv->buf);
	xpg_sanity_check(priv, &priv->intf->dev);

	/* POUT is fresh from a separate command */
	rc = xpg_transfer(priv, CMD_READ_POUT);
	if (rc)
		return rc;
	priv->pout_uw = l11_scaled(priv->buf[3], priv->buf[4], 1000000);

	return 0;
}

/* --- Periodic polling ---------------------------------------------------------- */

static void xpg_poll(struct work_struct *ws)
{
	struct xpg_priv *priv = container_of(to_delayed_work(ws),
					     struct xpg_priv, poll);
	int rc;

	mutex_lock(&priv->io_lock);
	rc = xpg_refresh(priv);
	if (rc) {
		if (priv->valid)
			dev_err(&priv->intf->dev,
				"PSU contact lost (error %d)\n", rc);
		priv->valid = false;
	} else {
		if (!priv->valid)
			dev_info(&priv->intf->dev, "PSU contact restored\n");
		priv->valid = true;
	}

	/* Re-arm under the same io_lock that disconnect/suspend set
	 * stopping on. By the time cancel_delayed_work_sync runs the timer
	 * is either already pending (and will be removed) or the poll never
	 * re-armed: this rules out the race in which a sync cancel would
	 * return with a pending timer after priv is freed (UAF). */
	if (!priv->stopping)
		schedule_delayed_work(&priv->poll,
				      msecs_to_jiffies(xpg_interval_ms()));
	mutex_unlock(&priv->io_lock);
}

/* --- hwmon ------------------------------------------------------------------- */

static const char *xpg_in_labels[] = {
	"AC Input", "+12V", "+5V", "+3.3V", "-12V"
};
static const char *xpg_curr_labels[] = {
	"IIN", "+12V1", "+12V2", "+12V3", "+12V4", "+12V5", "+12V6",
	"-12V", "+5V", "+3.3V", "+12V"
};
static const char *xpg_power_labels[] = {
	"DC Output", "AC Input"
};

static int xpg_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct xpg_priv *priv = dev_get_drvdata(dev);
	int rc;

	/* Cache access is under io_lock (fields are plain, no own lock).
	 * Freshness is maintained by the background poll; on a lost
	 * contact (or before the first successful poll) reads report an
	 * error instead of serving stale values. hwmon serializes its own
	 * readers with its own mutex; reads here hold the lock briefly. */
	mutex_lock(&priv->io_lock);
	if (priv->stopping || !priv->valid) {
		mutex_unlock(&priv->io_lock);
		return -ESHUTDOWN;
	}

	rc = 0;
	switch (type) {
	case hwmon_in:
		if (attr != hwmon_in_input || channel > 4) {
			rc = -EOPNOTSUPP;
			break;
		}
		switch (channel) {
		case 0:	*val = priv->vin_mv;	break;
		case 1:	*val = priv->vout_mv;	break;
		case 2:	*val = priv->v5_mv;	break;
		case 3:	*val = priv->v33_mv;	break;
		case 4:	*val = priv->vn12_mv;	break;
		}
		break;
	case hwmon_curr:
		if (attr != hwmon_curr_input) {
			rc = -EOPNOTSUPP;
			break;
		}
		if (channel == 0)
			*val = priv->iin_ma;
		else if (channel <= 10)
			*val = priv->rails_ma[channel - 1];
		else
			rc = -EOPNOTSUPP;
		break;
	case hwmon_power:
		if (attr != hwmon_power_input) {
			rc = -EOPNOTSUPP;
			break;
		}
		if (channel == 0)
			*val = priv->pout_uw;	/* DC output, primary */
		else if (channel == 1)
			*val = priv->pin_uw;
		else
			rc = -EOPNOTSUPP;
		break;
	case hwmon_temp:
		if (attr != hwmon_temp_input) {
			rc = -EOPNOTSUPP;
			break;
		}
		*val = priv->temp_millic;
		break;
	case hwmon_fan:
		if (attr != hwmon_fan_input) {
			rc = -EOPNOTSUPP;
			break;
		}
		*val = priv->fan_rpm;
		break;
	default:
		rc = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&priv->io_lock);
	return rc;
}

static int xpg_hwmon_read_string(struct device *dev,
				 enum hwmon_sensor_types type, u32 attr,
				 int channel, const char **str)
{
	if (type == hwmon_in && attr == hwmon_in_label && channel < 5)
		*str = xpg_in_labels[channel];
	else if (type == hwmon_curr && attr == hwmon_curr_label && channel < 11)
		*str = xpg_curr_labels[channel];
	else if (type == hwmon_power && attr == hwmon_power_label && channel < 2)
		*str = xpg_power_labels[channel];
	else if (type == hwmon_temp && attr == hwmon_temp_label)
		*str = "PSU GaN";
	else if (type == hwmon_fan && attr == hwmon_fan_label)
		*str = "PSU Fan";
	else
		return -EOPNOTSUPP;
	return 0;
}

static const struct hwmon_channel_info *const xpg_channel_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_LABEL,
			   HWMON_P_INPUT | HWMON_P_LABEL),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT | HWMON_F_LABEL),
	NULL
};

static const struct hwmon_chip_info xpg_chip_info = {
	.ops = &(const struct hwmon_ops) {
		.visible = 0444,
		.read = xpg_hwmon_read,
		.read_string = xpg_hwmon_read_string,
	},
	.info = xpg_channel_info,
};

/* --- Extra RO attributes: efficiency, model, FW ----------------------------- */

static ssize_t xpg_eff_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct xpg_priv *priv = dev_get_drvdata(dev);
	s64 eff;

	/* eff_pct is written by poll; read it under the same io_lock */
	mutex_lock(&priv->io_lock);
	eff = priv->eff_pct;
	mutex_unlock(&priv->io_lock);

	return sysfs_emit(buf, "%lld\n", eff);
}

static ssize_t xpg_model_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct xpg_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", priv->model);
}

static ssize_t xpg_fw_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xpg_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", priv->fw);
}

static struct device_attribute dev_attr_eff =
	__ATTR(efficiency_percent, 0444, xpg_eff_show, NULL);
static struct device_attribute dev_attr_model =
	__ATTR(model, 0444, xpg_model_show, NULL);
static struct device_attribute dev_attr_fw =
	__ATTR(firmware_version, 0444, xpg_fw_show, NULL);

static struct attribute *xpg_attrs[] = {
	&dev_attr_eff.attr,
	&dev_attr_model.attr,
	&dev_attr_fw.attr,
	NULL
};

static const struct attribute_group xpg_attr_group = {
	.attrs = xpg_attrs,
};

static const struct attribute_group *xpg_attr_groups[] = {
	&xpg_attr_group,
	NULL
};

/* --- USB driver --------------------------------------------------------------- */

static void xpg_read_mfr_string(struct xpg_priv *priv, u8 cmd, char *dst, size_t len)
{
	int rc;

	rc = xpg_transfer(priv, cmd);
	if (rc)
		return;
	dst[0] = '\0';
	strscpy(dst, (const char *)&priv->buf[3], len);
}

static int xpg_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct xpg_priv *priv;
	struct device *hwmon_dev;

	priv = devm_kzalloc(&intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->intf = intf;
	priv->udev = interface_to_usbdev(intf);
	mutex_init(&priv->io_lock);
	INIT_DELAYED_WORK(&priv->poll, xpg_poll);
	usb_set_intfdata(intf, priv);

	/* Device info (best effort) */
	mutex_lock(&priv->io_lock);
	xpg_read_mfr_string(priv, CMD_MFR_ID, priv->mfr, sizeof(priv->mfr));
	xpg_read_mfr_string(priv, CMD_MFR_MODEL, priv->model, sizeof(priv->model));
	xpg_read_mfr_string(priv, CMD_MFR_FW, priv->fw, sizeof(priv->fw));
	mutex_unlock(&priv->io_lock);
	dev_info(&intf->dev, "%s %s, FW %s\n",
		 priv->mfr[0] ? priv->mfr : "(no MFR_ID)",
		 priv->model[0] ? priv->model : "(no model)",
		 priv->fw[0] ? priv->fw : "(no FW)");

	hwmon_dev = devm_hwmon_device_register_with_info(
		&intf->dev, DRIVER_NAME, priv, &xpg_chip_info, xpg_attr_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	schedule_delayed_work(&priv->poll, 0);

	dev_info(&intf->dev,
		 "hwmon registered, polling every %d s (read-only)\n",
		 interval_sec);
	return 0;
}

static void xpg_disconnect(struct usb_interface *intf)
{
	struct xpg_priv *priv = usb_get_intfdata(intf);

	/* Set the stopping flag under io_lock — the same critical section
	 * in which poll decides whether to re-arm the work (see xpg_poll).
	 * By the time cancel_delayed_work_sync runs the timer is either
	 * already pending (and will be removed) or poll will never re-arm
	 * again: UAF of priv is ruled out.
	 * URBs do not need to be killed — usbcore has already aborted them
	 * before disconnect. hwmon and priv are released by the devm
	 * machinery after disconnect. */
	mutex_lock(&priv->io_lock);
	priv->stopping = true;
	mutex_unlock(&priv->io_lock);
	cancel_delayed_work_sync(&priv->poll);
}

static int xpg_suspend(struct usb_interface *intf, pm_message_t msg)
{
	struct xpg_priv *priv = usb_get_intfdata(intf);

	/* The PSU is self-powered; polling is not needed while suspended.
	 * stopping under io_lock — so a stray re-arm from xpg_poll cannot
	 * outlive the cancel. */
	mutex_lock(&priv->io_lock);
	priv->stopping = true;
	mutex_unlock(&priv->io_lock);
	cancel_delayed_work_sync(&priv->poll);
	return 0;
}

static int xpg_resume(struct usb_interface *intf)
{
	struct xpg_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->io_lock);
	priv->stopping = false;
	priv->valid = false;
	mutex_unlock(&priv->io_lock);
	schedule_delayed_work(&priv->poll,
			      msecs_to_jiffies(xpg_interval_ms()));
	return 0;
}

static const struct usb_device_id xpg_id_table[] = {
	{ USB_DEVICE(XPG_VID, XPG_PID) },
	{ /* terminated */ }
};
MODULE_DEVICE_TABLE(usb, xpg_id_table);

static struct usb_driver xpg_driver = {
	.name = DRIVER_NAME,
	.id_table = xpg_id_table,
	.probe = xpg_probe,
	.disconnect = xpg_disconnect,
	.suspend = xpg_suspend,
	.resume = xpg_resume,
};
module_usb_driver(xpg_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Staroverov <nsforth@gmail.com>");
MODULE_DESCRIPTION("hwmon driver for XPG Fusion 1600W Titanium PSU (read-only)");
