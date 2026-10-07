// SPDX-License-Identifier: GPL-2.0-only
/*
 * xpg_fusion.c — hwmon-драйвер БП XPG Fusion 1600W Titanium (Delta Electronics,
 * GPS-1600AB, VID 0x125F / PID 0x7900).
 *
 * Устройство — виртуальный HID без endpoint'ов (bNumEndpoints=0), весь обмен —
 * control transfer на EP0: HID Feature Report ID 7, буфер ровно 257 байт.
 *
 * Протокол (восстановлен из XPG Prime, resources/app.asar, FusionSeries.js +
 * живые зонды; см. xpg_fusion_monitor.py):
 *   запрос:  SET_REPORT  bmReq=0x21 bReq=0x09 wValue=0x0307 len=257
 *            [0]=0x07 [1]=0x00 [2]=команда [3..254]=0 [255]=PEC
 *   ответ:   GET_REPORT  bmReq=0xA1 bReq=0x01 wValue=0x0307 wLength=257
 *            [0]=0x07 [1]=0x00 [2]=эхо [3..254]=данные [255]=PEC
 *   PEC: CRC-8 (PMBus) по байтам 0..254. Без правильного PEC MCU молча
 *        отбрасывает команду (USB ACK, репорт нулевой) — валидируем echo+PEC.
 *
 * Только read-команды: 0xDF (FULLPACKAGE), 0x96 (POUT), 0x99/0x9A/0xDC (MFR).
 * Записей в устройство НЕ отправляем.
 *
 * Проверено: ядро 7.0.0-34-generic (Ubuntu 26.04).
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

#define REPORT_LEN	257		/* 1 (Report ID) + 256 (данные) */
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
#define BACKOFF_START_ERRS	3
#define BACKOFF_MAX_MS		10000

static int interval_sec = INTERVAL_DEF_SEC;
module_param(interval_sec, int, 0444);
MODULE_PARM_DESC(interval_sec, "Poll interval in seconds (default 1, range 1..60)");

/* --- PEC: точный порт PmbusAPIs.calcPEC из XPG Prime ------------------------ */

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

/* --- Linear11 (укладка Delta, порт PmbusAPIs.Linear11_Decoder) ---------------
 * N = ((hi & 0x78) >> 3); если hi & 0x80, N -= 16   (диапазон -16..7)
 * M = lo + (hi & 0x03) * 256; если hi & 0x04, M -= 1024   (-1024..1023)
 * X = M * 2^N. Возвращаем X * mult в целых (mV/mA/m°C/µW/RPM), s64.
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

/* --- Состояние устройства ---------------------------------------------------- */

struct xpg_priv {
	struct usb_interface *intf;
	struct usb_device *udev;
	struct delayed_work poll;
	struct mutex io_lock;
	bool stopping;
	u8 buf[REPORT_LEN];
	bool valid;
	unsigned long updated;
	int err_streak;
	s64 vin_mv;
	s64 vout_mv;
	s64 v5_mv;
	s64 v33_mv;
	s64 vn12_mv;		/* |-12V|, модуль положительный (label "-12V") */
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

/* интервал с backoff при сериях ошибок (до 10 с) */
static int xpg_poll_interval_ms(struct xpg_priv *priv)
{
	int ms = xpg_interval_ms();
	int shift;

	if (priv->err_streak >= BACKOFF_START_ERRS) {
		shift = priv->err_streak - BACKOFF_START_ERRS;
		if (shift > 3)
			shift = 3;
		ms *= 1 << shift;
		if (ms > BACKOFF_MAX_MS)
			ms = BACKOFF_MAX_MS;
	}
	return ms;
}

static void xpg_sanity_check(struct xpg_priv *priv, struct device *dev)
{
#define XPG_WARN_IF(cond, fmt, ...) do {		\
	if (cond)					\
		dev_warn_ratelimited(dev, fmt, ##__VA_ARGS__); \
} while (0)

	XPG_WARN_IF(priv->vin_mv < 80000 || priv->vin_mv > 300000,
		    "VIN вне диапазона: %lld mV\n", priv->vin_mv);
	XPG_WARN_IF(priv->vout_mv < 11000 || priv->vout_mv > 13500,
		    "VOUT вне диапазона: %lld mV\n", priv->vout_mv);
	XPG_WARN_IF(priv->temp_millic < -10000 || priv->temp_millic > 90000,
		    "TEMP вне диапазона: %lld mC\n", priv->temp_millic);
	XPG_WARN_IF(priv->fan_rpm < 0 || priv->fan_rpm > 3000,
		    "FAN вне диапазона: %lld RPM\n", priv->fan_rpm);
	XPG_WARN_IF(priv->pout_uw < 0 || priv->pout_uw > 2000000000LL,
		    "POUT вне диапазона: %lld uW\n", priv->pout_uw);
#undef XPG_WARN_IF
}

/* --- Обмен ------------------------------------------------------------------- */

/* Один полный цикл: SET(cmd) -> 50мс -> GET -> 30мс -> валидация.
 * Ответ остаётся в priv->buf. Вызывать под io_lock. */
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

	/* MCU молча отбрасывает команды с неверным PEC/неизвестной командой */
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

/* FULLPACKAGE + POUT, кэш актуализирован. Вызывать под io_lock. */
static int xpg_refresh(struct xpg_priv *priv)
{
	int rc;

	rc = xpg_transfer(priv, CMD_FULLPACKAGE);
	if (rc)
		return rc;
	xpg_decode_full(priv, priv->buf);
	xpg_sanity_check(priv, &priv->intf->dev);

	/* POUT свежее из отдельной команды (как в XPG Prime) */
	rc = xpg_transfer(priv, CMD_READ_POUT);
	if (rc)
		return rc;
	priv->pout_uw = l11_scaled(priv->buf[3], priv->buf[4], 1000000);

	priv->valid = true;
	priv->updated = jiffies;
	return 0;
}

/* --- Периодический опрос ------------------------------------------------------ */

static void xpg_poll(struct work_struct *ws)
{
	struct xpg_priv *priv = container_of(to_delayed_work(ws),
					     struct xpg_priv, poll);
	int rc;

	mutex_lock(&priv->io_lock);
	rc = xpg_refresh(priv);
	if (rc) {
		priv->err_streak++;
		if (priv->err_streak == 1)
			dev_warn(&priv->intf->dev,
				 "Опрос БП: ошибка %d, включён backoff\n", rc);
	} else {
		priv->err_streak = 0;
	}

	/* Re-arm под тем же io_lock, под которым disconnect/suspend ставят
	 * stopping. Тогда к моменту cancel_delayed_work_sync таймер либо уже
	 * pending (и будет снят), либо poll не реармировался: исключается
	 * гонка, при которой sync-cancel возвращался бы с pending-таймером
	 * после освобождения priv (UAF). */
	if (!priv->stopping)
		schedule_delayed_work(&priv->poll,
				      msecs_to_jiffies(xpg_poll_interval_ms(priv)));
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
	unsigned long now;
	int rc;

	/* Весь доступ к кэшу под io_lock: поля plain (без own-lock), и
	 * stale-check атомарен относительно refresh. hwmon свои читатели
	 * сериализует собственным mutex'ом; устаревший кэш обновляется
	 * синхронно, свежие чтения лока почти не держат. */
	mutex_lock(&priv->io_lock);
	if (priv->stopping) {
		mutex_unlock(&priv->io_lock);
		return -ESHUTDOWN;
	}

	now = jiffies;
	if (!priv->valid ||
	    time_after(now, priv->updated +
		       msecs_to_jiffies(xpg_interval_ms() + 1000))) {
		rc = xpg_refresh(priv);
		if (rc) {
			mutex_unlock(&priv->io_lock);
			return rc;
		}
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
			*val = priv->pout_uw;	/* DC-выход, основная */
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

/* --- Доп. RO-атрибуты: КПД, модель, FW --------------------------------------- */

static ssize_t xpg_eff_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct xpg_priv *priv = dev_get_drvdata(dev);
	s64 eff;

	/* eff_pct пишет poll; читаем под тем же io_lock */
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

/* --- USB-драйвер -------------------------------------------------------------- */

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

	/* Инфо об устройстве (best-effort) */
	mutex_lock(&priv->io_lock);
	xpg_read_mfr_string(priv, CMD_MFR_ID, priv->mfr, sizeof(priv->mfr));
	xpg_read_mfr_string(priv, CMD_MFR_MODEL, priv->model, sizeof(priv->model));
	xpg_read_mfr_string(priv, CMD_MFR_FW, priv->fw, sizeof(priv->fw));
	mutex_unlock(&priv->io_lock);
	dev_info(&intf->dev, "%s %s, FW %s\n",
		 priv->mfr[0] ? priv->mfr : "(нет MFR_ID)",
		 priv->model[0] ? priv->model : "(нет модели)",
		 priv->fw[0] ? priv->fw : "(нет FW)");

	hwmon_dev = devm_hwmon_device_register_with_info(
		&intf->dev, DRIVER_NAME, priv, &xpg_chip_info, xpg_attr_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	schedule_delayed_work(&priv->poll, 0);

	dev_info(&intf->dev,
		 "hwmon зарегистрирован, опрос каждые %d с (read-only)\n",
		 interval_sec);
	return 0;
}

static void xpg_disconnect(struct usb_interface *intf)
{
	struct xpg_priv *priv = usb_get_intfdata(intf);

	/* стоп-флаг под io_lock — тот же критический участок, в котором
	 * poll решает, реармировать ли work (см. xpg_poll). Тогда к моменту
	 * cancel_delayed_work_sync таймер либо уже pending (будет снят),
	 * либо poll больше не реармируется: UAF priv исключён.
	 * URB'ы убивать не нужно — usbcore уже их прервал до disconnect.
	 * hwmon и priv освобождаются devm-механизмом после disconnect. */
	mutex_lock(&priv->io_lock);
	priv->stopping = true;
	mutex_unlock(&priv->io_lock);
	cancel_delayed_work_sync(&priv->poll);
}

static int xpg_suspend(struct usb_interface *intf, pm_message_t msg)
{
	struct xpg_priv *priv = usb_get_intfdata(intf);

	/* БП self-powered, во время suspend опрос не нужен. stopping под
	 * io_lock — чтобы stray-реарм из xpg_poll не пережил cancel. */
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
	priv->err_streak = 0;
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
MODULE_AUTHOR("forth");
MODULE_DESCRIPTION("hwmon driver for XPG Fusion 1600W Titanium PSU (read-only)");
