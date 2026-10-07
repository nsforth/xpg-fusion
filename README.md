# xpg-fusion — hwmon-драйвер БП XPG Fusion 1600W Titanium

DKMS-модуль ядра Linux. Читает телеметрию БП (Delta Electronics, GPS-1600AB,
VID 0x125F / PID 0x7900) и экспортирует её через hwmon (lm_sensors / `sensors`).

Только read-only: модуль отправляет исключительно read-команды PMBus
(0xDF FULLPACKAGE, 0x96 POUT, 0x99/0x9A/0xDC MFR-строки). Записей нет.
Проверено на ядре 7.0.0-34-generic (Ubuntu 26.04).

## Каналы

| hwmon    | Значение            | Единицы | Label       |
|----------|---------------------|---------|-------------|
| in1      | Напряжение сети     | mV      | AC Input    |
| in2      | +12V                | mV      | +12V        |
| in3      | +5V                 | mV      | +5V         |
| in4      | +3.3V               | mV      | +3.3V       |
| in5      | −12V (модуль)       | mV      | -12V        |
| curr1    | Ток сети            | mA      | IIN         |
| curr2-7  | Ток +12V1..+12V6    | mA      | +12V1..6    |
| curr8-11 | Ток −12V, +5V, +3.3V, +12V total | mA | -12V, +5V, +3.3V, +12V |
| power1   | **Выходная мощность (DC)** | µW | DC Output |
| power2   | Входная мощность (AC)     | µW | AC Input  |
| temp1    | Температура GaN     | m°C     | PSU GaN     |
| fan1     | Обороты вентилятора | RPM     | PSU Fan     |

Доп. атрибуты sysfs (hwmonX/): `efficiency_percent` (заявленный КПД БП —
без маркетинговых поправок XPG Prime), `model`, `firmware_version`.

## Сборка и установка

```
sudo ./xpg-fusion-dkms-build.sh      # dkms add + build
sudo ./xpg-fusion-dkms-install.sh    # dkms install + udevadm trigger
```

Модуль подхватывается udev автоматически по alias `usb:v125Fp7900...`
(подключили БП — модуль загрузился). Если при установке БП уже подключён,
скрипт делает `udevadm trigger`.

Отключение: `dkms remove xpg-fusion/1.0 --all` (или `sudo rmmod xpg-fusion`
для временного). После отключения userspace-скрипт
(`xpg_fusion/xpg_fusion_monitor.py`) снова может открыть устройство.

## Параметры модуля

- `interval_sec` (по умолчанию 1) — период опроса в секундах (1..60).
  `modprobe xpg-fusion interval_sec=2`.

## Известное поведение

- Пока модуль привязан, XPG Prime / pyusb не смогут открыть устройство
  (-EBUSY). Это нормальное поведение (исключительный захват driver core).
- Если БП выключен по AC: control-запросы дают таймаут (2 с), опрос
  переходит в backoff (×2 до 10 с), `dev_warn_ratelimited` в dmesg.
  Ядро не блокируется.
- Ответ валидируется: эхо команды + PEC (CRC-8). MCU молча отбрасывает
  команды с неверным PEC — модуль обработает это как -EPROTO.
- −12V выводится с положительным знаком, label "-12V" (двухполярное питание).
- `usb_reset_device` НЕ вызывается автоматически (БП — не безобидное
  устройство; при необходимости — ручной `usb_reset_device` из userspace
  или v2 с opt-in параметром).

## Структура обмена (кратко)

HID Feature Report ID 7, ровно 257 байт, wValue=0x0307:
SET_REPORT [0]=07 [1]=00 [2]=команда [255]=PEC → 50 мс → GET_REPORT
(257 байт: [2]=эхо, [3..]=данные, [255]=PEC) → 30 мс.
Значения — Linear11 в нестандартной укладке Delta (см. xpg_fusion.c,
функция l11_scaled). Карта полей FULLPACKAGE — в xpg_decode_full().
