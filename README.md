# xpg-fusion — hwmon driver for the XPG Fusion 1600W Titanium PSU

Linux kernel DKMS module. Reads PSU telemetry (Delta Electronics, GPS-1600AB,
VID 0x125F / PID 0x7900) and exports it via hwmon (lm_sensors / `sensors`).

Read-only: the module sends PMBus read commands only
(0xDF FULLPACKAGE, 0x96 POUT, 0x99/0x9A/0xDC MFR strings). No writes.
Tested with kernel 7.0.0-34-generic (Ubuntu 26.04).

## Channels

| hwmon    | Value               | Unit | Label                          |
|----------|---------------------|------|--------------------------------|
| in1      | AC input voltage    | mV   | AC Input                       |
| in2      | +12V                | mV   | +12V                           |
| in3      | +5V                 | mV   | +5V                            |
| in4      | +3.3V               | mV   | +3.3V                          |
| in5      | −12V (magnitude)    | mV   | -12V                           |
| curr1    | AC input current    | mA   | IIN                            |
| curr2-7  | +12V1..+12V6 current| mA   | +12V1..6                       |
| curr8-11 | −12V, +5V, +3.3V, +12V total current | mA | -12V, +5V, +3.3V, +12V |
| power1   | **Output power (DC)** | µW  | DC Output                      |
| power2   | Input power (AC)    | µW   | AC Input                       |
| temp1    | GaN temperature     | m°C  | PSU GaN                        |
| fan1     | Fan speed           | RPM  | PSU Fan                        |

Extra sysfs attributes (hwmonX/): `efficiency_percent` (the PSU's claimed
efficiency — without XPG Prime's marketing corrections), `model`,
`firmware_version`.

## Build and installation

```
sudo ./xpg-fusion-dkms-build.sh      # dkms add + build
sudo ./xpg-fusion-dkms-install.sh    # dkms install + udevadm trigger
```

The module is picked up by udev automatically via the `usb:v125Fp7900...`
alias (plug in the PSU — the module loads). If the PSU is already connected
at install time, the script runs `udevadm trigger`.

Uninstall: `dkms remove xpg-fusion/1.0 --all` (or `sudo rmmod xpg-fusion`
for a temporary one). After removal, the userspace script
(`xpg_fusion/xpg_fusion_monitor.py`) can open the device again.

## Module parameters

- `interval_sec` (default 1) — polling interval in seconds (1..60).
  `modprobe xpg-fusion interval_sec=2`.

## Known behavior

- While the module is bound, XPG Prime / pyusb cannot open the device
  (-EBUSY). This is normal behavior (exclusive driver-core claim).
- If the PSU is powered off on AC: control requests time out (2 s), polling
  switches to backoff (×2, up to 10 s), `dev_warn_ratelimited` in dmesg.
  The kernel is not blocked.
- The response is validated: command echo + PEC (CRC-8). The MCU silently
  drops commands with a bad PEC — the module reports this as -EPROTO.
- −12V is exported with a positive sign, label "-12V" (dual-polarity rail).
- `usb_reset_device` is NOT called automatically (the PSU is not a
  harmless device; if needed — manual `usb_reset_device` from userspace,
  or v2 with an opt-in parameter).

## Transfer structure (briefly)

HID Feature Report ID 7, exactly 257 bytes, wValue=0x0307:
SET_REPORT [0]=07 [1]=00 [2]=command [255]=PEC → 50 ms → GET_REPORT
(257 bytes: [2]=echo, [3..]=data, [255]=PEC) → 30 ms.
Values are Linear11 with a non-standard Delta packing (see xpg_fusion.c,
function l11_scaled). The FULLPACKAGE field map is in xpg_decode_full().
