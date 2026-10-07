# M5Stack Cardputer ADV

The `cardputer-adv` firmware is the standard ESP32-S3 burst firmware plus a
standalone receiver on the Cardputer ADV's own screen and keyboard. It shows a
live spectrum and waterfall without a computer, and still answers the normal
serial protocol over native USB, so the browser viewer, `esp-sdr-bridge` and
the S3 `IQS`/`SPEC` modes keep working when a host is plugged in.

![Cardputer ADV spectrum and waterfall layout (host-side render of the UI code with a synthetic test signal).](cardputer-adv-ui.png)

**Preview:** built and checked in a host-side render only; not yet validated on
hardware. Please report display orientation or keyboard mapping problems.

## Install

- **M5Launcher (SD card):** copy `esp-sdr-cardputer-adv.bin` to the SD card and
  install it from Launcher's SD menu. Launcher reads the partition table in the
  image and installs the app; the app-only `cardputer-adv/2-esp_sdr.bin` also
  works.
- **USB:** flash `esp-sdr-cardputer-adv.bin` at offset `0x0` with any
  esptool-based flasher, or use `flash_args` from the artifact directory.

Build it like any other profile:

```sh
python tools/build_firmware.py --profile cardputer-adv --version local --output artifacts
```

## Screen

| Area | Content |
| --- | --- |
| Top line | Centre frequency, span, gain (`AGC` or `G<index>`), tuning step, `HOLD`/`BW` flags |
| Spectrum | Welch average of 16 × 256-point Hann FFTs per frame, dBFS; yellow peak hold; optional marker |
| Axis | Left/centre/right frequency, reference level and displayed range |
| Waterfall | Newest line on top, colour scaled to the same reference/range |

Each frame is one 4096-sample burst capture at 80, 40 or 16 MS/s; the receiver
is idle between frames (snapshot spectrum, not gap-free).

## Keys

| Key | Action |
| --- | --- |
| `,` `/` (Fn: ← →) | Tune down / up by the step |
| `;` `.` (Fn: ↑ ↓) | Step size 1/2/5/10/20 MHz |
| `f` | Type a frequency in MHz, `Enter` to tune, `` ` `` to cancel |
| `1`–`9`, `0` | Wi-Fi channels 1–9, 13 |
| `s` | Span 80 / 40 / 16 MHz |
| `g` | Hardware AGC ↔ manual gain; `-` `=` adjust manual gain |
| `b` | Analog filter automatic ↔ 20 MHz |
| `r` | Autoscale reference/range; `[` `]` move the reference by 5 dB |
| `a` | Averaging (off, light, medium, heavy) |
| `p` / `m` / `d` | Peak hold / peak marker / per-segment DC removal |
| `space` | Hold the display |
| `o` | Rotate the picture 180° |
| `h`, `?`, `Tab` | Key help |
| `q`, then `y` | Save settings and reboot (Launcher shows on boot) |

Frequency, span, step, gain, scale and orientation persist in NVS.

## With a host attached

The display runs only between serial commands. When a host sends a command it
takes the serial lease, the status line reads **USB host in control** and the
display freezes; it resumes five seconds after the host goes quiet and follows
any frequency the host set. Ring modes (`RING`, `SPEC`, `IQS`) overwrite ring
bank 0, which the waterfall borrows to save heap, so the waterfall restarts
after a host session.

## Hardware notes

| Function | Pins |
| --- | --- |
| ST7789V2 135×240 LCD (SPI3) | MOSI 35, SCLK 36, CS 37, DC 34, RST 33, backlight 38 |
| TCA8418 keyboard (I2C 0x34) | SDA 8, SCL 9, INT 11 (polled, unused) |
| Reserved for SD and audio | 12, 14, 39–44, 46 |

UART0 is disabled because its default pins (43/44) belong to the audio codec
and IR LED; native USB carries the protocol. The board pins are reserved before
GPIO discovery, so `GPIO?` exposes only the free header/Grove pins. The SPI and
I2C interrupt handlers run from flash so DRAM stays below the S3 RF ring
(`sram_guard.ld`).
