# M5Stack Cardputer ADV

The `cardputer-adv` firmware is the standard ESP32-S3 burst firmware plus a
standalone receiver on the Cardputer ADV's own screen, keyboard and speaker. It
shows a live spectrum and waterfall without a computer, plays NFM/AM/WFM audio
in listen mode, and still answers the normal
serial protocol over native USB, so the browser viewer, `esp-sdr-bridge` and
the S3 `IQS`/`SPEC` modes keep working when a host is plugged in.

![Cardputer ADV spectrum and waterfall layout (host-side render of the UI code with a synthetic test signal).](cardputer-adv-ui.png)

**Preview:** the spectrum/waterfall build runs on a real Cardputer ADV via
M5Launcher. Listen mode and the sniffer tone are new and checked in a
host-side simulation of the demodulator only.

## Install

- **M5Launcher (SD card):** copy `esp-sdr-cardputer-adv.bin` to the SD card and
  install it from Launcher's SD menu. Launcher reads the partition table in the
  image and installs the app. Use this merged image: the app-only
  `cardputer-adv/2-esp_sdr.bin` has no bootloader and crashed when installed
  on its own.
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
| `l` | Listen mode on the speaker, at the marker peak (marker on) or the centre |
| `n` | Sniffer tone: pitch rises with the strongest signal's height above the noise |
| `h`, `?`, `Tab` | Key help |
| `q`, then `y` | Save settings and reboot (Launcher shows on boot) |

Frequency, span, step, gain, scale and orientation persist in NVS.

## Listen mode

![Listen mode: frozen spectrum with the listen frequency in red, frequency, mode, signal bar and FM carrier offset (host-side render).](cardputer-adv-listen.png)

`l` switches from snapshots to the S3 continuous IQ path (`IQS` mode 2): the
LO sits 4 MHz below the listen frequency (FOFS gives 1 kHz steps), the
two-stage FIR decimates 16 MS/s to 31.25 kS/s (250 kS/s for WFM), and core 1
demodulates every sample straight into the ES8311 codec's I2S DMA ring.

- **NFM** (±12.5 kHz channel, 3 kHz audio low-pass), **AM** (envelope,
  normalised to the carrier, so it doubles as AGC), **WFM** (±100 kHz,
  75 µs de-emphasis).
- **Squelch** watches the demodulator's high-frequency noise, so it opens on
  any clean carrier without a level setting; `sql 0` keeps it open.
- **ofs** is the FM carrier's offset from the listen frequency: tune toward it.
- The screen holds still while audio plays (the capture runs with interrupts
  masked); any key ends the run, acts, and audio resumes. `v` adds a
  once-per-second signal meter refresh, at the cost of a tiny audio skip.
- A USB host command ends listen mode and the host takes over as usual.

| Key | Action |
| --- | --- |
| `,` `/` (Fn: ← →) | Tune down / up by the step |
| `;` `.` (Fn: ↑ ↓) | Step 1/5/10/25/100/1000 kHz |
| `f` | Type a frequency in MHz with decimals, e.g. `2437.125` |
| `m` | Mode NFM → AM → WFM |
| `-` `=` | Volume (16 steps, about 3 dB each) |
| `[` `]` | Squelch 0 (open) to 9 (tight) |
| `v` | Live signal meter on/off |
| `l`, `` ` `` | Back to the spectrum |
| `h`, `?` | Listen help (audio keeps playing) |

Mode, volume, squelch, step and the meter setting persist in NVS.

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
| TCA8418 keyboard (I2C 0x34) | SDA 8, SCL 9, INT 11 (ends listen-mode IQ runs) |
| ES8311 codec (I2C 0x18) + I2S1 | BCLK 41, WS 43, DOUT 42 (MCLK derived from BCLK) |
| Reserved for SD and IR | 12, 14, 39, 40, 44, 46 |

UART0 is disabled because its default pins (43/44) belong to the audio codec
and IR LED; native USB carries the protocol. The board pins are reserved before
GPIO discovery, so `GPIO?` exposes only the free header/Grove pins. The SPI and
I2C interrupt handlers run from flash so DRAM stays below the S3 RF ring
(`sram_guard.ld`). Audio needs no interrupt: the I2S driver's DMA descriptors
form a ring, and the writer reads the GDMA current-descriptor register to stay
two buffers ahead of playback (8 × 7.7 ms buffers).
