# M5Stack Cardputer ADV

The `cardputer-adv` firmware is the standard ESP32-S3 burst firmware plus a
standalone receiver on the Cardputer ADV's own screen, keyboard and speaker. It
shows a live spectrum and waterfall without a computer, plays NFM/AM/WFM/CW
audio in listen mode, hops between busy signals with the scanner, and still
answers the normal
serial protocol over native USB, so the browser viewer, `esp-sdr-bridge` and
the S3 `IQS`/`SPEC` modes keep working when a host is plugged in.

![Cardputer ADV spectrum and waterfall layout (host-side render of the UI code with a synthetic test signal).](cardputer-adv-ui.png)

**Preview:** the spectrum/waterfall build and listen mode run on a real
Cardputer ADV via M5Launcher. The scanner and CW mode are new and checked in a
host-side simulation (synthetic band and demodulator) only.

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
| `j` | Scanner: hunt the band and play a random busy signal (below) |
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
  75 µs de-emphasis), **CW** (500 Hz filter, AGC; a carrier on the listen
  frequency becomes a tone, held near 700 Hz by tracking its offset).
- **Squelch** watches the FM discriminator's high-frequency noise (AM: the
  envelope's), so it opens on any clean carrier without a level setting;
  `sql 0` keeps it open. In CW it gates the noise between dits.
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
| `m` | Mode NFM → AM → WFM → CW |
| `-` `=` | Volume (16 steps, about 3 dB each) |
| `[` `]` | Squelch 0 (open) to 9 (tight) |
| `v` | Live signal meter on/off |
| `j` | Start the scanner; while scanning, hop to the next signal |
| `x` | The scanner never picks this frequency again |
| `l`, `` ` `` | Back to the spectrum |
| `h`, `?` | Listen help (audio keeps playing) |

Mode, volume, squelch, step and the meter setting persist in NVS.

## Scanner

![Scanner: sweeping the 2300-2483 MHz panorama, then playing a Morse beacon in CW mode (host-side render with a synthetic band).](cardputer-adv-scan.png)

`j` (on the spectrum or in listen mode) hunts 2300-2483.5 MHz for signals
that could carry voice or Morse and plays a random one. That range is the
13 cm amateur band (2300-2450 MHz, which parts depend on the country) and the
2.4 GHz ISM band. The radio only hears roughly 1.8-2.6 GHz, so HF Morse,
shortwave number stations and the like are out of reach. What you can catch
up here: ham NBFM and CW beacons (2304 and 2320 MHz narrowband segments in
many regions), analog 2.4 GHz baby monitors, wireless mics and headsets.

1. **Sweep.** The spectrum area turns into a panorama of the whole range.
   Four passes step 16 windows across it, four 16 MS/s snapshots (62.5 kHz
   bins) per window; odd passes are shifted half a window, so each frequency
   is seen from two LO grids. A bin counts as hit in a snapshot when it
   stands 8 dB over that snapshot's noise floor.
2. **Candidates.** Peaks up to four bins wide (eight for wide FM) that were
   hit in at least a quarter of the snapshots, from both grids, at 10 dB or
   more. Seen from one grid only means a spur or IQ image tied to the LO; the
   40 MHz crystal's harmonics (2320, 2360, 2400, 2440, 2480 MHz) are skipped.
   Wide bursty regions (Wi-Fi, Bluetooth, microwave ovens) are kept as a
   fallback.
3. **Pick.** A weighted random pick, stronger and steadier first, rarely the
   one just played. Wide bursts only play when nothing narrow is left, as
   10 s of AM "data chatter".
4. **Centre.** A muted 250 ms wide probe measures where the carrier sits
   and how far it swings: tens of kHz of deviation means wide FM (headsets,
   AV senders), which then plays as WFM. Then NFM, re-centred every second.
5. **Listen and judge.** Each 1 s run measures carrier presence, keying and
   audio activity:
   - on/off keying at 1.5-40 edges a second switches to **CW**;
   - a solid carrier whose audio stays flat for 5 s is a **dead carrier**
     (often a spur): the scanner hops on, and the third such verdict on the
     same frequency skips it for good (a beacon's long key-down looks dead
     for a while too);
   - 8 s without the carrier, or 60 s of dwell (30 s WFM), hops to the next.

The status line shows the band, how many candidates are left and the dwell.
After two minutes, or when the list is used up, the scanner sweeps again.
Tuning (`,` `/` `f`) or `m` stops the scanner where it is. The skip list
(thrice-dead carriers and `x`) keeps the last 16 frequencies in NVS.

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
