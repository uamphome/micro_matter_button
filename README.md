# Micro Matter Button

A tiny, coin-cell-powered **Matter over Thread** smart home button built on the [HolyIoT 25008](https://s.click.aliexpress.com/e/_c4MbjoMv) and [HolyIoT 25015](https://s.click.aliexpress.com/e/_c3R5B6Fn) Bluetooth beacon boards. Both boards use the Nordic nRF54l15 SoC — a chip capable of running Matter and Zigbee, making it perfect for a DIY wireless smart home button.

This project was built for a YouTube build video. See the full step-by-step on my [YouTube channel](https://www.youtube.com/@uamphome)

[![Alt Text](assets/images/04_micro_matter_button.png)](assets/images/04_micro_matter_button.png)

> **No soldering required.** Flash the precompiled binary, commission it to your smart home, and you're done.

---

## Hardware

| Item | Notes |
|------|-------|
| [HolyIoT 25008](https://s.click.aliexpress.com/e/_c4MbjoMv) | Core board — button only |
| [HolyIoT 25015](https://s.click.aliexpress.com/e/_c3R5B6Fn) | Adds SHT40 temperature & humidity sensor |
| [Seeed Studio Xiao RP2040](https://s.click.aliexpress.com/e/_c3o6kaAl) | Used as a CMSIS-DAP programmer. via picoprobe firmware. Any Raspberry PI pico board with the RP2040 or RP2350 will do |
| [PCB spring-pin test clip](https://s.click.aliexpress.com/e/_c39zxHAZ) | Clamps to the board's SWD pads — no soldering needed |
| CR2032 coin cell battery | Powers the finished button |

The **25015** is a drop-in hardware upgrade over the 25008. It uses the same firmware but additionally reports temperature and humidity to your smart home hub. Both boards run off a single coin cell.

---

## Quick Start — Precompiled Binaries

> **Start here.** You do not need to install the nRF Connect SDK or compile anything.

### Step 1 — Download the right binary

Four precompiled binaries are provided in the [Releases](../../releases/latest) section:

| File | Board | Mode |
|------|-------|------|
| `micro_matter_button_h25008_om1.hex` | HolyIoT 25008 | Mode 1 — Dimmable switch (requires binding) |
| `micro_matter_button_h25008_om2.hex` | HolyIoT 25008 | Mode 2 — Generic switch |
| `micro_matter_button_h25015_om1.hex` | HolyIoT 25015 | Mode 1 — Dimmable switch (requires binding) |
| `micro_matter_button_h25015_om2.hex` | HolyIoT 25015 | Mode 2 — Generic switch |

**Not sure which mode to pick?** Read the [Operating Modes](#operating-modes) section below, then come back.

### Step 2 — Set up the programmer

The HolyIoT boards have no USB port. You flash them over SWD using a Raspberry Pi Pico running picoprobe firmware.

1. Hold the **BOOTSEL** button on the Pico and plug it into your computer.
2. Download the [picoprobe UF2](https://github.com/raspberrypi/picoprobe/releases) and drag it onto the Pico — it will appear as a USB drive.
3. Wire the Pico to the HolyIoT board's SWD pads through the PCB test clip:

   | Pico Pin | HolyIoT Pad |
   |----------|-------------|
   | GP2 (SWCLK) | SWCLK |
   | GP3 (SWDIO) | SWDIO |
   | GND | GND |
   | 3V3 | VCC (optional — or use the coin cell) |

<!-- INSERT PICTURE: PCB test clip attached to HolyIoT board sitting in 3D-printed programming jig, wired to Pico -->

### Step 3 — Flash with pyocd

Install pyocd and its nRF54l pack:

```bash
pip install pyocd
```

Flash the firmware (replace the filename with your chosen binary):

```bash
pyocd load ./holyiot_25008_mode1.hex -t nrf54l
```

That's it — the chip will reset and start advertising over Bluetooth LE.

### Step 4 — Commission to your smart home

Open your smart home app and add a new Matter device. Use the QR code or manual pairing code below when prompted.

[![Alt Text](assets/images/setup.png)](assets/images/setup.png)

**Manual pairing code:** `34970112332`  
**QR Code:** [Scan or screenshot this](https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT:4CT9142C00KA0648G00)

> The LED on the board will blink slowly while waiting for commissioning and turn off once paired.

---

## Operating Modes

Two firmware operating modes are available. Choose at flash time — no recompilation needed, just pick the right binary.

### Mode 1 — Dimmable Switch

> ⚠️ **This mode requires Matter binding to work.** After commissioning, you must bind the button to a light using your smart home hub. Without a binding, button presses are silently ignored. See [Setting Up Binding](#setting-up-binding-mode-1-only) below.

This mode acts as a **Dimmable Light Switch** (Matter Light Switch device type). Commands are sent directly to bound lights over the Thread network.

| Gesture | Action |
|---------|--------|
| Short press (< 400 ms) | Toggle the bound light on/off |
| Long press (hold > 500 ms) | Cycle brightness from 0% → 100% while held |
| Double-tap (two presses within 400 ms) | Wake the accelerometer and enter rotational dimming mode |
| Rotate while in dimming mode | Adjust brightness proportionally to the rotation angle |
| Tilt horizontally | Exit dimming mode and lock current brightness |

Rotational dimming activates for up to **10 seconds** after a double-tap. If no movement is detected for **3 seconds** during dimming, the accelerometer powers back down automatically to save battery.

**Best used with:** Home Assistant, or any Matter hub that supports binding.

---

### Mode 2 — Generic Switch

> ✅ **No binding required.** This mode works out of the box with Apple Home, Google Home, Amazon Alexa, Home Assistant, and any other Matter-certified platform.

This mode exposes a **Generic Switch** (Matter Generic Switch device type) on endpoint 2. Your smart home platform receives the button events and you configure what happens via automations.

| Gesture | Matter Event |
|---------|-------------|
| Short press | `ShortRelease` |
| Long press (hold > 500 ms) | `LongPress` → `LongRelease` on release |

Use your platform's automation builder to trigger any action — turn on a light, run a scene, send a notification, or anything else your hub supports.

**Works with:** Apple Home, Google Home, Amazon Alexa, Home Assistant, Samsung SmartThings, and any Matter-compatible hub.

---

## Setting Up Binding (Mode 1 Only)

Binding is a Matter concept that tells the switch which light(s) to control. Without it, the button has no target to send commands to.

### With Home Assistant (recommended)

Home Assistant makes binding straightforward, especially when running on a VM or alongside a Thread Border Router. I have a dedicated video on my YouTube channel walking through how to share Matter devices across fabrics and set up bindings in Home Assistant — [watch it here](https://www.youtube.com/watch?v=eK-TLgXYBtU).

### With chip-tool (advanced)

If you prefer the command line, use chip-tool to write the binding table.

---

## Battery Life

Average current consumption is approximately **6 µA** — achieved by:

- Running as a Thread Sleepy End Device (SED) with long poll intervals
- Powering down the LIS2DH12 accelerometer at boot (Mode 2) or on demand (Mode 1)
- Keeping all debug/serial peripherals disabled in the production build
- Link-time optimisation (LTO) enabled

A standard CR2032 (≈220 mAh capacity) gives a rough estimate of **~36,000 hours / ~4 years** of theoretical runtime. Real-world life will be lower due to battery self-discharge and voltage droop, but multi-year life off a single coin cell is realistic.

---

## 3D Printed Case

<!-- INSERT PICTURE: Side-by-side of plain case and grooved case designs -->

STL files for the 3D printed case are available in the `assets/stl/` directory. The design is two parts that screw together — the board sits in the base and the top half has a flexible plunger that transmits button presses through the face of the case. Prints cleanly in PLA with no post-processing.

Two versions are provided:
- `micro_matter_button_25008.stl` - Holyiot 25008
- `micro_matter_button_25015.stl` - Holyiot 25015
- `programming_jig.stl` - Jig to hold the Holyiot Board during programming

---

## HolyIoT 25015 — Temperature & Humidity Variant

The 25015 uses the same firmware binaries but adds two extra sensors:

- **SHT40** — temperature and humidity (fully working, reported every 5 minutes after commissioning)
- **LPS22HB** — barometric pressure (not yet implemented — the driver in NCS assumes I²C; the board uses SPI. The sensor is explicitly powered down at boot.)

The 25015 case has a small airflow hole near the sensor for accurate readings.

---

## Building from Source

For developers who want to customise the firmware.

### Prerequisites

- [nRF Connect SDK v3.x](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation.html) (v3.2.4 used for the precompiled binaries)
- West build tools

### Selecting a mode at build time

The operating mode is a Kconfig choice. Pass it on the west build command line:

```bash
# Mode 1 — Dimmable switch
west build -b holyiot_25008_nrf54l15_cpuapp -- -DCONFIG_MATTER_BUTTON_MODE_1=y

# Mode 2 — Generic switch
west build -b holyiot_25008_nrf54l15_cpuapp -- -DCONFIG_MATTER_BUTTON_MODE_2=y
```

Replace `holyiot_25008` with `holyiot_25015` for the sensor variant.

### KConfig options

| Option | Default | Description |
|--------|---------|-------------|
| `MATTER_BUTTON_MODE_1` | y | Dimmable switch with accelerometer-based rotational dimming |
| `MATTER_BUTTON_MODE_2` | n | Generic switch with short-press and long-press events |

Both options are mutually exclusive — only one may be set at a time.

---

## Factory Reset

Hold the button for **12 seconds**. The LED will blink white as a warning. Release before 15 seconds to cancel. After 15 seconds the device resets and re-enters commissioning mode.

---

## License

Source code is licensed under the [LicenseRef-Nordic-5-Clause](LICENSE) licence, consistent with the nRF Connect SDK samples this project is based on.

---

*Built and documented for the [YouTube channel](https://www.youtube.com/@uamphome). Full build guide and video linked below.*
