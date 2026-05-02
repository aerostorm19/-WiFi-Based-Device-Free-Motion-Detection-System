# 📡 WiFi-Based Device-Free Motion Detection System
## Complete Technical Reference — Raspberry Pi 3 × RSSI

> **Hardware:** Raspberry Pi 3 Model B (BCM43438 WiFi chipset)  
> **Technique:** RSSI-based passive RF sensing — no wearables, no cameras  
> **Constraint:** CSI extraction (nexmon_csi) is NOT supported → RSSI only

---

# TABLE OF CONTENTS

1. [Physical Setup & Hardware Connection](#1-physical-setup--hardware-connection)
2. [Raspberry Pi OS & Network Configuration](#2-raspberry-pi-os--network-configuration)
3. [Software Installation](#3-software-installation)
4. [Running the System](#4-running-the-system)
5. [System Architecture](#5-system-architecture)
6. [Algorithm Deep Dives](#6-algorithm-deep-dives)
   - 6.1 [RSSI Acquisition Pipeline](#61-rssi-acquisition-pipeline)
   - 6.2 [Outlier Rejection (Z-Score Filtering)](#62-outlier-rejection-z-score-filtering)
   - 6.3 [Rolling Mean Smoother](#63-rolling-mean-smoother)
   - 6.4 [Sliding Window Standard Deviation (Motion Score)](#64-sliding-window-standard-deviation-motion-score)
   - 6.5 [Adaptive Threshold (EMA-Based)](#65-adaptive-threshold-ema-based)
   - 6.6 [Motion State Machine with Hold Timer](#66-motion-state-machine-with-hold-timer)
   - 6.7 [Ornstein-Uhlenbeck Simulator](#67-ornstein-uhlenbeck-simulator)
7. [Data Logging & CSV Schema](#7-data-logging--csv-schema)
8. [Real-Time Visualization](#8-real-time-visualization)
9. [GPIO Output](#9-gpio-output)
10. [Calibration & Threshold Tuning](#10-calibration--threshold-tuning)
11. [Experimental Protocol](#11-experimental-protocol)
12. [Physics of RSSI-Based Motion Detection](#12-physics-of-rssi-based-motion-detection)
13. [Troubleshooting](#13-troubleshooting)
14. [Quick Reference Card](#14-quick-reference-card)

---

# 1. Physical Setup & Hardware Connection

## 1.1 What You Need

| Item | Quantity | Notes |
|------|----------|-------|
| Raspberry Pi 3 Model B | 1 | Onboard BCM43438 WiFi/BT chip |
| MicroSD card (≥16 GB, Class 10) | 1 | For RPi OS |
| WiFi Router | 1 | 2.4 GHz band — **the signal source** |
| USB power supply (5V 2.5A) | 1 | Powers the Pi |
| Ethernet cable | 1 (optional) | For initial SSH setup |
| LED (3mm or 5mm) | 1 (optional) | Motion indicator |
| 330 Ω resistor | 1 (optional) | Current limiter for LED |
| Buzzer (5V active) | 1 (optional) | Audio motion alert |
| Jumper wires | A few | GPIO connections |
| Laptop/PC | 1 | For SSH / monitoring |

---

## 1.2 Physical Placement — The Most Critical Step

The **geometry** of router ↔ monitored zone ↔ Pi determines detection quality.

```
╔══════════════════════════════════════════════════════════════════╗
║                     OPTIMAL LAYOUT                               ║ 
║                                                                  ║
║   [ROUTER]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[Raspberry Pi]     ║
║      │           Direct LoS path (2–5 m)            │            ║
║      │                    ↑                          │           ║
║      │         Person walks HERE (in path)           │           ║
║      │                                               │           ║
║      └───────── Reflected/scattered paths ───────────┘           ║
║                (multipath — the noise we measure)                ║
╚══════════════════════════════════════════════════════════════════╝
```

### Rules of Thumb

| Parameter | Recommended Value | Why |
|-----------|------------------|-----|
| Router ↔ Pi distance | 2–5 metres | Too close → small Δ RSSI; too far → weak signal & noise |
| Height of devices | 0.8–1.2 m (tabletop/shelf) | Matches human body centre of mass |
| Monitored corridor width | ≤ 3 m | Wider = weaker perturbation per crossing |
| Room type | Empty hallway or doorway | Reflections from furniture add background noise |
| Walls between devices | 0 (same room) | Each wall reduces signal and sensitivity |

### Why Line-of-Sight Matters

When a person steps into the direct path between router and Pi:
- They **absorb** ~3–6 dB of the 2.4 GHz signal (water content of the body)
- They **scatter** energy in new directions
- They **reflect** some energy back, creating additional multipath echoes

All three effects change the constructive/destructive interference pattern at the Pi's antenna, producing a measurable RSSI shift of 3–12 dBm in typical indoor environments.

---

## 1.3 GPIO Wiring 

```
Raspberry Pi GPIO Header (BCM numbering)

                    3V3  (1) (2)  5V
                  GPIO2  (3) (4)  5V
                  GPIO3  (5) (6)  GND ──────────┐
                  GPIO4  (7) (8)  GPIO14        │
                    GND  (9)(10)  GPIO15        │
  [LED+]──[330Ω]─GPIO17 (11)(12) GPIO18         │
                 GPIO27 (13)(14) GND ────────────┤
  [BZR+]────────GPIO27 (13)                      │
                  ...                             │
  [LED-] ─────────────────────────────────────────┘  (GND)
  [BZR-] ─────────────────────────────────────────   (GND)
```

**LED wiring:**
```
GPIO17 (pin 11) → [330 Ω resistor] → [LED anode (+)] → [LED cathode (−)] → GND (pin 6)
```

**Buzzer wiring (active 5V buzzer):**
```
GPIO27 (pin 13) → [Buzzer + terminal]
GND    (pin 9)  → [Buzzer − terminal]
```

> ⚠️ Always use a resistor with an LED. Without it, the GPIO pin sources ~16 mA max, and the LED will either dim or damage the Pi.

---

# 2. Raspberry Pi OS & Network Configuration

## 2.1 Flash Raspberry Pi OS

1. Download **Raspberry Pi Imager** from `raspberrypi.com/software`
2. Select: **Raspberry Pi OS Lite (64-bit)** (no desktop needed for headless) or **Raspberry Pi OS (with Desktop)** if you want the live graph on the Pi itself
3. In the Imager settings (gear icon ⚙️), configure:
   - **Hostname:** `motionpi`
   - **Enable SSH:** Yes (password or key auth)
   - **WiFi SSID / Password:** your router's credentials
   - **Locale / timezone:** set appropriately
4. Flash to SD card, insert into Pi, power on

## 2.2 Find the Pi on Your Network

```bash
# From your laptop:
ping motionpi.local          # mDNS — works on Linux/macOS, sometimes Windows

# Or scan your subnet:
nmap -sn 192.168.1.0/24 | grep -A1 "Raspberry"
```

## 2.3 SSH Into the Pi

```bash
ssh pi@motionpi.local        # default user is 'pi' if you kept it
# or
ssh pi@192.168.1.XXX         # replace with actual IP
```

## 2.4 Connect Pi to WiFi (if not done via Imager)

```bash
# On the Pi:
sudo raspi-config
# → System Options → Wireless LAN → enter SSID & password

# Or manually:
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Add to the file:
```
network={
    ssid="YourRouterName"
    psk="YourWiFiPassword"
    key_mgmt=WPA-PSK
}
```

Then apply:
```bash
sudo wpa_cli -i wlan0 reconfigure
```

## 2.5 Verify WiFi Association

```bash
iwconfig wlan0          # shows ESSID, signal level, bit rate
iw dev wlan0 link       # cleaner output — shows RSSI in dBm
ip addr show wlan0      # shows assigned IP address
```

Expected output from `iw dev wlan0 link`:
```
Connected to AA:BB:CC:DD:EE:FF (on wlan0)
        SSID: MyRouter
        freq: 2412
        RX: 12345 bytes (100 packets)
        TX: 5678 bytes (50 packets)
        signal: -62 dBm
        tx bitrate: 72.2 MBit/s MCS 7
```

The **`signal: -62 dBm`** line is exactly what our system reads.

## 2.6 Keep WiFi Associated (Disable Power Management)

By default, Linux may put the WiFi radio into sleep to save power. This causes RSSI read failures.

```bash
# Check current power management state:
iwconfig wlan0 | grep Power

# Disable power management permanently:
sudo nano /etc/rc.local
```

Add before `exit 0`:
```bash
iwconfig wlan0 power off
```

Or create a systemd service:
```bash
sudo nano /etc/systemd/system/disable-wifi-pm.service
```
```ini
[Unit]
Description=Disable WiFi Power Management
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iwconfig wlan0 power off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable disable-wifi-pm.service
sudo systemctl start  disable-wifi-pm.service
```

---

# 3. Software Installation

## 3.1 Update System

```bash
sudo apt update && sudo apt upgrade -y
```

## 3.2 Install System Tools

```bash
sudo apt install -y python3-pip python3-venv iw wireless-tools git
```

## 3.3 Transfer Project Files

**Option A — Copy from laptop via SCP:**
```bash
# From your laptop (in the wifi_motion_system/ folder):
scp -r wifi_motion_system/ pi@motionpi.local:~/
```

**Option B — Clone from a git repo (if you push it there):**
```bash
git clone https://github.com/yourname/wifi-motion-system.git
cd wifi-motion-system
```

**Option C — Create files directly on Pi via nano.**

## 3.4 Install Python Dependencies

```bash
cd ~/wifi_motion_system

# Option A: system-wide (simplest for RPi)
pip3 install matplotlib numpy --break-system-packages

# Option B: virtual environment (cleaner)
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 3.5 Verify Installation

```bash
# Test RSSI reading works:
iw dev wlan0 link | grep signal

# Test Python can read it:
python3 -c "
from rssi_collector import get_rssi
print('RSSI =', get_rssi('wlan0'), 'dBm')
"
```

## 3.6 Display Setup (for Live Graph)

**If running with a monitor attached to the Pi:**
- The matplotlib window appears on the HDMI display automatically.

**If running headless via SSH with X11 forwarding:**
```bash
# Connect with X forwarding:
ssh -X pi@motionpi.local
# Then run normally — the graph window appears on your laptop
python3 wifi_motion_system.py
```

**If running on a remote server/Pi with no display:**
```bash
python3 wifi_motion_system.py --no-gui
# The system still logs CSV and prints to terminal
```

**If using VNC for remote desktop on Pi:**
```bash
sudo raspi-config
# → Interface Options → VNC → Enable
# Connect with VNC Viewer from your laptop
```

---

# 4. Running the System

## 4.1 Basic Usage

```bash
cd ~/wifi_motion_system

# Normal run (live graph + CSV logging):
python3 wifi_motion_system.py

# Offline demo (no WiFi needed):
python3 wifi_motion_system.py --simulate

# Headless (SSH/no display):
python3 wifi_motion_system.py --no-gui

# Custom threshold:
python3 wifi_motion_system.py --threshold 3.0

# Custom sample rate (faster = more responsive, heavier CPU):
python3 wifi_motion_system.py --interval 0.2

# All options combined:
python3 wifi_motion_system.py --threshold 2.5 --interval 0.3 --log session1.csv
```

## 4.2 All CLI Options

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--simulate` | flag | off | Use synthetic RSSI generator instead of real WiFi |
| `--no-gui` | flag | off | Disable matplotlib window (headless / SSH mode) |
| `--threshold N` | float | 2.0 | Motion detection threshold (σ in dBm) |
| `--interval N` | float | 0.5 | Seconds between RSSI samples |
| `--log FILE` | string | rssi_log.csv | CSV output filename |
| `--interface IF` | string | wlan0 | WiFi interface name |

## 4.3 Console Output Explained

```
[14:22:01.340] RSSI:  -63.0 dBm | Smoothed:  -62.84 | Score (σ): 0.421 | Thr: 2.00 | ✅ No Motion
     │                │                  │                   │              │          └─ Status
     │                │                  │                   │              └─ Current adaptive threshold
     │                │                  │                   └─ Motion score = std dev of recent RSSI
     │                │                  └─ Short-window rolling mean (noise reduced)
     │                └─ Raw RSSI reading from hardware
     └─ Timestamp (HH:MM:SS.mmm)
```

## 4.4 Stopping the System

Press **Ctrl+C** — the system will:
1. Set the stop event (graceful shutdown)
2. Wait for the acquisition thread to finish
3. Flush and close the CSV log
4. Reset GPIO pins
5. Print a session summary

```
[INFO] Shutting down…
[INFO] Session complete.
       Samples collected : 1247
       Motion events     : 12
       Log file          : rssi_log.csv  (1247 rows)
       Goodbye.
```

---

# 5. System Architecture

## 5.1 Module Map

```
wifi_motion_system/
│
├── config.py                ← Central parameter store (edit this to tune)
│
├── rssi_collector.py        ← Hardware abstraction layer
│   ├── get_rssi()           ← Tries iw → iwconfig → /proc/net/wireless
│   ├── RSSISimulator        ← Ornstein-Uhlenbeck synthetic signal
│   └── get_rssi_auto()      ← Picks real or simulated based on config
│
├── motion_detector.py       ← Core DSP & decision engine
│   ├── _is_outlier()        ← Z-score gating on raw RSSI history
│   ├── _smooth()            ← Rolling mean filter
│   ├── _compute_std()       ← Sliding window standard deviation
│   ├── _update_adaptive()   ← EMA-based threshold adaptation
│   └── update()             ← Main pipeline: raw → MotionResult
│
├── data_logger.py           ← CSV writer (append-mode, buffered)
│
├── gpio_controller.py       ← LED/buzzer driver (RPi.GPIO wrapper)
│
└── wifi_motion_system.py    ← Orchestrator
    ├── acquisition_loop()   ← Background thread: read → detect → log → GPIO
    ├── build_figure()       ← Matplotlib dashboard layout
    ├── animate()            ← FuncAnimation callback (graph update)
    └── main()               ← CLI parsing, thread start, shutdown hook
```

## 5.2 Data Flow

```
                     ┌──────────────────────────────────────────────┐
                     │           ACQUISITION THREAD                 │
                     │                                              │
  WiFi Hardware ─────► get_rssi_auto()                             │
  (or Simulator)     │        │                                     │
                     │        ▼ raw RSSI (dBm float)                │
                     │   MotionDetector.update()                    │
                     │        │                                     │
                     │    ┌───▼───────────────────────────────┐     │
                     │    │  1. raw_buf.append(rssi)          │     │
                     │    │  2. outlier_check(raw_buf)        │     │
                     │    │  3. smooth_buf.append(rssi)       │     │
                     │    │  4. smoothed = mean(smooth_buf)   │     │
                     │    │  5. window.append(smoothed)       │     │
                     │    │  6. σ = stdev(window)             │     │
                     │    │  7. adaptive_thr update           │     │
                     │    │  8. state machine decision        │     │
                     │    └───┬───────────────────────────────┘     │
                     │        │ MotionResult                        │
                     │        ├──► DataLogger.log() → CSV file      │
                     │        ├──► GPIOController.set_motion()      │
                     │        └──► shared deques (plots/console)    │
                     └──────────────────────────────────────────────┘

                     ┌──────────────────────────────────────────────┐
                     │           MAIN THREAD                        │
                     │                                              │
                     │  FuncAnimation (every 500ms)                 │
                     │     ├── reads shared deques                  │
                     │     ├── redraws RSSI trace                   │
                     │     ├── redraws motion score trace           │
                     │     └── updates status & stats panels        │
                     └──────────────────────────────────────────────┘
```

---

# 6. Algorithm Deep Dives

---

## 6.1 RSSI Acquisition Pipeline

### What is RSSI?

RSSI (Received Signal Strength Indicator) is measured in **dBm** (decibel-milliwatts). It represents the power level of the received WiFi signal:

```
P_dBm = 10 × log₁₀(P_mW / 1 mW)
```

Typical indoor values:
- **−30 dBm:** Excellent (device very close to router)
- **−60 dBm:** Good (normal operation, 2–5 m away)
- **−75 dBm:** Weak (far away or behind walls)
- **−90 dBm:** Very poor (near unusable)

### How We Read It

The system tries three Linux methods in order:

**Method 1: `iw dev wlan0 link`**
```bash
$ iw dev wlan0 link
Connected to AA:BB:CC:DD:EE:FF (on wlan0)
        signal: -62 dBm          ← we parse this line
```
Parse regex: `signal:\s*([-\d.]+)\s*dBm`

**Method 2: `iwconfig wlan0`** (fallback)
```bash
$ iwconfig wlan0
wlan0  IEEE 802.11  ESSID:"MyRouter"
       Signal level=-62 dBm     ← we parse this line
```
Parse regex: `Signal level[=:]\s*([-\d.]+)\s*dBm`

**Method 3: `/proc/net/wireless`** (fallback)
```
/proc/net/wireless — kernel direct interface
Inter-| sta-|   Quality        |   Discarded packets
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon
 wlan0: 0000   70.  -60.  -95.
                     ^^^^
                     level field (index 3, may need -256 correction)
```

The level field may be reported as an unsigned byte on older kernels (0–255 instead of −256–0). The code applies a correction: if `raw > 0` then `raw -= 256`.

### Timing

The system calls the OS tool every `SCAN_INTERVAL` seconds (default 0.5s = 2 Hz). This gives a Nyquist frequency of 1 Hz, meaning we can detect motion events that last longer than 1 second — human walking pace is typically 0.5–2 steps/second, well within this bandwidth.

---

## 6.2 Outlier Rejection (Z-Score Filtering)

### Purpose

The Linux WiFi stack occasionally returns glitched RSSI values — sudden ±15 dBm jumps that last only one sample. These are hardware/driver artifacts, not real motion. If included in the detection window, one bad sample can falsely trigger a motion event.

### Algorithm

We maintain a rolling buffer `raw_buf` of the last `WINDOW_SIZE` raw RSSI readings. For each new sample:

```
z = |x_new - μ_raw| / max(σ_raw, MIN_STD_FLOOR)
```

Where:
- `x_new` = current raw RSSI sample
- `μ_raw` = mean of `raw_buf`
- `σ_raw` = standard deviation of `raw_buf`
- `MIN_STD_FLOOR` = 1.5 dBm (critical design choice — explained below)

If `z > OUTLIER_Z_SCORE` (default 4.0), the sample is replaced with `μ_raw`.

### Why the Minimum Std Floor?

Without the floor, a critical feedback loop can form:

```
1. Quiet environment → smooth values → σ_raw ≈ 0.3 dBm
2. Outlier threshold = z × σ = 4.0 × 0.3 = 1.2 dBm
3. A real RSSI change of 1.5 dBm due to motion gets rejected as "outlier"!
4. Window stays quiet → σ_raw stays tiny → threshold stays tiny → loop
```

The 1.5 dBm floor ensures the rejection band is always at least `4.0 × 1.5 = 6.0 dBm`, so only genuine hardware glitches (typically >10 dBm spikes) are rejected, while real motion-induced changes (3–8 dBm) are preserved.

### Key Invariant

> **Outlier detection uses `raw_buf` (raw history), NOT the smoothed window.**
>
> If we used smoothed values for outlier detection, the smoother's attenuation makes σ even smaller, tightening the trap. Using raw values keeps the rejection band calibrated to the true signal variance.

### Math Summary

```
x_in  = raw RSSI sample (dBm)
μ      = mean(raw_buf)
σ      = max(stdev(raw_buf), 1.5)
z_score = |x_in - μ| / σ

x_out = μ         if z_score > 4.0   (outlier rejected)
x_out = x_in      otherwise          (good sample)
```

---

## 6.3 Rolling Mean Smoother

### Purpose

After outlier rejection, the signal still contains high-frequency **thermal noise** from the RF hardware and **fast multipath fluctuations** unrelated to human-scale motion. A short rolling mean filters these out while preserving the slower motion-induced drifts.

### Algorithm

```
smooth_buf = deque(maxlen = SMOOTHING_WINDOW)   # default size = 3
smooth_buf.append(x_out)
x_smoothed = mean(smooth_buf)
```

The rolling mean is a **Finite Impulse Response (FIR)** low-pass filter with a rectangular window:

```
x_s[n] = (1/M) × Σᵢ₌₀^{M-1} x[n-i]
```

Where `M = SMOOTHING_WINDOW`.

### Frequency Response

The frequency response of a length-M rectangular FIR is a **sinc function**:

```
|H(f)| = |sin(πfM) / (M × sin(πf))|
```

For `M = 3` and `f_s = 2 Hz` (sample rate), the 3 dB cutoff is at approximately:

```
f_c ≈ f_s / (2M) = 2 / 6 ≈ 0.33 Hz
```

This means:
- Motion events slower than 0.33 Hz (i.e., lasting > 3 seconds) pass through largely unchanged
- Higher-frequency noise (thermal jitter, fast multipath) is attenuated by up to 9.5 dB

### Trade-off

A larger `SMOOTHING_WINDOW` gives cleaner signal but:
- Introduces more **group delay** (latency): delay ≈ (M−1)/2 × T_sample
- Attenuates short/quick motion bursts

With `M = 3` and `T = 0.5s`, the delay is just 0.5 seconds — acceptable for a demonstration system.

---

## 6.4 Sliding Window Standard Deviation (Motion Score)

### Core Idea

This is the **heart of the detection algorithm**. We maintain a deque `window` of the last `WINDOW_SIZE` smoothed RSSI values. The **standard deviation (σ) of this window is the motion score**.

Why standard deviation? Because:
- During **quiet periods**: RSSI is nearly constant → σ is small (0.1–1.0 dBm)
- During **motion periods**: RSSI fluctuates rapidly → σ is large (2.0–8.0+ dBm)

σ captures the *amount of change* without caring about the *direction* of change, making it robust to:
- Absolute RSSI level (changes with router distance)
- Slow environmental drift (temperature, humidity changes)

### Mathematics

Given a window of `N` smoothed RSSI values `{x₁, x₂, ..., xₙ}`:

**Step 1: Compute the mean**
```
μ = (1/N) × Σᵢ₌₁ᴺ xᵢ
```

**Step 2: Compute sample standard deviation** (Bessel's correction — uses N−1 in denominator, more accurate for small samples)
```
σ = √[ (1/(N-1)) × Σᵢ₌₁ᴺ (xᵢ − μ)² ]
```

**Expanded form:**
```
σ = √[ (Σxᵢ² − N×μ²) / (N-1) ]
```

Python uses `statistics.stdev()` which applies Bessel's correction automatically.

### Why Not Use Variance (σ²)?

Variance is σ². We use σ because:
1. σ has the **same units as RSSI (dBm)**, making threshold setting intuitive ("trigger when variation exceeds 2 dBm")
2. σ² magnifies large outliers more aggressively, making the system more sensitive to residual hardware glitches

### Window Size Effect

| WINDOW_SIZE | Time covered (at 0.5s interval) | Characteristics |
|-------------|--------------------------------|-----------------|
| 5 | 2.5 seconds | Highly responsive, noisy threshold |
| 10 | 5 seconds | Balanced (good for demos) |
| 20 | 10 seconds | Stable threshold, slower to react |
| 40 | 20 seconds | Very stable, misses brief motion |

**Our default of 20** covers 10 seconds: long enough to establish a stable baseline but short enough to respond within 2–3 seconds of motion onset.

### Signal Flow Illustration

```
Time →   0    1    2    3    4    5    6    7    8    9   10   11
RSSI →  -60  -61  -60  -61  -60  -55  -65  -58  -63  -60  -61  -60
                                  ↑                    ↑
                              Motion starts        Motion ends

Window at t=8 (size=5): {-55, -65, -58, -63, -60}
  μ = (-55 - 65 - 58 - 63 - 60) / 5 = -60.2
  σ = √[((-55+60.2)² + (-65+60.2)² + ... ) / 4]
    = √[(27.04 + 23.04 + 4.84 + 7.84 + 0.04) / 4]
    = √[62.8 / 4] = √15.7 ≈ 3.96 dBm   ← Motion detected!

Window at t=3 (quiet, size=5): {-60, -61, -60, -61, -60}
  μ = -60.4
  σ = √[(0.16 + 0.36 + 0.16 + 0.36 + 0.16) / 4] = √[0.3] ≈ 0.55 dBm   ← No motion
```

---

## 6.5 Adaptive Threshold (EMA-Based)

### Problem with a Fixed Threshold

A fixed threshold of, say, 2.0 dBm works well in one room but fails in another:
- **Low-noise room** (concrete walls, sparse furniture): quiet σ ≈ 0.2 dBm → threshold of 2.0 is unnecessarily high → missed detections
- **High-noise room** (many BT devices, appliances): quiet σ ≈ 1.5 dBm → threshold of 2.0 is too close to baseline → false alarms

The adaptive threshold self-adjusts to the local RF environment.

### Algorithm

We track the **Exponential Moving Average (EMA)** of the motion score σ during **quiet periods only**. This EMA estimates the environmental noise floor.

**EMA update rule** (applied only when NOT in motion):
```
EMA[t] = α × σ[t] + (1 − α) × EMA[t-1]
```

Where `α = ADAPTIVE_ALPHA` (default 0.03). This is a **low-pass IIR filter** with time constant:

```
τ = -1 / ln(1 - α) ≈ 1/α = 33 samples ≈ 16 seconds
```

The EMA changes slowly, tracking only gradual environmental shifts (not motion peaks).

**Adaptive threshold:**
```
margin = MOTION_THRESHOLD − CALM_THRESHOLD = 2.0 − 0.8 = 1.2 dBm
adaptive_thr = clamp(EMA + margin, floor, ceiling)

Where:
  floor   = CALM_THRESHOLD + margin/2 = 0.8 + 0.6 = 1.4 dBm (never too sensitive)
  ceiling = MOTION_THRESHOLD × 1.5 = 3.0 dBm (never deaf)
```

**Critical design rule:** The EMA is only updated when:
```
σ[t] < adaptive_thr × 0.8   (clearly in quiet state)
```

This prevents a feedback loop where motion peaks raise the EMA → raise the threshold → prevent detection.

### EMA Mathematics

The EMA at time step `n` can be expanded as:

```
EMA[n] = α × σ[n] + α(1-α) × σ[n-1] + α(1-α)² × σ[n-2] + ...
        = α × Σₖ₌₀ⁿ (1-α)ᵏ × σ[n-k]
```

This is a **geometrically weighted average** — recent samples have exponentially more weight than older ones. With `α = 0.03`:

| Sample age (steps back) | Relative weight |
|------------------------|-----------------|
| 0 (current) | 1.000 |
| 10 steps ago | (0.97)^10 ≈ 0.74 |
| 33 steps ago | (0.97)^33 ≈ 0.37 |
| 66 steps ago | (0.97)^66 ≈ 0.14 |

So the EMA has a "memory" of approximately 33 samples (16 seconds at 0.5s interval).

### State Transition with Adaptive Threshold

```
σ vs adaptive_thr:

         HIGH (σ ≥ adaptive_thr)
                ↓
    ┌─────── MOTION ───────┐
    │    (🚨 detected)     │
    │                      │
    │  Hold timer running  │ ← stays MOTION for ≥ MOTION_HOLD_SECONDS after last trigger
    │                      │
    └──────────────────────┘
                ↓ (σ < calm_thr AND hold timer expired)
         LOW (stable signal)
```

---

## 6.6 Motion State Machine with Hold Timer

### Problem with Direct Thresholding

Without a hold timer, the motion status would toggle rapidly:

```
σ:     0.3  0.3  2.8  0.3  2.9  0.3  2.7  0.4  0.3
State:  NO   NO   YES  NO   YES  NO   YES  NO   NO

→ Rapid toggling, hard to interpret, buzzer would beep 3 times
```

### Hold Timer Solution

Once motion is declared, we hold the "Motion Detected" state for `MOTION_HOLD_SECONDS` (default 2.0) after the **last triggering sample**:

```
σ:        0.3  0.3  2.8  0.3  2.9  0.3  2.7  0.4  0.3  0.3  0.3  0.3
Motion?    NO   NO   YES  YES  YES  YES  YES  YES  YES  YES  NO   NO
                                                           ↑
                              Hold expires 2s after this last trigger (σ=2.7)
```

### State Machine Logic

```python
if σ >= adaptive_thr:
    motion_detected = True
    last_trigger_ts = now           # reset the hold timer

elif motion_detected:
    if (now - last_trigger_ts) >= MOTION_HOLD_SECONDS:
        motion_detected = False     # hold expired → go quiet
    # else: still holding
```

### Why Two Thresholds?

The system uses:
- **MOTION_THRESHOLD** (2.0): to declare motion onset
- **CALM_THRESHOLD** (0.8): not used in the hold-timer version above

This creates **hysteresis** — the system requires σ to drop much further below the trigger threshold before declaring "No Motion". Without hysteresis, threshold crossings at the boundary cause flickering.

A simpler hysteresis version:
```
→ Declare MOTION if σ ≥ MOTION_THRESHOLD
→ Declare CALM  if σ <  CALM_THRESHOLD  AND hold expired
→ Hold previous state if CALM_THRESHOLD ≤ σ < MOTION_THRESHOLD
```

---

## 6.7 Ornstein-Uhlenbeck Simulator

The simulator generates synthetic RSSI that mimics real WiFi behavior without requiring hardware. It uses two processes:

### Quiet Background (Gaussian Drift)

The baseline RSSI drifts slowly using a simple random walk:

```
baseline[t] = baseline[t-1] + direction × U(0, 0.03)
```

Where `U(0, 0.03)` is a uniform random draw, and `direction` flips when the baseline drifts outside [−70, −55] dBm. This mimics slow environmental changes (temperature, humidity).

Background noise:
```
noise[t] = N(0, 0.5)     (Gaussian with σ = 0.5 dBm)
```

### Motion Burst (Ornstein-Uhlenbeck Process)

During motion periods, we add an **Ornstein-Uhlenbeck (OU) process** which models correlated noise — unlike Gaussian noise (which has no memory), the OU process drifts in a sustained direction for several steps before reverting. This matches real RSSI behaviour where a person walking through the signal path causes a sustained drift, not instant random jumps.

The OU stochastic differential equation:
```
dX = θ(μ − X)dt + σ dW
```

In discrete time (Euler-Maruyama approximation):
```
X[t] = X[t-1] + θ(μ − X[t-1]) + σ × N(0, 1)
```

Where:
- `θ = 0.4` (reversion speed — controls how fast signal returns to baseline)
- `μ = 0.0` (long-run mean displacement — motion doesn't permanently shift signal)
- `σ_OU = 3.5 dBm` (noise amplitude — controls motion intensity)

**Interpretation:**
- Small `θ` → slow reversion → long, sustained drifts (like a slow walker)
- Large `θ` → fast reversion → rapid fluctuations (like quick arm waves)
- Larger `σ_OU` → more dramatic RSSI swings

During quiet periods:
```
X[t] = X[t-1] × 0.5    (exponential decay back to zero — "motion ends")
```

### Total Simulated RSSI

```
rssi[t] = baseline[t] + noise[t] + X[t]
```

This produces:
- **Quiet periods:** RSSI ≈ −60 ± 0.5 dBm (low σ, no detection)
- **Motion periods:** RSSI drifts ±3–10 dBm over several seconds (high σ, detection triggered)

---

# 7. Data Logging & CSV Schema

## 7.1 Output File

Every run appends to `rssi_log.csv` (configurable via `--log`). If the file doesn't exist, it's created with a header. If it exists, new rows are appended (so multiple sessions accumulate in one file).

## 7.2 Schema

```csv
timestamp_iso,timestamp_epoch,raw_rssi_dbm,smoothed_rssi_dbm,motion_score_std,motion_detected,status,adaptive_threshold
2025-05-03 14:22:01.340,1746274921.340,-63.0,-62.84,0.4210,0,No Motion,2.0000
2025-05-03 14:22:01.840,1746274921.840,-62.5,-62.80,0.4190,0,No Motion,2.0000
2025-05-03 14:22:05.340,1746274925.340,-57.0,-60.82,2.8740,1,Motion Detected,2.0000
```

| Column | Type | Description |
|--------|------|-------------|
| `timestamp_iso` | string | Human-readable datetime (local timezone) |
| `timestamp_epoch` | float | Unix timestamp (seconds since 1970-01-01) |
| `raw_rssi_dbm` | float | Raw RSSI from hardware, after outlier correction |
| `smoothed_rssi_dbm` | float | Rolling-mean-smoothed RSSI |
| `motion_score_std` | float | σ of sliding window — the detection feature |
| `motion_detected` | int | 1 = motion, 0 = no motion |
| `status` | string | "Motion Detected" or "No Motion" |
| `adaptive_threshold` | float | Current effective detection threshold |

## 7.3 Analysing the CSV in Python

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("rssi_log.csv")
df["timestamp_iso"] = pd.to_datetime(df["timestamp_iso"])
df = df.set_index("timestamp_iso")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)

ax1.plot(df.index, df["raw_rssi_dbm"], alpha=0.5, label="Raw RSSI")
ax1.plot(df.index, df["smoothed_rssi_dbm"], label="Smoothed RSSI")
ax1.fill_between(df.index, df["raw_rssi_dbm"].min(), df["raw_rssi_dbm"].max(),
                  where=df["motion_detected"]==1, alpha=0.2, color="red", label="Motion")
ax1.set_ylabel("RSSI (dBm)")
ax1.legend()

ax2.plot(df.index, df["motion_score_std"], color="green", label="Motion Score σ")
ax2.plot(df.index, df["adaptive_threshold"], color="orange", linestyle="--", label="Threshold")
ax2.set_ylabel("Std Dev (dBm)")
ax2.legend()

plt.tight_layout()
plt.savefig("analysis.png", dpi=150)
plt.show()
```

---

# 8. Real-Time Visualization

## 8.1 Dashboard Layout

```
┌────────────────────────────────────────────────────────────────────┐
│  📡 WiFi RSSI — Real-Time Motion Detection | Raspberry Pi 3        │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  PANEL 1: RSSI vs TIME (top, full width)                           │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  ────────── Raw RSSI (grey)                                  │  │
│  │  ══════════ Smoothed RSSI (blue)                             │  │
│  │  ▓▓▓▓▓▓▓▓▓ Motion regions (red shading)                      │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  PANEL 2: MOTION SCORE vs TIME (middle, full width)                │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  ▁▂▃█ σ fill (green fill under curve)                        │  │
│  │  ─ ─ ─ Adaptive threshold (orange dashed)                    │  │
│  │  ....  Calm threshold (blue dotted)                          │  │
│  │  ▓▓▓▓  Motion zones (red shading where σ > threshold)        │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌────────────────────┐  ┌────────────────────────────────────┐    │
│  │   STATUS PANEL     │  │         STATS PANEL                │    │
│  │                    │  │                                    │    │
│  │  🚨 MOTION         │  │  Uptime:        03:27              │    │
│  │  or                │  │  Samples:       414                │    │
│  │  ✅ CLEAR          │  │  Motion events: 7                  │    │
│  │                    │  │  Interval:      0.5s               │    │
│  │  RSSI: -63.2 dBm   │  │  Window:        20 pts             │    │
│  │  σ: 2.847          │  │                                    │    │
│  └────────────────────┘  └────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────────┘
```

## 8.2 Animation Mechanism

`FuncAnimation` calls `animate()` every `UPDATE_INTERVAL_MS = 500ms`. Since the acquisition thread fills the shared deques independently, the graph shows the latest data available without blocking collection.

Both threads share data through `collections.deque` objects (thread-safe for single-producer/single-consumer use in Python's CPython due to the GIL).

## 8.3 Motion Region Highlighting

Motion regions are shaded with `ax.axvspan()`:
```python
for i in range(len(t)):
    if motion_flag_buf[i]:
        ax.axvspan(t[i], t[i+1], alpha=0.15, color="#f78166")
```

This paints a semi-transparent red band over any time interval where motion was detected.

---

# 9. GPIO Output

## 9.1 LED Behaviour

| System State | LED |
|---|---|
| No motion | OFF |
| Motion detected | ON (steady) |
| System not running / GPIO disabled | OFF |

## 9.2 Buzzer Behaviour

The buzzer fires a **single pulse** on the rising edge of a motion event (when `motion_detected` transitions from False → True). It does not beep continuously during motion — that would be annoying during a long detection period.

## 9.3 Enable GPIO

In `config.py`:
```python
GPIO_ENABLED = True
LED_PIN      = 17   # BCM numbering
BUZZER_PIN   = 27   # BCM numbering
```

Run with sudo (required for GPIO access):
```bash
sudo python3 wifi_motion_system.py
```

---

# 10. Calibration & Threshold Tuning

## 10.1 Step-by-Step Calibration Procedure

**Step 1: Baseline measurement**

Run the system for 2 minutes with no movement in the monitored zone:

```bash
python3 wifi_motion_system.py --no-gui --log baseline.csv
# Wait 2 minutes, Ctrl-C
```

Analyse the baseline σ:
```python
import pandas as pd
df = pd.read_csv("baseline.csv")
print(f"Baseline σ: mean={df.motion_score_std.mean():.3f}, "
      f"max={df.motion_score_std.max():.3f}, "
      f"99th pct={df.motion_score_std.quantile(0.99):.3f}")
```

Typical output:
```
Baseline σ: mean=0.320, max=1.150, 99th pct=0.890
```

**Step 2: Motion measurement**

Run for 2 minutes, walk through the zone 10 times:

```bash
python3 wifi_motion_system.py --no-gui --log motion_test.csv
```

```python
df = pd.read_csv("motion_test.csv")
motion_rows = df[df.motion_score_std > 1.5]
print(f"Motion σ: mean={motion_rows.motion_score_std.mean():.3f}, "
      f"min={motion_rows.motion_score_std.min():.3f}")
```

Typical output:
```
Motion σ: mean=3.840, min=2.100
```

**Step 3: Set thresholds**

```
CALM_THRESHOLD   = baseline_99th + 0.2  = 0.89 + 0.20 = 1.09 → round to 1.1
MOTION_THRESHOLD = motion_min   - 0.3  = 2.10 - 0.30 = 1.80 → round to 1.8
```

**Step 4: Verify**

```bash
python3 wifi_motion_system.py --threshold 1.8
# Walk through zone, confirm detection
```

## 10.2 Common Scenarios and Settings

| Environment | Suggested MOTION_THRESHOLD | Notes |
|---|---|---|
| Quiet office, router 2m away | 1.5–2.0 | Low background noise |
| Home with multiple WiFi devices | 2.5–3.5 | Higher background interference |
| Hallway / corridor | 1.2–1.8 | Excellent geometry, strong perturbation |
| Large open room | 3.0–4.0 | Person may not be in direct LoS path |
| Behind thin walls | 2.0–3.0 | Wall adds fixed attenuation, motion still visible |

---

# 11. Experimental Protocol

## 11.1 Recommended Test Sequence

### Test 1: Baseline (No Motion)
- All people out of the monitored zone for 3 minutes
- Record σ values
- Expected: σ < 1.0 dBm, "No Motion" status throughout

### Test 2: Single Person Walking Through
- One person walks slowly (2–3 s per crossing) between router and Pi
- Repeat 10 times, once every 10 seconds
- Expected: σ spikes to 2–6 dBm during each crossing, "Motion Detected" for each

### Test 3: Standing Still in the Zone
- Person walks in, then stops and stands motionless
- Expected: σ briefly spikes (entry), then drops to near-baseline while standing still

### Test 4: Arm Waving
- Person stands in the zone, waves arms
- Expected: sustained moderate σ (1.5–3.0 dBm) during waving

### Test 5: Multiple People
- Two people walk through simultaneously
- Expected: higher and more sustained σ than single person

### Test 6: Varying Distance
- Move Pi closer to router (1m) and repeat Test 2
- Move Pi further from router (8m) and repeat
- Observe sensitivity change

## 11.2 Data Recording Template

| Test | Duration | Persons | Events Expected | Events Detected | Notes |
|---|---|---|---|---|---|
| Baseline | 3 min | 0 | 0 | | |
| Single walk | 3 min | 1 | 10 | | |
| Standing still | 2 min | 1 | 0 | | |
| Arm wave | 2 min | 1 | continuous | | |
| Double walk | 3 min | 2 | 10 | | |

## 11.3 Good Demonstration Script (3-minute live demo)

```
[0:00]  "No one in the room — watch the signal, it's calm."
        → σ ≈ 0.2–0.5 dBm, "No Motion" displayed

[0:30]  "I'll now walk between the router and the Pi."
        → Walk through once. σ spikes, "🚨 Motion Detected" appears.

[0:45]  "Notice how it returns to calm within 2–3 seconds."
        → σ drops, status returns to "✅ No Motion"

[1:00]  "I'll now walk back and forth continuously for 20 seconds."
        → σ stays elevated, continuous motion detection.

[1:20]  "Now I'll stand very still in the zone."
        → σ drops despite person being present (only movement matters!)

[1:40]  "Now I'll wave my arms."
        → σ rises again — small movements are detectable too.

[2:00]  "You can see in the CSV log every event was recorded with a timestamp."
        → Open rssi_log.csv, point to the motion_detected column.
```

---

# 12. Physics of RSSI-Based Motion Detection

## 12.1 Radio Propagation Basics

WiFi operates at 2.4 GHz, with wavelength:

```
λ = c / f = 3×10⁸ m/s / 2.4×10⁹ Hz ≈ 12.5 cm
```

The human body, being largely water (relative permittivity ε_r ≈ 80, conductivity σ_e ≈ 1.5 S/m at 2.4 GHz), is:
- Highly **absorptive**: attenuates the direct signal by 3–6 dB
- Highly **reflective**: creates new multipath components
- A strong **scatterer**: diffracts energy around the body outline

## 12.2 Multipath Propagation

In an indoor environment, the received signal is the **superposition** of:
- One direct path (Line-of-Sight, LoS)
- Multiple reflected paths (off walls, floor, ceiling, furniture)
- Diffracted paths (around corners, edges)

```
r(t) = Σₖ aₖ × s(t − τₖ) × e^{j(2πf_c τₖ)}
```

Where:
- `aₖ` = amplitude of k-th path
- `τₖ` = delay of k-th path
- `f_c` = carrier frequency (2.4 GHz)
- `s(t)` = transmitted signal

The received power is:
```
P_rx = |Σₖ aₖ × e^{jφₖ}|²
```

where `φₖ = 2π f_c τₖ` is the phase of each path.

When a person moves, they:
1. Change some path amplitudes `aₖ` (absorption)
2. Change some path delays `τₖ` (new reflected paths)
3. Add new paths (reflections off the body)

This changes the phasor sum and thus `P_rx` — which appears as an RSSI change.

## 12.3 Why RSSI Varies Even Without Motion

Several factors cause baseline RSSI noise:
- **Thermal noise** in the receiver frontend: typically ±0.5 dBm
- **AGC quantisation**: the WiFi chip's gain control reports RSSI in 0.5–1 dBm steps
- **Other WiFi devices** transmitting on nearby channels (co-channel and adjacent-channel interference)
- **Appliances** (microwaves at 2.45 GHz, baby monitors, Bluetooth) causing ISM-band congestion
- **Temperature** changing the RF amplifier characteristics by ~0.02 dB/°C

Our detection threshold must be set above this background noise floor.

## 12.4 Why RSSI Varies With Motion

A person walking through the signal path changes the RSSI through three mechanisms:

**Mechanism 1: Direct signal attenuation (−3 to −6 dB)**
The body absorbs and scatters some of the LoS energy. This appears as a sudden dip in RSSI when the person blocks the path.

**Mechanism 2: Phase shift in reflected paths**
New reflections off the body's moving surface arrive at the receiver with different delays. Depending on whether they add constructively or destructively to the existing multipath, RSSI can go up or down — sometimes the person causes a +2 dBm spike rather than a dip.

**Mechanism 3: Doppler smearing**
A moving reflector causes slight frequency shifts in reflected signals (Doppler effect):
```
f_doppler = 2 × v × f_c × cos(θ) / c
```
For a person walking at v = 1.4 m/s toward the Pi:
```
f_d ≈ 2 × 1.4 × 2.4×10⁹ × 1 / 3×10⁸ ≈ 22 Hz
```
This Doppler shift causes the interference pattern to vary at ~22 Hz, producing the characteristic RSSI fluctuations at walking pace.

## 12.5 Comparison: RSSI vs CSI

| Feature | RSSI | CSI |
|--------|------|-----|
| What it measures | Total received power (scalar) | Per-subcarrier amplitude + phase (vector) |
| Information content | Low (1 number per sample) | High (56–256 complex numbers per sample) |
| Hardware needed | Any WiFi adapter | Specific chipsets + patched drivers (e.g. nexmon, Atheros) |
| Pi 3 BCM43438 support | ✅ Yes | ❌ No |
| Sensitivity to motion | Moderate | High |
| Noise robustness | Lower (single measurement) | Higher (spatial diversity across subcarriers) |
| Typical detection range | 1–5 m LoS | 1–8 m, through walls |

Our RSSI approach is the correct and only choice for the BCM43438 chipset.

---

# 13. Troubleshooting

## 13.1 RSSI Reading Failures

**Symptom:** `[WARN] RSSI read failed — check that wlan0 is up`

**Causes & Fixes:**

```bash
# Check WiFi is up:
ip link show wlan0
# Should show "UP" state

# If down:
sudo ip link set wlan0 up

# Check it's associated to an AP:
iw dev wlan0 link
# Must show "Connected to..." — not "Not connected."

# If not connected, reconnect:
sudo wpa_cli -i wlan0 reassociate

# Check if iw is installed:
which iw
sudo apt install iw

# Test manual read:
iw dev wlan0 link | grep signal
```

## 13.2 Always Shows "Motion Detected"

**Causes:**
- Threshold too low for the noise environment
- Background RF interference (nearby appliances, many WiFi networks)
- Power management causing burst polling with stale cached values

**Fixes:**
```bash
# Raise threshold:
python3 wifi_motion_system.py --threshold 4.0

# Disable WiFi power management:
sudo iwconfig wlan0 power off

# Increase window size for more stable σ:
# In config.py: WINDOW_SIZE = 30
```

## 13.3 Never Detects Motion

**Causes:**
- Person not in the direct LoS path
- Threshold too high
- Window size too large (motion signal is diluted by quiet samples)

**Fixes:**
```bash
# Lower threshold:
python3 wifi_motion_system.py --threshold 1.0

# Reduce window size in config.py:
# WINDOW_SIZE = 10

# Verify geometry: router ← person → Pi alignment
```

## 13.4 No Graph Window (GUI Issues)

```bash
# Check DISPLAY variable is set:
echo $DISPLAY
# Should show ":0" (local) or "localhost:10.0" (X forwarding)

# If empty, set it:
export DISPLAY=:0

# For SSH without X forwarding, use headless mode:
python3 wifi_motion_system.py --no-gui

# Install missing packages:
sudo apt install python3-tk
```

## 13.5 ImportError / ModuleNotFoundError

```bash
# Install dependencies:
pip3 install matplotlib numpy --break-system-packages

# If using venv:
source venv/bin/activate
pip install -r requirements.txt

# Check Python version:
python3 --version   # Must be 3.8+
```

---

# 14. Quick Reference Card

## Config Parameters at a Glance

```python
# ── In config.py ──────────────────────────────────────────────────────

WIFI_INTERFACE        = "wlan0"  # WiFi interface to monitor
SCAN_INTERVAL         = 0.5     # Seconds between RSSI reads (2 Hz)

WINDOW_SIZE           = 20      # Sliding window depth (10 seconds at 0.5s)
SMOOTHING_WINDOW      = 3       # Rolling mean width (noise filter)
OUTLIER_Z_SCORE       = 4.0     # Z-score cutoff for hardware glitch rejection

MOTION_THRESHOLD      = 2.0     # σ (dBm) → trigger "Motion Detected"
CALM_THRESHOLD        = 0.8     # σ (dBm) → return to "No Motion"
MOTION_HOLD_SECONDS   = 2.0     # Hold "Motion" this many seconds after last trigger

ADAPTIVE_THRESHOLD    = True    # Auto-tune threshold to environment
ADAPTIVE_ALPHA        = 0.03    # EMA learning rate (smaller = slower adaptation)

GPIO_ENABLED          = False   # Set True + wire up LED/buzzer
LED_PIN               = 17      # BCM pin for LED
BUZZER_PIN            = 27      # BCM pin for buzzer

SIMULATE              = False   # True = offline synthetic RSSI demo
LOG_FILENAME          = "rssi_log.csv"
```

## Algorithm Formula Sheet

```
RSSI reading (dBm):
  P_dBm = 10 × log₁₀(P_mW / 1 mW)

Outlier Z-score:
  z = |x − μ_raw| / max(σ_raw, 1.5)    reject if z > 4.0

Rolling mean smoother (FIR, order M):
  x_s[n] = (1/M) × Σᵢ₌₀^{M-1} x[n-i]

Sample standard deviation (motion score):
  μ = (1/N) Σᵢ xᵢ
  σ = √[ (1/(N-1)) Σᵢ (xᵢ − μ)² ]

EMA (adaptive threshold baseline):
  EMA[t] = α × σ[t] + (1−α) × EMA[t-1]    (only when NOT in motion)

Adaptive threshold:
  adaptive_thr = clamp(EMA + margin, calm+margin/2, motion_thr×1.5)

OU simulator (motion burst):
  X[t] = X[t-1] + θ(0 − X[t-1]) + σ_OU × N(0,1)
  rssi[t] = baseline[t] + N(0,0.5) + X[t]  (during motion)
  rssi[t] = baseline[t] + N(0,0.5)          (during quiet)
```

## Run Commands

```bash
# Live run on Pi:
python3 wifi_motion_system.py

# Offline demo:
python3 wifi_motion_system.py --simulate

# Headless (SSH, no display):
python3 wifi_motion_system.py --no-gui

# Custom threshold + interval + log:
python3 wifi_motion_system.py --threshold 2.5 --interval 0.3 --log run1.csv

# Raspberry Pi with GPIO LED/buzzer:
sudo python3 wifi_motion_system.py   # (sudo needed for GPIO)
```

---

*Document generated for the WiFi-Based Device-Free Motion Detection System*  
*Hardware: Raspberry Pi 3 Model B (BCM43438) | Method: RSSI Variance Analysis*  
*Version 1.0 — May 2025*
