# Audio → Servo + Plasma 2040 LED Daemon
### Real-time audio-driven motion + LED light for Raspberry Pi installations

This repository contains a Python application that turns **audio playback** into:

- **Analog-speed servo control** (PWM duty + direction)
- **LED animation** on up to **three Pimoroni Plasma 2040** boards (WS2812 NeoPixels)
- **OSC-controllable “show” playback**, with RMS analysis and JSON export

Designed for installations where **sound, motion, and light** must be tightly synchronized.

---

## Features

### Audio Engine
- Stereo audio file loading (Left/Right independently)
- RMS envelope analysis (4096 window / 2048 hop)
- Auto L/R gain calibration
- Low-pass filtered playback
- Global gain + soft limiter
- Looping / one-shot playback
- JSON show export

### Servo Control
- Hardware PWM on **GPIO18**
- Direction pin on **GPIO19**
- Enable pin on **GPIO20**
- Min/max duty sliders
- Smooth deceleration / stop
- Optional: flip direction on silence
- Optional: direction-linked LED hue (white/red)
- Duty utilization tracker
- Peak duty recorder

### LED System (Plasma 2040)
- Controls 1–3 Pimoroni **Plasma 2040** boards via USB
- Each board runs MicroPython and drives a WS2812/NeoPixel strip
- Adjustable LED brightness floor, max, gamma, smoothing
- Pattern mode (odd/even shuffle)
- Optional red tint when servo reversed
- Always keeps ≥ half pixels lit when active
- LED ramp diagnostic mode (0 → 65% → 0)

### OSC Control
- Custom bind IP + port (e.g. `0.0.0.0:9000`)
- Custom OSC paths for start/stop
- Works with TouchOSC, Max/MSP, Pd, Python-OSC, etc.

---

## Hardware Requirements

### Raspberry Pi
Any Pi 3B/4B/5 works.

| Purpose | GPIO |
|--------|------|
| Servo PWM | **18** |
| Servo Direction | **19** |
| Servo Enable | **20** |

### Servo / Motor Driver
Assumes a driver accepting:

- 3.3 V PWM “speed” input  
- 3.3 V logic direction  
- Enable pin  

PWM frequency: **5 kHz**  
Duty: **0–100%**

(Not an RC servo - this is for analog-speed servo/motor drivers.)

### LED Strips
- WS2812B (NeoPixel), GRB
- 5 V power
- Same pixel count per Plasma board

### Pimoroni Plasma 2040
Each board:

- Appears to the Pi as `/dev/ttyACM*`
- Runs Pimoroni MicroPython firmware with the `plasma` library
- Drives a strip from its DAT pin

Up to **three** boards at once.

---

## Flashing Plasma 2040 Firmware

1. Download UF2 from:  
   https://github.com/pimoroni/pimoroni-pico/releases  
   (Choose the **Plasma 2040 MicroPython firmware**)

2. Put board into bootloader mode:  
   - Hold **BOOT**  
   - Plug USB into Raspberry Pi  
   - Release **BOOT**  
   → Drive `RPI-RP2` appears

3. Flash firmware:  
   - Drag `.uf2` onto `RPI-RP2`

Repeat for each Plasma 2040.

---

## Installing on Raspberry Pi

```bash
sudo apt update
sudo apt install python3-pip python3-tk python3-numpy python3-scipy python3-sounddevice libportaudio2 python3-serial
pip3 install soundfile python-osc
```

---

## Running the Application

Clone the repository and launch the daemon:

```bash
git clone https://github.com/robertoalonsotrillo/70AR.git
cd https://github.com/robertoalonsotrillo/70AR.git
python3 MIX_21_PWM_LED_UPDATED.py
```

The GUI will appear with five main tabs:

### 1. **Select & Scan**
- Load Left/Right audio files  
- Select audio output device  
- Run RMS analysis  

### 2. **Mapping**
- Set servo **min/max duty**  
- Set LED **brightness floor / max**  
- Adjust **gamma** and **smoothing**  
- Behaviour options:
  - Flip servo direction on silence  
  - Use red hue when direction is reversed  
  - Enable pixel shuffle (odd/even alternating pattern)  

### 3. **Play & Hardware**
- Connect Plasma 2040 boards  
- Enable/disable servo driver  
- Start / Stop playback  
- Run LED ramp test  
- Monitor **servo utilization**  
- Monitor **LED brightness**  

### 4. **Audio Gain**
- Adjust Left/Right channel volumes  
- Apply global gain (dB)  
- Enable/disable soft limiter  

### 5. **Export JSON**
Exports a complete show configuration, including:
- RMS envelope and percentiles  
- Servo mapping parameters  
- LED mapping parameters  
- Behaviour options  
- Hardware configuration (LED count, GPIO pins, etc.)  

All GUI settings are automatically saved to:

```text
~/.showplayer_settings.json
