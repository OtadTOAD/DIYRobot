# Hardware & Wiring Notes
## AI Cargo Robot — Raspberry Pi 4B

---

## 1. Power Architecture

### Rule: Separate Power Supplies

| Component | Power Source |
|---|---|
| Raspberry Pi 4B | 20,000mAh Power Bank via USB-A → USB-C |
| L298N Motor Driver | 4× AA Alkaline Battery Pack (6V) |

**Why separate?** Motor current spikes cause voltage dips. If the Pi shares the motor supply it will brownout, reset, or suffer SD card corruption.

### Pi Power Note

The Raspberry Pi 4B uses **USB-C** for power (not Micro-USB — that was Pi 3 and earlier). Use a USB-A to USB-C cable from the power bank. The 10W USB-A port is sufficient for this project since no heavy USB peripherals are attached.

---

## 2. Motor Power — 4× AA Battery Pack

### Why 4× AA (6V)?

| Config | Voltage | Verdict |
|---|---|---|
| 4× AA alkaline | 6V | ✅ Safe — at TT motor rated max |
| 5× AA alkaline | 7.5V | ⚠️ Over spec — risks burning out TT motors |

TT DC gear motors are rated 3V–6V. The L298N itself accepts up to 35V, so the motor is the limiting factor.

Under load, alkaline AAs typically sag to 5–5.5V, which is actually ideal operating voltage for TT motors.

### Using Two 2-Slot Holders in Series

If you have two 2-slot AA battery holders, wire them in series to get 4× AA = 6V:

```
[AA][AA] Holder 1    [AA][AA] Holder 2
   (−) ────────────── (+)
   
Free (−) of Holder 1 → L298N GND
Free (+) of Holder 2 → L298N VCC
```

No need to buy a new 4-slot holder — series connection adds voltages.

---

## 3. L298N Motor Driver Wiring

### L298N Signal Logic

| Signal | Purpose |
|---|---|
| ENA / ENB | PWM speed control (0–100% duty cycle) |
| IN1, IN2 | Channel A direction |
| IN3, IN4 | Channel B direction |

Direction truth table per channel:

| IN1 | IN2 | Result |
|---|---|---|
| HIGH | LOW | Forward |
| LOW | HIGH | Backward |
| LOW | LOW | Coast |
| HIGH | HIGH | Brake |

### GPIO → L298N Connections

```
Raspberry Pi 4B          L298N
─────────────────────────────────
GPIO 12 (Pin 32)  →  ENA   (Hardware PWM ✅)
GPIO  5 (Pin 29)  →  IN1
GPIO  6 (Pin 31)  →  IN2
GPIO 13 (Pin 33)  →  ENB   (Hardware PWM ✅)
GPIO 19 (Pin 35)  →  IN3
GPIO 26 (Pin 37)  →  IN4
GND (any)         →  GND   (shared ground — see Section 4)
```

GPIO 12 and GPIO 13 are hardware PWM pins on the Pi 4B. This ensures stable motor speed even when the CPU is busy running SLAM or the web server.

### L298N → Motors

```
OUT1 + OUT2  →  Left motors  (both left TT motors wired in parallel)
OUT3 + OUT4  →  Right motors (both right TT motors wired in parallel)
```

One L298N output channel drives two TT motors in parallel.

### L298N 5V Output Pin

Leave the L298N onboard 5V output pin **unconnected**. Since the Pi has its own clean power supply, this pin is not needed and connecting it risks back-feeding voltage.

---

## 4. Shared Ground

### Why Shared Ground Is Required

The Pi and L298N use separate power supplies, but they must share a common GND reference. Without it, the L298N has no reference point for the GPIO signals from the Pi and will behave unpredictably.

### How to Wire It

```
4× AA Battery (−)  →  L298N GND pin
Pi GND (any pin)   →  L298N GND pin  (same pin or adjacent GND pin)
```

### Is This Safe?

Yes. GND is a reference point, not a power source. No dangerous current flows through the shared GND wire. Motor current flows entirely through the battery loop:

```
Battery (+) → L298N VCC → motors → L298N GND → Battery (−)
```

The Pi GND wire only carries tiny signal return currents (~mA). It does not receive motor current and will not damage the Pi.

---

## 5. HC-SR04 Ultrasonic Sensor Wiring

### The Voltage Problem

HC-SR04 runs on 5V. Its ECHO pin outputs **5V**. The Raspberry Pi GPIO pins are **3.3V maximum**. Feeding 5V into a GPIO pin can permanently damage the Pi.

The TRIG pin is safe — the Pi outputs 3.3V and the HC-SR04 accepts that as HIGH.

Only the **ECHO pin** needs voltage protection.

### Voltage Divider Circuit (Per Sensor)

```
HC-SR04 ECHO (5V)
        |
       1kΩ
        |
        +──────→ Pi GPIO (~3.3V) ✅
        |
       2kΩ
        |
       GND
```

Math: V_out = 5V × 2kΩ / (1kΩ + 2kΩ) = **3.33V** ✅

### Why Some Videos Skip the Divider

- Some use the **HC-SR04P** variant which natively supports 3.3V
- Some power the sensor from 3.3V (lower range but safe output)
- Some just get lucky — but it slowly degrades GPIO pins over time

For the standard **HC-SR04 + Raspberry Pi**, the voltage divider is mandatory.

### Full Wiring Per Sensor

```
HC-SR04 VCC   →  Pi 5V pin (Pin 2 or Pin 4)
HC-SR04 GND   →  Pi GND
HC-SR04 TRIG  →  Pi GPIO (direct, no divider)
HC-SR04 ECHO  →  1kΩ → Pi GPIO
                      ↘ 2kΩ → GND
```

### All 5 Sensors — GPIO Reference

| Sensor | TRIG GPIO | ECHO GPIO |
|---|---|---|
| Front | GPIO 17 (Pin 11) | GPIO 27 (Pin 13) |
| Right | GPIO 22 (Pin 15) | GPIO 23 (Pin 16) |
| Back | GPIO 24 (Pin 18) | GPIO 25 (Pin 22) |
| Left | GPIO 10 (Pin 19) | GPIO 9 (Pin 21) |
| Down | GPIO 11 (Pin 23) | GPIO 8 (Pin 24) |

You will build **5 identical voltage divider circuits** — one per ECHO pin. A small breadboard or perfboard is recommended to keep them organized.

---

## 6. Sensor Placement

### Horizontal Sensors (Front, Right, Back, Left)

Mounting height: **~12 cm from ground** (approximately 60% of robot body height).

This targets the most dangerous obstacle zone:

```
~70–80 cm  → table tops        (too high, ignore)
~40–50 cm  → chair seats       (too high, ignore)
~10–25 cm  → chair & desk legs ✅ detected at 12 cm
~ 5–10 cm  → floor debris      (too low, mostly ignore)
```

Note: exact height may vary slightly per side depending on chassis mounting points. Fine-tune during physical assembly.

Sensors must be mounted **level** — even a few degrees of tilt causes missed detections or floor reflections.

### Downward Sensor (Drop Detection)

Mounted at the **front-bottom, angled downward**.

| Normal reading | ~15 cm |
|---|---|
| > 23 cm | Floor disappeared → emergency stop |
| < 7 cm | Obstacle below → stop and reverse |

---

## 7. LM393 Wheel Encoder Wiring

### How It Works

The LM393 uses an IR LED + photodetector pair with a slotted disk on the motor shaft. Each slot produces one pulse. Counting pulses gives wheel displacement.

### Wiring

```
LM393 VCC  →  Pi 3.3V or 5V
LM393 GND  →  Pi GND
LM393 OUT  →  Pi GPIO (direct, no voltage divider needed)
```

| Encoder | GPIO |
|---|---|
| Left wheel | GPIO 20 (Pin 38) |
| Right wheel | GPIO 21 (Pin 40) |

No voltage divider needed — LM393 output is compatible with Pi GPIO directly.

### Distance Calculation

```
distance per pulse = wheel circumference / number of slots

Example (65mm wheel, 20 slots):
circumference = π × 65mm = ~204mm
distance per pulse = 204mm / 20 = ~10.2mm per pulse
```

### Heading Calculation

```
left pulses > right pulses  →  turned right
right pulses > left pulses  →  turned left
equal pulses                →  straight line
```

### Practical Note

Align the slotted disk carefully within the sensor gap — misalignment is the most common cause of noisy or missing encoder readings. Use the onboard potentiometer on the LM393 module to tune sensitivity once mounted.

---

## 8. Full System Wiring Summary

```
POWER BANK (20,000mAh USB-A)
  └── USB-A to USB-C ──→ Raspberry Pi 4B (USB-C port)

4× AA BATTERY PACK (6V)
  (+) ──→ L298N VCC
  (−) ──→ L298N GND ←── Pi GND (shared ground reference)

RASPBERRY PI 4B → L298N
  GPIO 12 → ENA    GPIO 5  → IN1    GPIO 6  → IN2
  GPIO 13 → ENB    GPIO 19 → IN3    GPIO 26 → IN4
  GND     → GND (shared)

L298N OUTPUTS → TT MOTORS
  OUT1+OUT2 → Left motors (parallel)
  OUT3+OUT4 → Right motors (parallel)
  5V output → unconnected

HC-SR04 SENSORS (×5)
  VCC  → Pi 5V
  GND  → Pi GND
  TRIG → GPIO (direct)
  ECHO → 1kΩ → GPIO, 2kΩ to GND (voltage divider)

LM393 ENCODERS (×2)
  VCC → Pi 3.3V    GND → Pi GND    OUT → GPIO (direct)
  Left: GPIO 20    Right: GPIO 21

USB CAMERA (UVC)
  USB cable → Pi USB-A port  (appears as /dev/video0)
```

---

*Notes compiled during project design review — hardware & wiring phase.*
