# rtc_gatekeeper

Low‑power ATmega328P gate‑timer system with DS3231 RTC, LCD UI, deep sleep, countdown mode, and daily scheduled triggers.

This repository contains **two firmware variants**, each supporting the same timing/UI/sleep system but using different latch‑actuation hardware:

- **Servo‑driven latch**  
  (`GateTimer_ServoLatch_RTC.ino`)
- **Pulsed nitinol/SMA latch**  
  (`GateTimer_NitinolLatch_RTC.ino`)

---
## IMPORTANT HARDWARE NOTE — LCD Backlight Resistor

If the LCD backlight is powered from **pin A3**, which is switched on/off for low‑power sleep.

**A current‑limiting resistor is REQUIRED** between A3 and the LCD backlight anode.

- Typical value: **1500 Ω, Vcc 4.2V**
- Purpose: Prevent excessive current draw from A3  
- Without this resistor, A3 may be overloaded or the LCD backlight may draw too much current during wake cycles.

---

## Firmware Variants

### 1. Servo Latch  
**File:** `GateTimer_ServoLatch_RTC.ino`

Designed for a mechanism driven by a **single hobby servo**.

Features:
- Adaptive servo‑speed control based on Vcc drop  
- Automatic torque‑boost sequences for cold/sticky mechanisms  
- Multi‑attempt open with safe close + overload detection  
- Detailed diagnostics (avg drop, max drop, adaptive threshold, last success/fail stats)

### 2. Nitinol/SMA Latch  
**File:** `GateTimer_NitinolLatch_RTC.ino`

Designed for a **pulsed nitinol wire** latch.

Features:
- Pulse‑duration control (100–1000 ms)  
- Voltage‑drop‑based open detection  
- Automatic threshold adaptation  
- Multi‑attempt retry scheduling with increasing pulse duration  
- Diagnostics for avg drop, max drop, open time, quickest/slowest opens

Both variants share the same UI, RTC timing, sleep behaviour, and daily trigger logic.

---

## Overview

GateKeeper is a low‑power, RTC‑driven gate timer built around the ATmega328P and DS3231.  
It supports:

- Countdown‑based latch activation  
- Up to five daily scheduled gate‑open times  
- Deep sleep with watchdog wakeups  
- A 16×2 LCD and four‑button interface  
- Variant‑specific latch diagnostics (servo or nitinol)

---

## Shared Features

### Countdown Timer
- Set days, hours, minutes, seconds  
- Latch activation when timer expires  
- Actuation method depends on firmware variant

### Daily Gate Times
- Five independent triggers  
- Hour, minute, enabled state  
- Auto‑reset after the minute passes

### RTC‑Based Timing
- DS3231 used for all timing  
- Accurate countdown expiry  
- Accurate daily triggers  
- Detects long sleep and shows welcome message

### User Interface
- 16×2 HD44780 LCD  
- LCD powered from A3 allowing full shutdown during sleep  
- Four buttons: Up, Down, Left, Right  
- Menu system:
  - Home  
  - Mode Select  
  - Countdown Setup  
  - Gate Times Setup  
  - Options  
  - Clock Set  
  - Diagnostics  
  - Reset confirmations  

### Power Management
- Deep sleep after configurable timeout  
- LCD fully powered down  
- LCD pins set to high‑impedance  
- ADC, comparator, SPI, TWI, Timer1, Timer2 disabled  
- Wake sources:
  - Up button  
  - Down button  
  - Watchdog tick  
- Welcome message after long sleep

---

## Hardware

### Microcontroller
ATmega328P (bare chip or Arduino‑compatible)

### Modules
- DS3231 RTC  
- 16×2 HD44780 LCD

### Inputs
- Up button on pin 3 (INT1)  
- Down button on pin 2 (INT0)  
- Left button on A1  
- Right button on A0  

### Outputs (Variant‑Dependent)

| Variant | Output Pin | Description |
|--------|-------------|-------------|
| **Servo** | Pin 5 | Servo signal |
|          | A2 | Servo power MOSFET gate |
| **Nitinol** | A2 | Pulse output to SMA wire |

### Shared Outputs
- LCD VCC control on A3  
- I²C SDA/SCL on A4/A5  

---

## Menu System

### Home
- Alternates between battery voltage, current time, and next gate time  
- Shows countdown status if active  
- Right enters Mode Select  
- Left enters Diagnostics

### Mode Select
- Gate Times  
- Countdown  
- Options  
- Set Clock  

### Countdown Setup
- Adjust days/hours/minutes/seconds  
- Start with Up or Down on “Go”

### Gate Times
- Five triggers  
- Hour, minute, enabled state  

### Options
Servo variant:
- Sleep timeout  
- Overload drop threshold  
- Reset countdown  
- Reset gate times  
- Exit  

Nitinol variant:
- Sleep timeout  
- Pulse duration  
- Voltage threshold  
- Reset countdown  
- Reset gate times  
- Exit  

### Clock Set
- Adjust hours/minutes/seconds  
- Apply with Right  

### Diagnostics
Variant‑specific latch diagnostics (servo or nitinol)

---

## Sleep and Wake Behaviour

- Device sleeps after the configured timeout  
- LCD power removed  
- Wake via Up/Down button or watchdog tick  
- Welcome message shown after extended sleep  

---

## Repository Structure

```
/README.md
/GateTimer_ServoLatch_RTC.ino
/GateTimer_NitinolLatch_RTC.ino
```

---

## Notes

- Both variants use the same menu/UI codebase for consistency.  
- Choose the firmware that matches your latch hardware.  
- Servo variant includes adaptive torque and safe‑close logic.  
- Nitinol variant includes pulse‑based open detection and retry scheduling.
