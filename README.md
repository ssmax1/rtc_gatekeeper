# rtc_gatekeeper

Arduino ATmega328P based lock timer with DS3231 RTC.  
Provides countdown timing, daily scheduled triggers, low‑power sleep, and a simple LCD/button interface.

---

## Overview

GateKeeper is a low‑power lock‑timer system built around the ATmega328P and DS3231 RTC.  
It supports countdown‑based locking, daily gate‑time triggers, and deep sleep with watchdog‑based periodic wakeups.  
A 16x2 LCD and four‑button interface provide an on‑device menu system.

---

## Features

**Countdown timer**  
- Set days, hours, minutes, seconds  
- Lock output pulses when the timer expires  
- Pulse duration configurable between 100 and 500 ms  

**Daily gate times**  
- Five independent triggers  
- Each trigger has hour, minute, and enabled state  
- Automatic reset after the minute passes  

**RTC‑based timing**  
- DS3231 used for all timing  
- Accurate countdown expiry  
- Accurate daily triggers  
- Used to detect long sleep and show welcome message  

**User interface**  
- 16x2 HD44780 LCD  
- LCD powered from A3 for full shutdown during sleep  
- Four buttons: Up, Down, Left, Right  
- Full menu system:
  - Home  
  - Mode Select  
  - Countdown Setup  
  - Gate Times Setup  
  - Options  
  - Clock Set  
  - Reset confirmations  

**Power management**  
- Deep sleep after configurable timeout  
- LCD fully powered down  
- LCD pins set to high‑impedance  
- ADC, comparator, SPI, TWI, Timer1, Timer2 disabled  
- Wake sources:
  - Up button  
  - Down button  
  - Watchdog tick  
- Welcome message shown after long sleep  

---

## Hardware

**Microcontroller**  
ATmega328P (bare chip or Arduino‑compatible)

**Modules**  
DS3231 RTC  
16x2 HD44780 LCD

**Inputs**  
Up button on pin 3 (INT1)  
Down button on pin 2 (INT0)  
Left button on A1  
Right button on A0  

**Outputs**  
Lock output on pin 5  
LCD VCC control on A3  
I2C SDA/SCL on A4 and A5  

---

## Menu System

**Home**  
- Alternates between battery voltage and current time  
- Shows countdown status if active  
- Right enters Mode Select  

**Mode Select**  
- Countdown  
- Gate Times  
- Options  
- Set Clock  

**Countdown Setup**  
- Adjust days, hours, minutes, seconds  
- Start with Up or Down on "Go"  

**Gate Times**  
- Five triggers  
- Each with hour, minute, enabled state  

**Options**  
- Sleep timeout (5–30 seconds)  
- Pulse duration (100–500 ms)  
- Reset countdown  
- Reset gate times  
- Exit  

**Clock Set**  
- Adjust hours, minutes, seconds  
- Apply with Right  

---

## Sleep and Wake Behaviour

- Device sleeps after the configured timeout  
- LCD power removed  
- Wake via Up, Down button press (LCD wake), or watchdog tick (system only)  
- Welcome message shown after extended sleep  
