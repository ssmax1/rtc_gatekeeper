// --- Part 1: Includes, Globals, Enums ---

#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

// --- EEPROM Setup ---
const int EEPROM_GATETIMES_ADDR = 0;
const uint8_t EEPROM_VALID_KEY = 0xA5;
const int EEPROM_MAGIC_ADDR    = 100;

// Deferred save tracking driven strictly by WDT
bool eepromPendingSave = false;
int wdtSaveTicks = 0;

// On-board LED Pin
const int ledPin = 13;

// LCD pins
const int rs = 4, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Heart Symbol
const byte heart[8] PROGMEM = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};

// Nitinol pulse latch
const int latchPower = A2;     // pulse + servo power mosfet gate
unsigned long pulseDurationMs = 400;   // adjustable 100–1000ms
bool pulseActive = false;
bool pulseManualActive = false;
unsigned long pulseStart = 0;
DateTime retryAtRTC;
bool retryScheduled = false;
int retryCount = 0;
const int maxRetries = 10;   // global max retries constant
bool nitinolReady = false;

long pulseBaseline = 0;
bool lockOpened = false;
int voltageThreshold = 30;   // mV threshold return for opening

long dropSum = 0;
long dropSamples = 0;
long maxDrop = 0;
unsigned long openTimeMs = 0;

// Diagnostics menu index
int diagIndex = 0;   // 0=outcome,1=success,2=fail,3=temp,4=close,5=exit
enum PulseOutcome {
  OUT_NONE,
  OUT_OPEN_BREAK,
  OUT_OPEN_BREAK_RETRY,
  OUT_LOCK_OPENED,
  OUT_OPENED_RETRY,
  OUT_FAIL_MAX
};

PulseOutcome lastOutcome = OUT_NONE;

// Last success stats
long lastSuccessVcc = 0;
long lastSuccessAvg = 0;
long lastSuccessEnd = 0;
unsigned long lastSuccessOpenMs = 0;

// Last fail stats
long lastFailVcc = 0;
long lastFailAvg = 0;
long lastFailEnd = 0;
unsigned long lastFailPulseMs = 0;

// Quickest successful open
unsigned long bestOpenTime = 999999;
long bestOpenAvg = 0;
long bestOpenEnd = 0;
long bestOpenVcc = 0;

// Slowest successful open
unsigned long worstOpenTime = 0;
long worstOpenAvg = 0;
long worstOpenEnd = 0;
long worstOpenVcc = 0;

// Single Servo
Servo releaseServo;
const int servoPin = 5;        // signal pin
const int servoOpen = 95;     // adjust for your mechanism
const int servoClosed = 47;    // adjust for your mechanism
const int servoDetectThreshold = 20;
bool servoIsOpen = false;   // tracks manual servo state
static bool comboHandled = false;
long OverloadCloseOpenDelta = 0;
long tempMulti = 1;

float currentTempC = 0.0;

// Last success stats
int lastSuccessAttempt = 0;
float lastSuccessTemp = 0.0;

// Last fail stats
int lastFailAttempt = 0;
float lastFailTemp = 0.0;

// Last close stats
long lastClose_avgDrop = 0;
long lastClose_adaptiveThr = 0;
long lastClose_maxDrop = 0;
bool lastClose_overload = false;

// Buttons
const int btnRight = A0;
const int btnLeft  = A1;
const int btnUp    = 3;  // INT1
const int btnDown  = 2;  // INT0

// LCD VCC driven from pin A3
const int lcdVccPin = A3;
unsigned long lastButtonPress = 0;

// State machine
enum MenuState {
  HOME,
  MODE_SELECT,
  SET_COUNTDOWN,
  RUNNING_COUNTDOWN,
  CANCEL_PROMPT,
  SET_CLOCK,
  SET_GATETIMES,
  OPTIONS,
  CONFIRM_RESET_COUNTDOWN,
  CONFIRM_RESET_GATETIMES,
  DIAGNOSTICS
};

MenuState menuState = HOME;
MenuState lastState = HOME;

RTC_DS3231 rtc;

// Watchdog flags and LED blink counter
volatile bool watchdogTick = false;
volatile int wdtCounter = 0;
volatile bool blinkLED = false;
volatile bool statusLEDon = false;

volatile bool wokeFromButton = false;

// Countdown timer
DateTime endTime;
bool lockActive = false;

// Daily triggers (single lock, 5 triggers)
struct DailyTrigger {
  int hour;
  int minute;
  bool enabled;
  bool triggered;
};
DailyTrigger dailyTriggers[5];
int currentTriggerIndex = 0;

// Countdown set values
int setDays    = 0;
int setHours   = 0;
int setMinutes = 0;
int setSeconds = 0;

int fieldIndex = 0;
bool cancelChoice = false;

// Display state
bool displayActive = true;
bool lcdReady = true;

// Message display state
bool showingMessage = false;
unsigned long lastMessageStart = 0;
bool WakeMessageCheck = true;
bool welcomeMessage = false;

// temp time for clock set
int tempHours = 0;
int tempMinutes = 0;
int tempSeconds = 0;

// Alternate display toggle for HOME
static bool showVoltage = true;
static unsigned long lastAlt = 0;

// Extra
static bool needsRefresh = false;
unsigned long sleepTimeoutMs = 10000;   // adjustable between 5000–30000
bool sleeping = false;

// OPTIONS
bool optionsEditMode = false;
int optionsFieldIndex = 0;

// Options help scroll (stored in Flash memory)
const char optionsHelpMsg[] PROGMEM = "Options: Up/Down to change option or values; Right to edit/confirm;   ";
static unsigned int optionsHelpPos = 0;
static unsigned long lastOptionsScroll = 0;
const unsigned long optionsScrollIntervalMs = 300;

// Sleep tracking using RTC
DateTime lastwelcome;

// --- Lock Type Detection Globals & Constants ---
enum LockType {
  LOCK_UNKNOWN,
  LOCK_SERVO,
  LOCK_MONO_PULSE
};

LockType activeLockType = LOCK_UNKNOWN;
const int lockDetectThreshold = 200;

// EEPROM Helper Functions
void saveGateTimesToEEPROM() {
  EEPROM.put(EEPROM_GATETIMES_ADDR, dailyTriggers);
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_VALID_KEY);
}

void loadGateTimesFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_VALID_KEY) {
    EEPROM.get(EEPROM_GATETIMES_ADDR, dailyTriggers);
  } else {
    for (int t = 0; t < 5; t++) {
      dailyTriggers[t] = {0, 0, false, false};
    }
    saveGateTimesToEEPROM();
  }
}


// --- Part 2: Watchdog, ISRs, Setup ---

ISR(WDT_vect) {
  watchdogTick = true;
  WDTCSR |= (1 << WDIE); // Re-arm Watchdog interrupt bit

  wdtCounter++;
  if (wdtCounter >= 8) {
    wdtCounter = 0;
    blinkLED = true;     // Signal loop to blink Pin 13
  }
}

void wakeISR() {
  if (sleeping) {
    wokeFromButton = true;
    WakeMessageCheck = true;
  }  
  lastButtonPress = millis();
}

void setupWatchdog8s() {
  MCUSR = 0;
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // 8s tick
}

void setup() {
  // Pin 13 LED setup
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Servo Power Off On Startup
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, LOW);

  // Load Gate Times from EEPROM
  loadGateTimesFromEEPROM();

  // Power LCD on at startup
  pinMode(lcdVccPin, OUTPUT);
  digitalWrite(lcdVccPin, HIGH);
  delay(50);

  Wire.begin();
  Wire.setWireTimeout(25000, true); // Prevents I2C deadlock freezes after long uptime

  if (!rtc.begin()) {
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("RTC not found!"));
    while (1);
  }

  byte heartBuf[8];
  memcpy_P(heartBuf, heart, 8);

  lcd.begin(16, 2);
  lcd.createChar(0, heartBuf);
  lcd.setCursor(0,0); lcd.write(byte(0)); lcd.print(F(" Welcome   To ")); lcd.write(byte(0));
  lcd.setCursor(0,1); lcd.write(byte(0)); lcd.print(F(" Gate  Keeper ")); lcd.write(byte(0));
  delay(1500);
  lcd.clear();
  lcd.setCursor(0,0);  lcd.write(byte(0)); lcd.print(F("Velkommen  Til")); lcd.write(byte(0));
  lcd.setCursor(0,1);  lcd.write(byte(0)); lcd.print(F(" Portvogteren ")); lcd.write(byte(0));
  delay(1500);
  lcd.clear();

  // Buttons
  pinMode(btnLeft,  INPUT_PULLUP);
  pinMode(btnRight, INPUT_PULLUP);
  pinMode(btnUp,    INPUT_PULLUP);
  pinMode(btnDown,  INPUT_PULLUP);

  lastButtonPress = millis();

  attachInterrupt(digitalPinToInterrupt(btnUp),   wakeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnDown), wakeISR, FALLING);

  setupWatchdog8s();
  sei();

  ADCSRA |= _BV(ADEN);
  for (int i = 0; i < 4; i++) {
      readVcc();
      delay(5);
  }

  menuState = HOME;
  refreshLCD();
}


// --- Part 3: Sleep helpers, staged restore ---

void preparePinsForSleep() {
  cli();

  // Put LCD signal pins high-Z
  pinMode(rs, INPUT);
  pinMode(en, INPUT);
  pinMode(d4, INPUT);
  pinMode(d5, INPUT);
  pinMode(d6, INPUT);
  pinMode(d7, INPUT);

  releaseServo.detach();
  pinMode(servoPin, INPUT);

  // Cut LCD VCC
  pinMode(lcdVccPin, INPUT);
  digitalWrite(lcdVccPin, LOW);
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, LOW);

  // Keep wake buttons active
  pinMode(btnUp,   INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);

  displayActive = false;
  lcdReady = false;

  // turn off peripherals
  ADCSRA &= ~_BV(ADEN);     // ADC off
  ACSR   |= _BV(ACD);       // comparator off
  PRR   |= _BV(PRADC) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2);

  sei();
  delay(20);
}

void showWelcomeAfterLongSleepIfNeeded() {
  DateTime wakeRTC = rtc.now();
  TimeSpan slept = wakeRTC - lastwelcome;

  if (slept.totalseconds() > 300 ) {  // > 5 mins
    byte heartBuf[8];
    memcpy_P(heartBuf, heart, 8);
    lcd.createChar(0, heartBuf);
    lcd.clear();
    lcd.setCursor(0,0); lcd.write(byte(0)); lcd.print(F(" Welcome   To ")); lcd.write(byte(0));
    lcd.setCursor(0,1); lcd.write(byte(0)); lcd.print(F(" Gate  Keeper ")); lcd.write(byte(0));
    delay(1500);
    lcd.clear();
    lcd.setCursor(0,0);  lcd.write(byte(0)); lcd.print(F("Velkommen  Til")); lcd.write(byte(0));
    lcd.setCursor(0,1);  lcd.write(byte(0)); lcd.print(F(" Portvogteren ")); lcd.write(byte(0));
    delay(1500);
  }
} 

void stagedRestoreAfterButtonWake() {
  cli();

  // Re-enable peripherals
  PRR   &= ~(_BV(PRADC) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2));
  ACSR  &= ~_BV(ACD);
  ADCSRA |= _BV(ADEN);

  // Power LCD back on
  pinMode(lcdVccPin, OUTPUT);
  digitalWrite(lcdVccPin, HIGH);
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, LOW);

  sei();
  delay(50);

  // Restore LCD signal pins
  pinMode(rs, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(d4, OUTPUT);
  pinMode(d5, OUTPUT);
  pinMode(d6, OUTPUT);
  pinMode(d7, OUTPUT);

  // Restore buttons
  pinMode(btnLeft,  INPUT_PULLUP);
  pinMode(btnRight, INPUT_PULLUP);
  pinMode(btnUp,    INPUT_PULLUP);
  pinMode(btnDown,  INPUT_PULLUP);

  delay(30);

  lcd.begin(16, 2);
  delay(150);

  lcdReady = true;
  displayActive = true;

  lastButtonPress = millis();
  refreshLCD();
}

int getEnabledTriggerCount() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (dailyTriggers[i].enabled) {
      count++;
    }
  }
  return count;
}


// --- Part 4: Button Logic ---

unsigned long lastPressTime[4]   = {0,0,0,0};
unsigned long pressStartTime[4]  = {0,0,0,0};
unsigned long nextRepeatTime[4]  = {0,0,0,0};
bool          buttonHeld[4]      = {false,false,false,false};

const unsigned long debounceMs       = 150;
const unsigned long initialRepeatDelay = 400;
const unsigned long repeatPeriodSlow   = 150;
const unsigned long repeatPeriodFast   = 60;
const unsigned long accelerateAt       = 1500;

void handleUp();
void handleDown();
void handleLeft();
void handleRight();

void checkButton(int pin, int index, void (*handler)()) {
  unsigned long now = millis();
  bool pressed = (digitalRead(pin) == LOW);

  if (pressed) {
    if (!buttonHeld[index]) {
      if (now - lastPressTime[index] > debounceMs) {
        buttonHeld[index] = true;
        pressStartTime[index] = now;
        nextRepeatTime[index] = now + initialRepeatDelay;

        handler();
        needsRefresh = true;

        lastPressTime[index] = now;
        lastButtonPress = now;
      }
    } else {
      unsigned long heldMs = now - pressStartTime[index];
      unsigned long period = (heldMs >= accelerateAt) ? repeatPeriodFast : repeatPeriodSlow;

      if (now >= nextRepeatTime[index]) {
        handler();
        needsRefresh = true;

        nextRepeatTime[index] += period;
        lastButtonPress = now;
      }
    }
  } else {
    if (buttonHeld[index]) {
      buttonHeld[index] = false;
    }
  }
}


// --- Part 5: Button Handlers ---

void handleUp() {
  switch (menuState) {
    case HOME: break;
    case DIAGNOSTICS:
      diagIndex = (diagIndex + 5) % 6;
      break; 
    case MODE_SELECT:
      fieldIndex = (fieldIndex + 1) % 4;
      break;
    case OPTIONS:
      if (!optionsEditMode) {
        optionsFieldIndex = (optionsFieldIndex + 1) % 7;
      } else {
        if (optionsFieldIndex == 0) {
          sleepTimeoutMs += 1000;
          if (sleepTimeoutMs > 30000) sleepTimeoutMs = 5000;
        } else if (optionsFieldIndex == 1) {
          pulseDurationMs += 50;
          if (pulseDurationMs > 1000) pulseDurationMs = 50;
        } else if (optionsFieldIndex == 2) {
          voltageThreshold += 5;
          if (voltageThreshold > 200) voltageThreshold = 10;
        } else if (optionsFieldIndex == 3) {
          OverloadCloseOpenDelta += 10;
          if (OverloadCloseOpenDelta > 50) OverloadCloseOpenDelta = 0;
        }
      }
      break;
    case CONFIRM_RESET_COUNTDOWN:
    case CONFIRM_RESET_GATETIMES:
      cancelChoice = !cancelChoice;
      break;
    case SET_COUNTDOWN:
      if (fieldIndex == 4) {
        DateTime now = rtc.now();
        endTime = now + TimeSpan(setDays, setHours, setMinutes, setSeconds);
        lockActive = true;
        menuState = RUNNING_COUNTDOWN;
      } else {
        if (fieldIndex == 0)      setDays    = (setDays + 1) % 1000;
        else if (fieldIndex == 1) setHours   = (setHours + 1) % 24;
        else if (fieldIndex == 2) setMinutes = (setMinutes + 1) % 60;
        else if (fieldIndex == 3) setSeconds = (setSeconds + 1) % 60;
      }
      break;
    case SET_GATETIMES: {
      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];
      if (fieldIndex == 0)      tr.hour = (tr.hour + 1) % 24;
      else if (fieldIndex == 1) tr.minute = (tr.minute + 1) % 60;
      if ((fieldIndex == 0 || fieldIndex == 1) && !tr.enabled ) tr.enabled = true;
      else if (fieldIndex == 2) tr.enabled = !tr.enabled;
      tr.triggered = false;

      // Flag pending deferred save & reset WDT save counter
      eepromPendingSave = true;
      wdtSaveTicks = 0;
      break;
    }
    case SET_CLOCK:
      if (fieldIndex == 0)      tempHours   = (tempHours   + 1) % 24;
      else if (fieldIndex == 1) tempMinutes = (tempMinutes + 1) % 60;
      else if (fieldIndex == 2) tempSeconds = (tempSeconds + 1) % 60;
      break;
    case RUNNING_COUNTDOWN: break;
    case CANCEL_PROMPT:
      cancelChoice = !cancelChoice;
      break;
  }
  needsRefresh = true;
}

void handleDown() {
  switch (menuState) {
    case HOME: break;
    case DIAGNOSTICS:
      diagIndex = (diagIndex + 1) % 6;
      break;     
    case MODE_SELECT:
      fieldIndex = (fieldIndex + 3) % 4;
      break;
    case OPTIONS:
      if (!optionsEditMode) {
        optionsFieldIndex = (optionsFieldIndex + 6) % 7;
      } else {
        if (optionsFieldIndex == 0) {
          sleepTimeoutMs -= 1000;
          if (sleepTimeoutMs < 5000) sleepTimeoutMs = 30000;
        } else if (optionsFieldIndex == 1) {
          pulseDurationMs -= 50;
          if (pulseDurationMs < 50) pulseDurationMs = 1000;
        } else if (optionsFieldIndex == 2) {
          voltageThreshold -= 5;
          if (voltageThreshold < 10) voltageThreshold = 200;
        } else if (optionsFieldIndex == 3) {
          OverloadCloseOpenDelta -= 10;
          if (OverloadCloseOpenDelta < 0) OverloadCloseOpenDelta = 50;
        }
      }
      break;
    case CONFIRM_RESET_COUNTDOWN:
    case CONFIRM_RESET_GATETIMES:
      cancelChoice = !cancelChoice;
      break;
    case SET_COUNTDOWN:
      if (fieldIndex == 4) {
        DateTime now = rtc.now();
        endTime = now + TimeSpan(setDays, setHours, setMinutes, setSeconds);
        lockActive = true;
        menuState = RUNNING_COUNTDOWN;
      } else {
        if (fieldIndex == 0)      setDays    = (setDays    + 999) % 1000;
        else if (fieldIndex == 1) setHours   = (setHours   + 23)  % 24;
        else if (fieldIndex == 2) setMinutes = (setMinutes + 59)  % 60;
        else if (fieldIndex == 3) setSeconds = (setSeconds + 59)  % 60;
      }
      break;
    case SET_GATETIMES: {
      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];
      if (fieldIndex == 0)      tr.hour = (tr.hour + 23) % 24;
      else if (fieldIndex == 1) tr.minute = (tr.minute + 59) % 60;
      if ((fieldIndex == 0 || fieldIndex == 1) && !tr.enabled ) tr.enabled = true;
      else if (fieldIndex == 2) tr.enabled = !tr.enabled;
      tr.triggered = false;

      // Flag pending deferred save & reset WDT save counter
      eepromPendingSave = true;
      wdtSaveTicks = 0;
      break;
    }
    case SET_CLOCK:
      if (fieldIndex == 0)      tempHours   = (tempHours   + 23) % 24;
      else if (fieldIndex == 1) tempMinutes = (tempMinutes + 59) % 60;
      else if (fieldIndex == 2) tempSeconds = (tempSeconds + 59) % 60;
      break;
    case RUNNING_COUNTDOWN: break;
    case CANCEL_PROMPT:
      cancelChoice = !cancelChoice;
      break;
  }
  needsRefresh = true;
}

void handleRight() {
  switch (menuState) {
    case HOME:
      menuState = MODE_SELECT;
      fieldIndex = 0;
      break;
    case DIAGNOSTICS:
      menuState = HOME;
      needsRefresh = true;
      break;    
    case MODE_SELECT:
      if (fieldIndex == 0) {
        menuState = SET_GATETIMES;
        currentTriggerIndex = 0;
        fieldIndex = 0;
      } else if (fieldIndex == 1) {
        if (lockActive) menuState = RUNNING_COUNTDOWN;
        else {
          menuState = SET_COUNTDOWN;
          fieldIndex = 0;
        }
      } else if (fieldIndex == 2) {
        menuState = OPTIONS;
        optionsFieldIndex = 0;
        optionsEditMode = false;
      } else if (fieldIndex == 3) {
        menuState = SET_CLOCK;
        DateTime now = rtc.now();
        tempHours   = now.hour();
        tempMinutes = now.minute();
        tempSeconds = now.second();
        fieldIndex = 0;
      }
      break;
    case OPTIONS:
      if (!optionsEditMode) {
        if (optionsFieldIndex <= 3) {
          optionsEditMode = true;
        } else if (optionsFieldIndex == 4) {
          menuState = CONFIRM_RESET_COUNTDOWN;
          cancelChoice = false;
        } else if (optionsFieldIndex == 5) {
          menuState = CONFIRM_RESET_GATETIMES;
          cancelChoice = false;
        } else if (optionsFieldIndex == 6) {
          menuState = HOME;
        }
      } else {
        optionsEditMode = false;
      }
      break;
    case CONFIRM_RESET_COUNTDOWN:
      lcd.clear();
      if (cancelChoice) {
        lockActive = false;
        setDays = setHours = setMinutes = setSeconds = 0;
        lcd.setCursor(0,0); lcd.print(F("Countdown has"));
        lcd.setCursor(0,1); lcd.print(F("been cleared!"));
      } else {
        lcd.setCursor(0,0); lcd.print(F("No Changes Made"));
        lcd.setCursor(0,1); lcd.print(F("Returning..."));
      }
      lastMessageStart = millis();
      showingMessage = true;
      menuState = OPTIONS;
      break;
    case CONFIRM_RESET_GATETIMES:
      lcd.clear();
      if (cancelChoice) {
        for (int t = 0; t < 5; t++) dailyTriggers[t] = {0,0,false,false};
        saveGateTimesToEEPROM(); // Clear EEPROM
        eepromPendingSave = false;
        wdtSaveTicks = 0;

        lcd.setCursor(0,0); lcd.print(F("Gate Times have"));
        lcd.setCursor(0,1); lcd.print(F("been cleared!"));
      } else {
        lcd.setCursor(0,0); lcd.print(F("No Changes Made"));
        lcd.setCursor(0,1); lcd.print(F("Returning..."));
      }
      lastMessageStart = millis();
      showingMessage = true;
      menuState = OPTIONS;
      break;
    case SET_COUNTDOWN:
      if (fieldIndex < 4) fieldIndex++;
      break;
    case SET_GATETIMES: {
      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];
      if (fieldIndex < 2) {
        fieldIndex++;
      } else {
        currentTriggerIndex = (currentTriggerIndex + 1) % 5;
        fieldIndex = 0;
      }
      (void)tr;
      break;
    }
    case RUNNING_COUNTDOWN:
      menuState = CANCEL_PROMPT;
      cancelChoice = false;
      break;
    case CANCEL_PROMPT:
      menuState = cancelChoice ? HOME : RUNNING_COUNTDOWN;
      if (cancelChoice) lockActive = false;
      break;
    case SET_CLOCK:
      if (fieldIndex < 3) {
        fieldIndex++;
      } else {
        DateTime now = rtc.now();
        rtc.adjust(DateTime(now.year(), now.month(), now.day(),
                            tempHours, tempMinutes, tempSeconds));
        menuState = HOME;
      }
      break;
  }
  needsRefresh = true;
}

void handleLeft() {
  switch (menuState) {
    case HOME:
      menuState = DIAGNOSTICS;
      diagIndex = 0;
      needsRefresh = true;
      break;
    case DIAGNOSTICS:
      menuState = HOME;
      needsRefresh = true;
      break;
    case MODE_SELECT:
      menuState = HOME;
      break;
    case OPTIONS:
      if (optionsEditMode) optionsEditMode = false;
      else menuState = HOME;
      break;
    case CONFIRM_RESET_COUNTDOWN:
    case CONFIRM_RESET_GATETIMES:
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(F("No Changes Made"));
      lcd.setCursor(0,1); lcd.print(F("Returning..."));
      lastMessageStart = millis();
      showingMessage = true;
      menuState = OPTIONS;
      break;
    case SET_COUNTDOWN:
      if (fieldIndex > 0) fieldIndex--;
      else { menuState = MODE_SELECT; fieldIndex = 1; }
      break;
    case SET_GATETIMES:
      if (fieldIndex > 0) fieldIndex--;
      else { menuState = MODE_SELECT; fieldIndex = 0; }
      break;
    case RUNNING_COUNTDOWN:
      menuState = MODE_SELECT;
      fieldIndex = 0;
      break;
    case CANCEL_PROMPT:
      menuState = RUNNING_COUNTDOWN;
      break;
    case SET_CLOCK:
      if (fieldIndex > 0) fieldIndex--;
      else menuState = HOME;
      break;
  }
  needsRefresh = true;
}


// --- Part 6: Helpers ---

long readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint16_t result = ADC;
  long vcc = 1125300L / result;
  return vcc;
}

bool runSweepCheck(long baseline, int currentAngle, int targetAngle) {
  releaseServo.write(targetAngle);
  unsigned long startCheck = millis();
  while (millis() - startCheck < 35) {
    long currentDrop = baseline - readVcc();
    if (currentDrop >= servoDetectThreshold) return true; 
  }
  releaseServo.write(currentAngle);
  delay(30); 
  return false;
}

LockType determineConnectedLock() {
  PRR &= ~_BV(PRADC);
  ADCSRA |= _BV(ADEN);
  delay(15);

  long baseline = 0;
  for (int i = 0; i < 5; i++) {
    baseline += readVcc();
    delay(2);
  }
  baseline /= 5;

  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, HIGH);
  delay(10); 
  long testVcc1 = readVcc();
  digitalWrite(latchPower, LOW);

  if ((baseline - testVcc1) >= lockDetectThreshold) {
    nitinolReady = true;
    return LOCK_MONO_PULSE;
  }

  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, HIGH); 
  delay(15); 

  releaseServo.attach(servoPin);
  int startAngle = servoIsOpen ? servoOpen : servoClosed;
  int targetAngle = servoIsOpen ? (servoOpen - 10) : (servoClosed + 10);

  bool servoDetected = runSweepCheck(baseline, startAngle, targetAngle);

  if (!servoDetected) {
    releaseServo.detach();
    digitalWrite(servoPin, LOW);
    resetServoRail(); 
    
    pinMode(latchPower, OUTPUT);
    digitalWrite(latchPower, HIGH); 
    delay(15); 
    
    releaseServo.attach(servoPin);
    servoDetected = runSweepCheck(baseline, startAngle, targetAngle);
  }

  releaseServo.detach();
  digitalWrite(servoPin, LOW);

  if (servoDetected) return LOCK_SERVO;

  nitinolReady = false;
  digitalWrite(latchPower, LOW);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Open / No Latch"));
  lcd.setCursor(0, 1); lcd.print(F("Reconnect Latch!"));
  delay(3000);

  return LOCK_MONO_PULSE; 
}

void resetServoRail() {
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, LOW);
  delay(300);   

  digitalWrite(latchPower, HIGH);
  delay(200);   

  releaseServo.attach(servoPin);
  delay(150);

  releaseServo.write(servoIsOpen ? servoOpen : servoClosed);
  delay(200);
  releaseServo.write(servoIsOpen ? (servoOpen - 10) : (servoClosed + 10));
  delay(200);
  releaseServo.write(servoIsOpen ? servoOpen : servoClosed);
  delay(200);

  releaseServo.detach();
}

bool servoOpenWithRetry(int currentMaxRetries = 5) {
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, HIGH);
  delay(120);

  PRR &= ~_BV(PRADC);
  ADCSRA |= _BV(ADEN);
  delay(50);

  auto attemptOpen = [&](int attemptNum) -> bool {
    releaseServo.attach(servoPin);
    delay(200);

    long baseline = 0;
    for (int i = 0; i < 5; i++) {
      baseline += readVcc();
      delay(5);
    }
    baseline /= 5;

    long dropSum = 0;
    long maxDrop = 0;
    int samples = 0;

    int delayMs = 10;
    const int minDelay = 5;
    const int maxDelay = 50;
    const int targetDrop = 400;

    if (attemptNum > 2 && !servoIsOpen) {
      for (int i = 0; i < 5; i++) {
        releaseServo.write(servoClosed + 2);
        delay(40);
        releaseServo.write(servoClosed);
        delay(40);
      }
      releaseServo.write(servoClosed + 15);
      delay(150);
      releaseServo.write(servoClosed - 5);
      delay(120);
      releaseServo.write(servoClosed);
      delay(120);
    }

    for (int pos = servoClosed; pos <= servoOpen; pos++) {
      releaseServo.write(pos);
      delay(delayMs);

      long vccNow = readVcc();
      long drop = baseline - vccNow;

      if (drop > targetDrop) {
        delayMs += 5;
        if (delayMs > maxDelay) delayMs = maxDelay;
      } else {
        delayMs -= 2;
        if (delayMs < minDelay) delayMs = minDelay;
      }

      if (drop > 0) {
        dropSum += drop;
        samples++;
        if (drop > maxDrop) maxDrop = drop;
      }
    }

    unsigned long start = millis();
    unsigned long timeout = 3000UL * tempMulti;

    long avgDrop = (samples > 0) ? dropSum / samples : 0;
    long thresh_opened = max(30L, avgDrop / 4);

    const int W = 5;
    long window[W] = {0};
    int idx = 0;
    int filled = 0;
    long sum = 0;
    bool recovered = false;

    while (millis() - start < timeout) {
      delay(50);
      long v = readVcc();
      long drop = baseline - v;

      sum -= window[idx];
      window[idx] = drop;
      sum += drop;
      idx = (idx + 1) % W;

      if (filled < W) filled++;

      long movAvg = sum / filled;
      if (movAvg <= thresh_opened) {
        recovered = true;
        break;
      }
    }

    delay(150);
    long endVcc = readVcc();
    releaseServo.detach();

    bool moved = avgDrop > 30;
    bool success = moved && recovered;
    if (!moved) resetServoRail();

    if (success) {
      lastSuccessVcc = baseline;
      lastSuccessAvg = avgDrop;
      lastSuccessEnd = baseline - endVcc;
      lastSuccessAttempt = attemptNum;
      lastSuccessTemp = currentTempC;
      lastOutcome = OUT_LOCK_OPENED;
    } else {
      lastFailVcc = baseline;
      lastFailAvg = avgDrop;
      lastFailEnd = baseline - endVcc;
      lastFailAttempt = attemptNum;
      lastFailTemp = currentTempC;
      lastOutcome = OUT_NONE;
    }

    if (displayActive && lcdReady) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Attempt "));
      lcd.print(attemptNum);
      lcd.print(success ? F(" OK") : F(" FAIL"));

      lcd.setCursor(0,1);
      lcd.print(F("A:")); lcd.print(avgDrop);
      lcd.print(F(" M:")); lcd.print(maxDrop);
      lcd.print(F(" R:")); lcd.print(baseline - endVcc);
      delay(1000);
    }

    return success;
  };

  for (int attempt = 1; attempt <= currentMaxRetries; attempt++) {
    if (attemptOpen(attempt)) {
      servoIsOpen = true;
      if (displayActive && lcdReady) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(F("Latch Opened"));
        lcd.setCursor(0,1); lcd.print(F("After "));
        lcd.print(attempt); lcd.print(F(" tries"));
        delay(600);
        refreshLCD();
      }
      return true;
    }
    safeCloseServo();
    delay(300);
  }

  if (displayActive && lcdReady) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(F("Open FAILED"));
    lcd.setCursor(0,1); lcd.print(F("Check mechanism"));
    delay(2000);
    refreshLCD();
  }

  lastOutcome = OUT_FAIL_MAX;
  return false;
}

bool safeCloseServo() {
  pinMode(latchPower, OUTPUT);
  digitalWrite(latchPower, HIGH);
  delay(120);

  PRR &= ~_BV(PRADC);
  ADCSRA |= _BV(ADEN);
  delay(50);

  const int maxRetries   = 3;
  const int maxRetries_c = 10;

  lastClose_overload = false;
  releaseServo.attach(servoPin);
  delay(150);

  for (int attempt = 0; attempt < maxRetries; attempt++) {
    long baselineVcc = readVcc();
    long adaptiveThreshold = (lastSuccessAvg == 0 ? 70 : lastSuccessAvg) + OverloadCloseOpenDelta;

    lastClose_avgDrop     = 0;
    lastClose_maxDrop     = 0;
    lastClose_adaptiveThr = adaptiveThreshold;

    long totalDrop = 0;
    int  samples   = 0;

    int pos_c = servoOpen;
    int attempt_c = 0;

    while (attempt_c < maxRetries_c) {
      attempt_c++;

      for (; pos_c >= servoClosed; pos_c--) {
        releaseServo.write(pos_c);
        delay(25 * tempMulti);

        long vccNow = readVcc();
        long drop   = baselineVcc - vccNow;

        if (drop > lastClose_maxDrop) lastClose_maxDrop = drop;

        if (drop >= 0) {
          totalDrop += drop;
          samples++;
          lastClose_avgDrop = totalDrop / samples;
        }

        if (drop > adaptiveThreshold && pos_c < servoOpen - 3) {
          lastClose_overload = true;

          int backpos = pos_c + 16;
          if (backpos > servoOpen) backpos = servoOpen;

          for (int back = pos_c; back <= backpos; back++) {
            releaseServo.write(back);
            delay(10);
          }

          pos_c = backpos;
          delay(200);
          goto retry_inner;
        }

        if (pos_c == servoClosed) {
          servoIsOpen = false;
          delay(800 * tempMulti);
          releaseServo.detach();
          return true;
        }
      }
      retry_inner:;
    }

    for (int back = pos_c; back <= servoOpen; back++) {
      releaseServo.write(back);
      delay(10);
    }
    delay(600);
  }

  releaseServo.detach();
  return false;
}

void update_Vthresh(long avgDrop) {
  long newThresh = avgDrop / 4;
  if (abs(newThresh - voltageThreshold) > 10) {
    if (newThresh < 20) newThresh = 20;
    if (newThresh > 200) newThresh = 200;
    voltageThreshold = newThresh;
  }
}

void triggerLock() {
  if (!displayActive || !lcdReady) {
    stagedRestoreAfterButtonWake();
  }

  PRR &= ~(_BV(PRADC) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2));
  ACSR &= ~_BV(ACD);
  ADCSRA |= _BV(ADEN);
  delay(50);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Gate Triggered!"));
  lastMessageStart = millis();
  showingMessage = true;

  activeLockType = determineConnectedLock();
  if (activeLockType == LOCK_SERVO) {
    pinMode(latchPower, OUTPUT);
    digitalWrite(latchPower, HIGH);
    delay(120);

    releaseServo.attach(servoPin);
    delay(150);

    bool openedOK = servoOpenWithRetry();
    delay(5000);
    bool closedOK = safeCloseServo();

    if (!closedOK) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(F("Latch Overload!"));
      lcd.setCursor(0,1); lcd.print(F("Check mechanism"));
      delay(4000);
    }

    releaseServo.detach();
  } else {
    pinMode(latchPower, OUTPUT);
    digitalWrite(latchPower, LOW);
    delay(20);

    pulseBaseline = (readVcc() + readVcc()) / 2;
    lockOpened = false;

    dropSum = 0;
    dropSamples = 0;
    maxDrop = 0;
    openTimeMs = 0;

    pulseActive = true;
    pulseStart = millis();
    digitalWrite(latchPower, HIGH);
  }
}

void updatePulse() {
  unsigned long now = millis();

  if (!pulseActive && retryScheduled && rtc.now() >= retryAtRTC) {
    retryScheduled = false;
    triggerLock();
    return;
  }

  if (!pulseActive) return;

  long vccNow = readVcc();
  long drop = pulseBaseline - vccNow;

  if (drop > 0) {
    dropSum += drop;
    dropSamples++;
    if (drop > maxDrop) maxDrop = drop;
  }

  long avgDrop = (dropSamples > 0) ? dropSum / dropSamples : 0;

  if (drop < voltageThreshold && !lockOpened) {
    lockOpened = true;
    digitalWrite(latchPower, LOW);
    pulseActive = false;
    lockActive = false;

    openTimeMs = now - pulseStart;

    if (avgDrop < voltageThreshold && maxDrop < voltageThreshold) {
      lastOutcome = (retryCount == 0) ? OUT_OPEN_BREAK : OUT_OPEN_BREAK_RETRY;
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(F("Open/Break/Nolock/Reconnect"));
      lcd.setCursor(0,1);
      lcd.print(F("A")); lcd.print(dropSamples > 0 ? dropSum / dropSamples : 0);
      lcd.print(F(" M")); lcd.print(maxDrop);
      lcd.print(F(" d")); lcd.print(drop);
      lastMessageStart = millis();
      showingMessage = true;
      return; 
    }
    
    if (pulseDurationMs - openTimeMs > 50) {
      pulseDurationMs = openTimeMs + 50;   
      if (pulseDurationMs < 50) pulseDurationMs = 50;
    }

    lastSuccessVcc = pulseBaseline;
    lastSuccessAvg = avgDrop;
    lastSuccessEnd = drop - voltageThreshold;
    lastSuccessOpenMs = openTimeMs;
    lastSuccessTemp = currentTempC;

    if (openTimeMs < bestOpenTime) {
      bestOpenTime = openTimeMs;
      bestOpenAvg = avgDrop;
      bestOpenEnd = drop - voltageThreshold;
      bestOpenVcc = pulseBaseline;
    }

    if (openTimeMs > worstOpenTime) {
      worstOpenTime = openTimeMs;
      worstOpenAvg = avgDrop;
      worstOpenEnd = drop - voltageThreshold;
      worstOpenVcc = pulseBaseline;
    }

    update_Vthresh(avgDrop);

    lastOutcome = (retryCount == 0) ? OUT_LOCK_OPENED : OUT_OPENED_RETRY;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Opened ")); lcd.print(openTimeMs); lcd.print(F("ms "));
    lcd.setCursor(0,1);
    lcd.print(F("A")); lcd.print(dropSamples > 0 ? dropSum / dropSamples : 0);
    lcd.print(F(" M")); lcd.print(maxDrop);
    if (retryCount > 0){
      lcd.print(F(" Rt")); lcd.print(retryCount);
    } else {
      lcd.print(F(" d")); lcd.print(drop - voltageThreshold);
    }
    delay(2500);    
    lastMessageStart = millis();
    showingMessage = true;
    return;
  }

  if (now - pulseStart >= pulseDurationMs) {
    digitalWrite(latchPower, LOW);
    pulseActive = false;

    if (!lockOpened) {
      lastFailVcc = pulseBaseline;
      lastFailAvg = avgDrop;
      lastFailEnd = drop - voltageThreshold;
      lastFailPulseMs = pulseDurationMs;
      lastFailTemp = currentTempC;
      update_Vthresh(avgDrop);

      pulseDurationMs += 100;
      if (pulseDurationMs > 1000) pulseDurationMs = 1000;

      if (retryCount >= maxRetries) {
        lastOutcome = OUT_FAIL_MAX;
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(F("Max Retries"));
        lcd.setCursor(0,1);
        lcd.print(F("Fail A:")); lcd.print(dropSamples > 0 ? dropSum / dropSamples : 0);
        lcd.print(F(" M:")); lcd.print(maxDrop);
        delay(2500);

        lockActive = false;
        showingMessage = true;
        lastMessageStart = millis();
        return;
      }
      
      retryCount++;
      retryAtRTC = rtc.now() + TimeSpan(0, 0, 0, 15);  
      retryScheduled = true;

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Fail A:")); lcd.print(dropSamples > 0 ? dropSum / dropSamples : 0);
      lcd.print(F(" M:")); lcd.print(maxDrop);

      lcd.setCursor(0,1);
      lcd.print(F("Retry ")); lcd.print(retryCount); lcd.print(F(" in 15s"));
      
      delay(2500);
      lastMessageStart = millis();
      showingMessage = true;
    }
  }
}

bool getNextGateTime(int &outHour, int &outMinute, String &outFormattedList) {
  DateTime now = rtc.now();
  int nowMinutes = now.hour() * 60 + now.minute();

  int bestDiff = 24 * 60 + 1;  
  bool found = false;

  // Build the formatted string starting with "T "
  outFormattedList = "T";

  for (int t = 0; t < 5; t++) {
    DailyTrigger &tr = dailyTriggers[t];

    // Append formatted hour ("08") or "--" if disabled
    outFormattedList += " ";
    if (tr.enabled) {
      if (tr.hour < 10) outFormattedList += "0"; // Zero-pad single digits
      outFormattedList += tr.hour;
    } else {
      outFormattedList += "--";
    }

    // Check for next trigger logic
    if (!tr.enabled) continue;

    int triggerMinutes = tr.hour * 60 + tr.minute;
    int diff = triggerMinutes - nowMinutes;
    if (diff < 0) diff += 24 * 60;  

    if (diff < bestDiff) {
      bestDiff = diff;
      outHour = tr.hour;
      outMinute = tr.minute;
      found = true;
    }
  }

  return found;
}

void printTwoDigits(int number) {
  if (number < 10) lcd.print('0');
  lcd.print(number);
}

void printThreeDigits(int number) {
  if (number < 100) lcd.print('0');
  if (number < 10) lcd.print('0');
  lcd.print(number);
}


// --- Part 7: LCD Refresh ---

void refreshLCD() {
  if (!displayActive || !lcdReady) return;
  if (showingMessage || welcomeMessage) return;

  lcd.clear();
  lcd.noCursor();

  DateTime now = rtc.now();

  switch (menuState) {
    case HOME: {
      static int homeScreenMode = 0;  
      unsigned long interval = (currentTempC < 0.0f) ? 6000UL : 3000UL;

      if (millis() - lastAlt >= interval) {
        homeScreenMode = (homeScreenMode + 1) % 3;
        lastAlt = millis();
      }

      // --- ROW 0 PRINTING ---
      lcd.setCursor(0, 0);
      
      if (homeScreenMode == 0) {
        long vcc = readVcc();
        float tC = currentTempC;
        lcd.print(F("B ")); 
        lcd.print(vcc / 1000.0, 2);
        lcd.print(F("v T ")); 
        lcd.print(tC, 1); 
        lcd.print(F("c    ")); // Padded to clear row 0
      } 
      else if (homeScreenMode == 1) {
        lcd.print(F("Time "));
        printTwoDigits(now.hour());   lcd.print(':');
        printTwoDigits(now.minute()); lcd.print(':');
        printTwoDigits(now.second());
        lcd.print(F("   "));   // Padded to clear row 0
      } 
      else if (homeScreenMode == 2) {
        int nh, nm;
        String gateListStr;
        
        if (getNextGateTime(nh, nm, gateListStr)) {
          lcd.print(F("Next Gate "));
          printTwoDigits(nh); 
          lcd.print(':'); 
          printTwoDigits(nm);
        } else {
          lcd.print(F("All Times OFF   "));
        }
        
        // Mode 2 manages ROW 1 here directly
        lcd.setCursor(0, 1);
        lcd.print(gateListStr); 
      }

      // --- ROW 1 PRINTING (Only for Mode 0 and Mode 1) ---
      
      lcd.setCursor(0, 1);
        
      if (lockActive) {
        long total = endTime.unixtime() - now.unixtime();
        if (total > 0) {
          int dd = total / 86400;
          int hh = (total % 86400) / 3600;
          int mm = (total % 3600) / 60;
          int ss = total % 60;
          
          lcd.print(F("Run "));
          printThreeDigits(dd); lcd.print(':');
          printTwoDigits(hh);   lcd.print(':');
          printTwoDigits(mm);   lcd.print(':');
          printTwoDigits(ss);
        } else {
          lcd.print(F("Running Expired "));          }
      } else if (homeScreenMode != 2) {
      lcd.print(F("Press Right ->  "));
      }
      

      break;
    }

    case MODE_SELECT: {
      lcd.setCursor(0,0); lcd.print(F("Select Mode"));
      lcd.setCursor(0,1);
      if (fieldIndex == 0)      lcd.print(F("> Gate Times"));
      else if (fieldIndex == 1) lcd.print(F("> Countdown"));
      else if (fieldIndex == 2) lcd.print(F("> Options"));
      else if (fieldIndex == 3) lcd.print(F("> Set Clock"));
      break;
    }

    case OPTIONS: {
      lcd.setCursor(0,0);
      if (!optionsEditMode) {
        char bufTop[17];
        size_t msgLen = strlen_P(optionsHelpMsg);
        for (int i = 0; i < 16; i++) {
          bufTop[i] = pgm_read_byte(&optionsHelpMsg[(optionsHelpPos + i) % msgLen]);
        }
        bufTop[16] = '\0';
        lcd.print(bufTop);
      } else {
        lcd.print(F("Adjust Option: "));
      }

      lcd.setCursor(0,1);
      if (optionsFieldIndex == 0) {
        lcd.print(F("Timeout ")); lcd.print(sleepTimeoutMs / 1000); lcd.print(F("s"));
      } else if (optionsFieldIndex == 1) {
        lcd.print(F("NL Pulse ")); lcd.print(pulseDurationMs); lcd.print(F("ms"));
      } else if (optionsFieldIndex == 2) {
        lcd.print(F("NL MinDrop ")); lcd.print(voltageThreshold); lcd.print(F("mV"));
      } else if (optionsFieldIndex == 3) {
        lcd.print(F("CloseOvLoad ")); lcd.print(OverloadCloseOpenDelta); lcd.print(F("mV"));
      } else if (optionsFieldIndex == 4) {
        lcd.print(F("Reset Countdown"));
      } else if (optionsFieldIndex == 5) {
        lcd.print(F("Reset GateTimes"));
      } else if (optionsFieldIndex == 6) {
        lcd.print(F("Exit Options"));
      }

      if (optionsEditMode && optionsFieldIndex <= 3) {
        lcd.setCursor(14,1);
        lcd.print(F("<>"));
      }
      break;
    }

    case CONFIRM_RESET_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print(F("Clear Countdown?"));
      lcd.setCursor(0,1); lcd.print(cancelChoice ? F("Yes") : F("No "));
      break;
    }

    case CONFIRM_RESET_GATETIMES: {
      lcd.setCursor(0,0); lcd.print(F("Clear Gate Times?"));
      lcd.setCursor(0,1); lcd.print(cancelChoice ? F("Yes") : F("No "));
      break;
    }

    case SET_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print(F("Set D:HH:MM:SS"));
      lcd.setCursor(0,1);
      printThreeDigits(setDays); lcd.print(':');
      printTwoDigits(setHours); lcd.print(':');
      printTwoDigits(setMinutes); lcd.print(':');
      printTwoDigits(setSeconds);
      lcd.setCursor(14,1); lcd.print(F("Go"));

      if (fieldIndex == 0)      lcd.setCursor(0,1);   
      else if (fieldIndex == 1) lcd.setCursor(4,1);   
      else if (fieldIndex == 2) lcd.setCursor(7,1);   
      else if (fieldIndex == 3) lcd.setCursor(10,1);  
      else if (fieldIndex == 4) lcd.setCursor(14,1);  
      lcd.cursor();
      break;
    }

    case SET_GATETIMES: {
      lcd.setCursor(0,0); lcd.print(F("TimeSet-")); lcd.print(currentTriggerIndex + 1);
      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];

      lcd.setCursor(0,1);
      printTwoDigits(tr.hour); lcd.print(':');
      printTwoDigits(tr.minute);
      lcd.print(tr.enabled ? F("  -> On") : F("  -> Off"));

      if (fieldIndex == 0)      lcd.setCursor(0,1);   
      else if (fieldIndex == 1) lcd.setCursor(3,1);   
      else if (fieldIndex == 2) lcd.setCursor(10,1);  
      lcd.cursor();
      break;
    }

    case RUNNING_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print(F("Running"));
      lcd.setCursor(0,1);
      long total = endTime.unixtime() - now.unixtime();
      if (total > 0) {
        int dd = total / 86400;
        int hh = (total % 86400) / 3600;
        int mm = (total % 3600) / 60;
        int ss = total % 60;
        printThreeDigits(dd); lcd.print(':');
        printTwoDigits(hh); lcd.print(':');
        printTwoDigits(mm); lcd.print(':');
        printTwoDigits(ss);
      } else {
        lcd.print(F("Expired"));
      }
      break;
    }

    case CANCEL_PROMPT: {
      lcd.setCursor(0,0); lcd.print(F("Cancel Timer?"));
      lcd.setCursor(0,1); lcd.print(cancelChoice ? F("Yes") : F("No "));
      break;
    }

    case SET_CLOCK: {
      lcd.setCursor(0,0); lcd.print(F("Adjust Clock"));
      lcd.setCursor(0,1);
      printTwoDigits(tempHours); lcd.print(':');
      printTwoDigits(tempMinutes); lcd.print(':');
      printTwoDigits(tempSeconds);
      lcd.setCursor(10,1); lcd.print(F("Apply"));

      if (fieldIndex == 0)      lcd.setCursor(0,1);
      else if (fieldIndex == 1) lcd.setCursor(3,1);
      else if (fieldIndex == 2) lcd.setCursor(6,1);
      else if (fieldIndex == 3) lcd.setCursor(10,1);
      lcd.cursor();
      break;
    }

    case DIAGNOSTICS: {
      lcd.clear();
      lcd.noCursor();

      if (diagIndex == 0) {
        lcd.setCursor(0,0); lcd.print(F("Last Outcome:"));
        lcd.setCursor(0,1);
        switch (lastOutcome) {
          case OUT_NONE:             break;
          case OUT_OPEN_BREAK:       lcd.print(F("Open or Break")); break;
          case OUT_OPEN_BREAK_RETRY: lcd.print(F("RT Open or Break")); break;
          case OUT_LOCK_OPENED:      lcd.print(F("Latch Opened")); break;
          case OUT_OPENED_RETRY:     lcd.print(F("Opened w retry")); break;
          case OUT_FAIL_MAX:         lcd.print(F("Retry's Failed")); break;
        }
      } else if (diagIndex == 1) {
        lcd.setCursor(0,0); lcd.print(F("V ")); lcd.print(lastSuccessVcc); lcd.print(F(" A ")); lcd.print(lastSuccessAvg);
        lcd.setCursor(0,1); lcd.print(F("d ")); lcd.print(lastSuccessEnd);
        if (activeLockType == LOCK_MONO_PULSE) {
          lcd.print(F(" T ")); lcd.print(lastSuccessOpenMs); lcd.print(F("ms"));
        } else {
          lcd.print(F(" Attempt ")); lcd.print(lastSuccessAttempt);          
        }
      } else if (diagIndex == 2) {
        lcd.setCursor(0,0); lcd.print(F("VF ")); lcd.print(lastFailVcc); lcd.print(F(" AF ")); lcd.print(lastFailAvg);
        lcd.setCursor(0,1); lcd.print(F("dF ")); lcd.print(lastFailEnd);
        if (activeLockType == LOCK_MONO_PULSE) {
          lcd.print(F(" PF ")); lcd.print(lastFailPulseMs); lcd.print(F("ms"));
        } else {
          lcd.print(F(" Att ")); lcd.print(lastFailAttempt);
        }
      } else if (diagIndex == 3) {
        lcd.setCursor(0,0); lcd.print(F("Last Okay:")); lcd.print(lastSuccessTemp, 1); lcd.print(F("c"));
        lcd.setCursor(0,1); lcd.print(F("Last Fail:")); lcd.print(lastFailTemp, 1); lcd.print(F("c"));
      } else if (diagIndex == 4) {        
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(F("C avg ")); lcd.print(lastClose_avgDrop); lcd.print(F(" t ")); lcd.print(lastClose_adaptiveThr);
        lcd.setCursor(0,1); lcd.print(F("mx ")); lcd.print(lastClose_maxDrop); lcd.print(F(" OL:")); lcd.print(lastClose_overload ? F("Y") : F("N"));
      } else if (diagIndex == 5) {
        lcd.setCursor(0,0); lcd.print(F("To Exit Press"));
        lcd.setCursor(0,1); lcd.print(F("Right or Left"));
      }
      break;
    }
  }
}


// --- Part 8: Main Loop ---

void loop() {
  if (wokeFromButton) {
    stagedRestoreAfterButtonWake();
    sleeping = false;
    wokeFromButton = false;
  }
  

  // --- LED Blink for  Every 4th WDT ---
  if (blinkLED && sleeping && !statusLEDon) {
    blinkLED = false;
    statusLEDon=true;
    int enabledCount = getEnabledTriggerCount();
    // Flash once for every enabled trigger
    for (int i = 0; i < enabledCount; i++) {
      digitalWrite(ledPin, HIGH);
      delay(75); 
      digitalWrite(ledPin, LOW);
      delay(200); 
    }
  }

  // --- Watchdog Tick ---
  if (watchdogTick) {
    watchdogTick = false;
    
    if (getEnabledTriggerCount() > 0) {
      digitalWrite(ledPin, statusLEDon = !statusLEDon);
    }

    // --- Deferred EEPROM Save logic ---
    if (eepromPendingSave) {
      wdtSaveTicks++;
      if (wdtSaveTicks >= 4) {
        saveGateTimesToEEPROM();
        eepromPendingSave = false;
        wdtSaveTicks = 0;
      }
    }

    PRR &= ~_BV(PRTWI);
    Wire.begin();
    delay(10);

    DateTime now = rtc.now();

    currentTempC = rtc.getTemperature();
    tempMulti = (currentTempC < -12 ? 4 :
                 currentTempC < -6  ? 3 :
                 currentTempC < 0   ? 2 : 1);

    if (lockActive && now >= endTime) {
      retryCount = 0;
      lockActive = false;
      pulseManualActive = false;
      triggerLock();
    }

    for (int t = 0; t < 5; t++) {
      DailyTrigger &tr = dailyTriggers[t];

      if (tr.enabled && now.hour() == tr.hour && now.minute() == tr.minute && !tr.triggered) {
        retryCount = 0;
        triggerLock();
        tr.triggered = true;
      }

      if (tr.enabled && (now.hour() != tr.hour || now.minute() != tr.minute) && tr.triggered) {
        tr.triggered = false;
      }
    }

    if (displayActive && lcdReady && menuState == RUNNING_COUNTDOWN) {
      refreshLCD();
    }
  }

  // --- Buttons ---
  checkButton(btnDown, 0, handleDown);
  checkButton(btnUp,   1, handleUp);
  checkButton(btnLeft, 2, handleLeft);
  checkButton(btnRight,3, handleRight);

  updatePulse();

  // Combo: Up + Down → toggle servo state or open latch
  if (digitalRead(btnUp) == LOW && digitalRead(btnDown) == LOW && !pulseManualActive) {
    if (!comboHandled) {
      comboHandled = true;
      delay(200);

      if (!displayActive || !lcdReady) stagedRestoreAfterButtonWake();
      if (!pulseActive && !lockActive) activeLockType = determineConnectedLock();

      if (activeLockType == LOCK_MONO_PULSE && !pulseActive && !lockActive && nitinolReady) {          
        delay(300);
        DateTime now = rtc.now();
        endTime = now + TimeSpan(0, 0, 0, 2);   
        lockActive = true;
        pulseManualActive = true;
        menuState = HOME;
      } else if (activeLockType == LOCK_SERVO && !pulseActive) {
        if (servoIsOpen) {
          bool ok = safeCloseServo();
          if (!ok) {
            lcd.clear();
            lcd.setCursor(0,0); lcd.print(F("Latch Overload!"));
            lcd.setCursor(0,1); lcd.print(F("Check Mechanism"));
            delay(1500);
          } else {
            lcd.clear();
            lcd.setCursor(0,0); lcd.print(F("Latch CLOSED"));
            delay(600);
          }
        } else {
          servoOpenWithRetry();
        }
      }
      refreshLCD();
    }
  } else {
    comboHandled = false;
  }

  // OPTIONS help scroll
  if (millis() - lastButtonPress < (sleepTimeoutMs - 500)) {
    if (menuState == OPTIONS && displayActive && lcdReady && !optionsEditMode) {
      if (millis() - lastOptionsScroll >= optionsScrollIntervalMs) {
        optionsHelpPos++;
        size_t msgLen = strlen_P(optionsHelpMsg);
        if (optionsHelpPos >= msgLen) optionsHelpPos = 0;
        lastOptionsScroll = millis();
        needsRefresh = true;
      }
    }
  }

  if (menuState != lastState) {
    if (displayActive && lcdReady) refreshLCD();
    lastState = menuState;
  }

  if (showingMessage && millis() - lastMessageStart >= 2000) {
    showingMessage = false;

    if (menuState == OPTIONS || menuState == CONFIRM_RESET_COUNTDOWN || menuState == CONFIRM_RESET_GATETIMES) {
      menuState = OPTIONS;
    } else {
      menuState = HOME;
    }
    refreshLCD();
  }

  if (needsRefresh && displayActive && lcdReady) {
    refreshLCD();
    needsRefresh = false;
  }

  if (WakeMessageCheck && displayActive && lcdReady) {
    welcomeMessage = true;
    showWelcomeAfterLongSleepIfNeeded();
    WakeMessageCheck = false;
    welcomeMessage = false;
    lastwelcome = rtc.now();
  }

  static unsigned long lastRefresh = 0;
  if (menuState == HOME && displayActive && lcdReady) {
    if (millis() - lastRefresh >= 1000) {
      refreshLCD();
      lastRefresh = millis();
    }
  }

  // --- Sleep Logic ---
  if (millis() - lastButtonPress >= sleepTimeoutMs) {
    sleeping = true;
    preparePinsForSleep();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    cli();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
  } else {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
  }
}