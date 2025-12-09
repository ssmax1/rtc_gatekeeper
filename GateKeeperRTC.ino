// --- Part 1: Includes, Globals, Enums ---

#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

// LCD pins
const int rs = 4, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Single lock pin
const int lockPin = 5;

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
  CONFIRM_RESET_GATETIMES
};

MenuState menuState = HOME;
MenuState lastState = HOME;

RTC_DS3231 rtc;

// Watchdog flags
volatile bool watchdogTick = false;
volatile bool wokeFromButton = false;
volatile bool wokeFromWDT = false;

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

// Pulse control
bool pulseActive = false;
unsigned long pulseStart = 0;

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
bool globalResetPrompt = false;
static bool needsRefresh = false;
uint16_t clockDivide = 1;
unsigned long sleepTimeoutMs = 10000;   // adjustable between 5000–30000
unsigned long pulseDurationMs = 400;    // adjustable between 100–500

// OPTIONS
bool optionsEditMode = false;
int optionsFieldIndex = 0;

// Options help scroll
const char optionsHelpMsg[] = "Options - Right to edit/confirm; Up/Down to change options and values...   ";
static unsigned int optionsHelpPos = 0;
static unsigned long lastOptionsScroll = 0;
const unsigned long optionsScrollIntervalMs = 300;

// Sleep tracking using RTC
DateTime lastwelcome;


// --- Part 2: Watchdog, ISRs, Setup ---

ISR(WDT_vect) {
  wokeFromWDT = true;
  watchdogTick = true;
}

void wakeISR() {
  wokeFromButton = true;
  lastButtonPress = millis();
  WakeMessageCheck = true;
}

void setupWatchdog8s() {
  MCUSR = 0;
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // 8s tick
}

void updateClockDivide() {
  uint8_t prescaleBits = CLKPR & 0x0F;
  static const uint16_t prescaleTable[9] = {
    1, 2, 4, 8, 16, 32, 64, 128, 256
  };
  if (prescaleBits <= 8) clockDivide = prescaleTable[prescaleBits];
  else clockDivide = 1;
}

void setup() {
  // Lock pin low on startup
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, LOW);

  // Daily triggers init
  for (int t = 0; t < 5; t++) {
    dailyTriggers[t] = {0, 0, false, false};
  }

  // Power LCD on at startup
  pinMode(lcdVccPin, OUTPUT);
  digitalWrite(lcdVccPin, HIGH);
  delay(50);

  Wire.begin();
  if (!rtc.begin()) {
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("RTC not found!");
    while (1);
  }

  lcd.begin(16, 2);
  lcd.setCursor(0,0); lcd.print("  Welcome   To  ");
  lcd.setCursor(0,1); lcd.print("  Gate  Keeper  ");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(" Velkommen  Til ");
  lcd.setCursor(0,1); lcd.print("  Portvogteren  ");
  delay(1500);
  lcd.clear();

  // Clock prescaler
  CLKPR = 0x80;
  CLKPR = 0x00;
  updateClockDivide();

  // Buttons
  pinMode(btnLeft,  INPUT_PULLUP);
  pinMode(btnRight, INPUT_PULLUP);
  pinMode(btnUp,    INPUT_PULLUP);
  pinMode(btnDown,  INPUT_PULLUP);

  lastButtonPress = millis();

  attachInterrupt(digitalPinToInterrupt(btnUp),   wakeISR, LOW);
  attachInterrupt(digitalPinToInterrupt(btnDown), wakeISR, LOW);

  setupWatchdog8s();
  sei();

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

  // Cut LCD VCC
  pinMode(lcdVccPin, INPUT);
  digitalWrite(lcdVccPin, LOW);

  // Ensure lock off
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, LOW);

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

  if (slept.totalseconds() > 900 ) {  // > 15 mins
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("  Welcome   To  ");
    lcd.setCursor(0,1); lcd.print("  Gate Keeper  ");
    delay(1500);

    lcd.clear();
    lcd.setCursor(0,0); lcd.print(" Velkommen  Til ");
    lcd.setCursor(0,1); lcd.print("  Portvogteren  ");
    delay(1500);
  }
}

void stagedRestoreAfterButtonWake() {
  //wokeFromButton = false;
  cli();

  // Re-enable peripherals
  PRR   &= ~(_BV(PRADC) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2));
  ACSR  &= ~_BV(ACD);
  ADCSRA |= _BV(ADEN);

  // Power LCD back on
  pinMode(lcdVccPin, OUTPUT);
  digitalWrite(lcdVccPin, HIGH);

  sei();
  delay(50);

  // Restore LCD signal pins
  pinMode(rs, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(d4, OUTPUT);
  pinMode(d5, OUTPUT);
  pinMode(d6, OUTPUT);
  pinMode(d7, OUTPUT);

  // Restore lock pin
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, LOW);

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


// --- Part 4: Main Loop, Locks, Button Logic ---

void updateLocks() {
  if (pulseActive && (millis() - pulseStart >= pulseDurationMs)) {
    digitalWrite(lockPin, LOW);
    pulseActive = false;
  }
}

// Debounce + repeat state
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

        detachInterrupt(digitalPinToInterrupt(btnUp));
        detachInterrupt(digitalPinToInterrupt(btnDown));
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
      attachInterrupt(digitalPinToInterrupt(btnUp),   wakeISR, LOW);
      attachInterrupt(digitalPinToInterrupt(btnDown), wakeISR, LOW);
    }
  }
}


// --- Part 5: Button Handlers ---

void handleUp() {
  switch (menuState) {
    case HOME:
      // No action on Up at Home
      break;

    case MODE_SELECT:
      // move up in menu
      fieldIndex = (fieldIndex + 1) % 4; // wrap 0..3
      break;

    case OPTIONS:
      if (!optionsEditMode) {
        optionsFieldIndex = (optionsFieldIndex + 1) % 5; // cycle up
      } else {
        if (optionsFieldIndex == 0) {          // Timeout
          sleepTimeoutMs += 1000;
          if (sleepTimeoutMs > 30000) sleepTimeoutMs = 5000;
        } else if (optionsFieldIndex == 1) {   // Pulse duration
          pulseDurationMs += 100;
          if (pulseDurationMs > 500) pulseDurationMs = 100;
        }
      }
      break;

    case CONFIRM_RESET_COUNTDOWN:
    case CONFIRM_RESET_GATETIMES:
      cancelChoice = !cancelChoice;
      break;

    case SET_COUNTDOWN:
      if (fieldIndex == 4) {
        // On "Go": Up starts countdown
        if (setDays == 0 && setHours == 0 && setMinutes == 0 && setSeconds == 0) {
          // do nothing if zero-length?
        }
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
      else if (fieldIndex == 2) tr.enabled = !tr.enabled;
      tr.triggered = false;
      break;
    }

    case SET_CLOCK:
      if (fieldIndex == 0)      tempHours   = (tempHours   + 1) % 24;
      else if (fieldIndex == 1) tempMinutes = (tempMinutes + 1) % 60;
      else if (fieldIndex == 2) tempSeconds = (tempSeconds + 1) % 60;
      break;

    case RUNNING_COUNTDOWN:
      // no Up behavior here
      break;

    case CANCEL_PROMPT:
      cancelChoice = !cancelChoice;
      break;
  }
  needsRefresh = true;
}

void handleDown() {
  switch (menuState) {
    case HOME:
      // No action on Down at Home
      break;

    case MODE_SELECT:
      // move down in menu
      fieldIndex = (fieldIndex + 3) % 4;
      break;

    case OPTIONS:
      if (!optionsEditMode) {
        optionsFieldIndex = (optionsFieldIndex + 4) % 5;
      } else {
        if (optionsFieldIndex == 0) {          // Timeout
          sleepTimeoutMs -= 1000;
          if (sleepTimeoutMs < 5000) sleepTimeoutMs = 30000;
        } else if (optionsFieldIndex == 1) {   // Pulse duration
          pulseDurationMs -= 100;
          if (pulseDurationMs < 100) pulseDurationMs = 500;
        }
      }
      break;

    case CONFIRM_RESET_COUNTDOWN:
    case CONFIRM_RESET_GATETIMES:
      cancelChoice = !cancelChoice;
      break;

    case SET_COUNTDOWN:
      if (fieldIndex == 4) {
        // On "Go": Down also starts countdown
        if (setDays == 0 && setHours == 0 && setMinutes == 0 && setSeconds == 0) {
          // do nothing if zero-length?
        }
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
      if (fieldIndex == 0)      tr.hour   = (tr.hour   + 23) % 24;
      else if (fieldIndex == 1) tr.minute = (tr.minute + 59) % 60;
      else if (fieldIndex == 2) tr.enabled = !tr.enabled;
      tr.triggered = false;
      break;
    }

    case SET_CLOCK:
      if (fieldIndex == 0)      tempHours   = (tempHours   + 23) % 24;
      else if (fieldIndex == 1) tempMinutes = (tempMinutes + 59) % 60;
      else if (fieldIndex == 2) tempSeconds = (tempSeconds + 59) % 60;
      break;

    case RUNNING_COUNTDOWN:
      // no Down behavior
      break;

    case CANCEL_PROMPT:
      cancelChoice = !cancelChoice;
      break;
  }
  needsRefresh = true;
}

void handleRight() {
  switch (menuState) {
    case HOME:
      // Enter MODE_SELECT
      menuState = MODE_SELECT;
      fieldIndex = 0; // 0=Countdown,1=GateTimes,2=Options,3=SetClock
      break;

    case MODE_SELECT:
      // Right selects current item
      if (fieldIndex == 0) {          // Countdown
        // If countdown active, go to RUNNING; else to SET_COUNTDOWN
        if (lockActive) menuState = RUNNING_COUNTDOWN;
        else {
          menuState = SET_COUNTDOWN;
          fieldIndex = 0;
        }
      } else if (fieldIndex == 1) {   // Gate Times
        menuState = SET_GATETIMES;
        currentTriggerIndex = 0;
        fieldIndex = 0;
      } else if (fieldIndex == 2) {   // Options
        menuState = OPTIONS;
        optionsFieldIndex = 0;
        optionsEditMode = false;
      } else if (fieldIndex == 3) {   // Set Clock
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
        // Right = select / confirm / enter
        if (optionsFieldIndex == 0 || optionsFieldIndex == 1) {
          // Enter edit mode (Timeout or Pulse)
          optionsEditMode = true;
        } else if (optionsFieldIndex == 2) {
          // Reset Countdown
          menuState = CONFIRM_RESET_COUNTDOWN;
          cancelChoice = false;
        } else if (optionsFieldIndex == 3) {
          // Reset GateTimes
          menuState = CONFIRM_RESET_GATETIMES;
          cancelChoice = false;
        } else if (optionsFieldIndex == 4) {
          // Exit Options -> Home
          menuState = HOME;
        }
      } else {
        // In edit mode: Right exits edit mode (same as Left)
        optionsEditMode = false;
      }
      break;

    case CONFIRM_RESET_COUNTDOWN:
      // Right confirms
      if (cancelChoice) {
        // Yes
        lockActive = false;
        setDays = setHours = setMinutes = setSeconds = 0;
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Countdown has");
        lcd.setCursor(0,1); lcd.print("been cleared!");
        lastMessageStart = millis();
        showingMessage = true;
      } else {
        // No
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("No Changes Made");
        lcd.setCursor(0,1); lcd.print("Returning...");
        lastMessageStart = millis();
        showingMessage = true;
      }
      menuState = OPTIONS;
      break;

    case CONFIRM_RESET_GATETIMES:
      // Right confirms
      if (cancelChoice) {
        // Yes
        for (int t = 0; t < 5; t++) {
          dailyTriggers[t] = {0,0,false,false};
        }
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Gate Times have");
        lcd.setCursor(0,1); lcd.print("been cleared!");
        lastMessageStart = millis();
        showingMessage = true;
      } else {
        // No
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("No Changes Made");
        lcd.setCursor(0,1); lcd.print("Returning...");
        lastMessageStart = millis();
        showingMessage = true;
      }
      menuState = OPTIONS;
      break;

    case SET_COUNTDOWN:
      // Right moves between fields only; does NOT start countdown on Go
      if (fieldIndex < 4) {
        fieldIndex++;
      } else {
        // fieldIndex == 4 on "Go": do nothing
      }
      break;

    case SET_GATETIMES: {
      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];
      if (fieldIndex < 2) {
        fieldIndex++;
      } else {
        // Move to next trigger
        currentTriggerIndex = (currentTriggerIndex + 1) % 5;
        fieldIndex = 0;
      }
      (void)tr;
      break;
    }

    case RUNNING_COUNTDOWN:
      // Right -> CANCEL_PROMPT
      menuState = CANCEL_PROMPT;
      cancelChoice = false;
      break;

    case CANCEL_PROMPT:
      // Right confirms Yes/No
      if (cancelChoice) {
        // Yes -> cancel timer
        lockActive = false;
        menuState = HOME;
      } else {
        // No -> back to running
        menuState = RUNNING_COUNTDOWN;
      }
      break;

    case SET_CLOCK:
      if (fieldIndex < 3) {
        fieldIndex++;
      } else {
        // Apply
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
      // No Left from main
      break;

    case MODE_SELECT:
      // Back to Home
      menuState = HOME;
      break;

    case OPTIONS:
      if (optionsEditMode) {
        // Exit edit mode
        optionsEditMode = false;
      } else {
        // Left = back to previous (Home)
        menuState = HOME;
      }
      break;

    case CONFIRM_RESET_COUNTDOWN:
      // Left = cancel and return
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("No Changes Made");
      lcd.setCursor(0,1); lcd.print("Returning...");
      lastMessageStart = millis();
      showingMessage = true;
      menuState = OPTIONS;
      break;

    case CONFIRM_RESET_GATETIMES:
      // Left = cancel and return
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("No Changes Made");
      lcd.setCursor(0,1); lcd.print("Returning...");
      lastMessageStart = millis();
      showingMessage = true;
      menuState = OPTIONS;
      break;

    case SET_COUNTDOWN:
      if (fieldIndex > 0) {
        fieldIndex--;
      } else {
        // fieldIndex == 0: back to MODE_SELECT
        menuState = MODE_SELECT;
        fieldIndex = 0;
      }
      break;

    case SET_GATETIMES:
      if (fieldIndex > 0) {
        fieldIndex--;
      } else {
        // Back to MODE_SELECT
        menuState = MODE_SELECT;
        fieldIndex = 1; // highlight Gate Times
      }
      break;

    case RUNNING_COUNTDOWN:
      // Left -> back to MODE_SELECT
      menuState = MODE_SELECT;
      fieldIndex = 0;
      break;

    case CANCEL_PROMPT:
      // Left cancels and returns to RUNNING
      menuState = RUNNING_COUNTDOWN;
      break;

    case SET_CLOCK:
      if (fieldIndex > 0) {
        fieldIndex--;
      } else {
        // fieldIndex == 0: back to Home without saving
        menuState = HOME;
      }
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

void triggerLock() {
  if (!displayActive && !lcdReady) {
    // we're waking from sleep due to trigger; do staged restore
    stagedRestoreAfterButtonWake();
  }

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Lock Triggered!");
  lastMessageStart = millis();
  showingMessage = true;

  digitalWrite(lockPin, HIGH);
  pulseActive = true;
  pulseStart = millis();
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
      // Alternate between Battery and Time every 3s
      if (millis() - lastAlt >= 3000) {
        showVoltage = !showVoltage;
        lastAlt = millis();
      }

      lcd.setCursor(0,0);
      if (showVoltage) {
        lcd.print("Battery ");
        long vcc = readVcc();
        lcd.print(vcc / 1000.0, 2);
        lcd.print(" V");
      } else {
        lcd.print("Time ");
        char buf[9];
        sprintf(buf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        lcd.print(buf);
      }

      lcd.setCursor(0,1);
      if (lockActive) {
        long total = endTime.unixtime() - now.unixtime();
        if (total > 0) {
          int dd = total / 86400;
          int hh = (total % 86400) / 3600;
          int mm = (total % 3600) / 60;
          int ss = total % 60;
          char bufRun[16];
          sprintf(bufRun, "Run %03d:%02d:%02d:%02d", dd, hh, mm, ss);
          lcd.print(bufRun);
        } else {
          lcd.print("Running Expired");
        }
      } else {
        lcd.print("Press Right ->");
      }
      break;
    }

    case MODE_SELECT: {
      lcd.setCursor(0,0);
      lcd.print("Select Mode");
      lcd.setCursor(0,1);
      // Show current item text only
      if (fieldIndex == 0)      lcd.print("> Countdown");
      else if (fieldIndex == 1) lcd.print("> Gate Times");
      else if (fieldIndex == 2) lcd.print("> Options");
      else if (fieldIndex == 3) lcd.print("> Set Clock");
      break;
    }

    case OPTIONS: {
      lcd.setCursor(0,0);
      if (!optionsEditMode) {
        // scrolling help
        char bufTop[17];
        size_t msgLen = strlen(optionsHelpMsg);
        for (int i = 0; i < 16; i++) {
          bufTop[i] = optionsHelpMsg[(optionsHelpPos + i) % msgLen];
        }
        bufTop[16] = '\0';
        lcd.print(bufTop);
      } else {
        lcd.print("Adjust Option: ");
      }

      lcd.setCursor(0,1);
      if (optionsFieldIndex == 0) {
        lcd.print("Timeout ");
        lcd.print(sleepTimeoutMs / 1000);
        lcd.print("s");
      } else if (optionsFieldIndex == 1) {
        lcd.print("Pulse ");
        lcd.print(pulseDurationMs);
        lcd.print("ms");
      } else if (optionsFieldIndex == 2) {
        lcd.print("Reset Countdown");
      } else if (optionsFieldIndex == 3) {
        lcd.print("Reset GateTimes");
      } else if (optionsFieldIndex == 4) {
        lcd.print("Exit Options");
      }

      if (optionsEditMode && (optionsFieldIndex == 0 || optionsFieldIndex == 1)) {
        lcd.setCursor(14,1);
        lcd.print("<>");
      }
      break;
    }

    case CONFIRM_RESET_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print("Clear Countdown?");
      lcd.setCursor(0,1);
      lcd.print(cancelChoice ? "Yes" : "No ");
      break;
    }

    case CONFIRM_RESET_GATETIMES: {
      lcd.setCursor(0,0); lcd.print("Clear Gate Times?");
      lcd.setCursor(0,1);
      lcd.print(cancelChoice ? "Yes" : "No ");
      break;
    }

    case SET_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print("Set D:HH:MM:SS");
      lcd.setCursor(0,1);
      char bufTime[17];
      sprintf(bufTime, "%03d:%02d:%02d:%02d", setDays, setHours, setMinutes, setSeconds);
      lcd.print(bufTime);
      lcd.setCursor(14,1); lcd.print("Go");

      if (fieldIndex == 0)      lcd.setCursor(0,1);   // Days
      else if (fieldIndex == 1) lcd.setCursor(4,1);   // Hours
      else if (fieldIndex == 2) lcd.setCursor(7,1);   // Minutes
      else if (fieldIndex == 3) lcd.setCursor(10,1);  // Seconds
      else if (fieldIndex == 4) lcd.setCursor(14,1);  // Go
      lcd.cursor();
      break;
    }

    case SET_GATETIMES: {
      lcd.setCursor(0,0);
      lcd.print("TimeSet-");
      lcd.print(currentTriggerIndex + 1);

      DailyTrigger &tr = dailyTriggers[currentTriggerIndex];

      lcd.setCursor(0,1);
      char bufGate[6];
      sprintf(bufGate, "%02d:%02d", tr.hour, tr.minute);
      lcd.print(bufGate);
      lcd.print(tr.enabled ? "  -> On" : "  -> Off");

      if (fieldIndex == 0)      lcd.setCursor(0,1);   // hours
      else if (fieldIndex == 1) lcd.setCursor(3,1);   // minutes
      else if (fieldIndex == 2) lcd.setCursor(10,1);  // On/Off
      lcd.cursor();
      break;
    }

    case RUNNING_COUNTDOWN: {
      lcd.setCursor(0,0); lcd.print("Running");
      lcd.setCursor(0,1);
      long total = endTime.unixtime() - now.unixtime();
      if (total > 0) {
        int dd = total / 86400;
        int hh = (total % 86400) / 3600;
        int mm = (total % 3600) / 60;
        int ss = total % 60;
        char bufRun[16];
        sprintf(bufRun, "%03d:%02d:%02d:%02d", dd, hh, mm, ss);
        lcd.print(bufRun);
      } else {
        lcd.print("Expired");
      }
      break;
    }

    case CANCEL_PROMPT: {
      lcd.setCursor(0,0); lcd.print("Cancel Timer?");
      lcd.setCursor(0,1);
      lcd.print(cancelChoice ? "Yes" : "No ");
      break;
    }

    case SET_CLOCK: {
      lcd.setCursor(0,0); lcd.print("Adjust Clock");
      lcd.setCursor(0,1);
      char bufClock[9];
      sprintf(bufClock, "%02d:%02d:%02d", tempHours, tempMinutes, tempSeconds);
      lcd.print(bufClock);
      lcd.setCursor(10,1); lcd.print("Apply");

      if (fieldIndex == 0)      lcd.setCursor(0,1);
      else if (fieldIndex == 1) lcd.setCursor(3,1);
      else if (fieldIndex == 2) lcd.setCursor(6,1);
      else if (fieldIndex == 3) lcd.setCursor(10,1);
      lcd.cursor();
      break;
    }
  }
}


// --- Part 8: Main Loop ---

void loop() {
  if (watchdogTick) {
    watchdogTick = false;

    // Re‑enable TWI/I²C temporarily to talk to RTC
    PRR &= ~_BV(PRTWI);
    DateTime now = rtc.now();

    // Countdown check
    if (lockActive && now >= endTime) {
      lockActive = false;
      triggerLock();
    }

    // Daily triggers check
    for (int t = 0; t < 5; t++) {
      DailyTrigger &tr = dailyTriggers[t];
      if (tr.enabled &&
          now.hour() == tr.hour &&
          now.minute() == tr.minute &&
          !tr.triggered) {
        triggerLock();
        tr.triggered = true;
      }
      if (tr.enabled &&
         (now.hour() != tr.hour || now.minute() != tr.minute) &&
          tr.triggered) {
        tr.triggered = false;
      }
    }

    if (displayActive && lcdReady && menuState == RUNNING_COUNTDOWN) {
      refreshLCD();
    }
  }

  // Buttons
  checkButton(btnDown, 0, handleDown);
  checkButton(btnUp,   1, handleUp);
  checkButton(btnLeft, 2, handleLeft);
  checkButton(btnRight,3, handleRight);

  // Combo: Up + Down -> SET_CLOCK from anywhere (if awake)
  if (digitalRead(btnUp) == LOW && digitalRead(btnDown) == LOW) {
    menuState = SET_CLOCK;
    DateTime now = rtc.now();
    fieldIndex   = 0;
    tempHours    = now.hour();
    tempMinutes  = now.minute();
    tempSeconds  = now.second();
    if (displayActive && lcdReady) refreshLCD();
  }

  // OPTIONS help scroll (only when awake and not too close to sleep)
  if (millis() - lastButtonPress < (sleepTimeoutMs - 500)) {
    if (menuState == OPTIONS && displayActive && lcdReady && !optionsEditMode) {
      if (millis() - lastOptionsScroll >= optionsScrollIntervalMs) {
        optionsHelpPos++;
        size_t msgLen = strlen(optionsHelpMsg);
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

  updateLocks();

  if (showingMessage && millis() - lastMessageStart >= 2000) {
    showingMessage = false;
    // After messages, return to sensible place
    if (menuState == OPTIONS ||
        menuState == CONFIRM_RESET_COUNTDOWN ||
        menuState == CONFIRM_RESET_GATETIMES) {
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

  if (WakeMessageCheck && displayActive && lcdReady){
    welcomeMessage = true;
    showWelcomeAfterLongSleepIfNeeded();
    WakeMessageCheck = false;
    welcomeMessage = false;
    lastwelcome = rtc.now();
  }
  

  // Periodic HOME refresh for time / battery / running display
  static unsigned long lastRefresh = 0;
  if (menuState == HOME && displayActive && lcdReady) {
    if (millis() - lastRefresh >= 1000) {
      refreshLCD();
      lastRefresh = millis();
    }
  }

  // Sleep logic
  if (pulseActive) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
  } else if (millis() - lastButtonPress >= sleepTimeoutMs) {
    preparePinsForSleep();
    PRR |= _BV(PRTWI);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    cli();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();

    if (wokeFromButton) {
      stagedRestoreAfterButtonWake();
      // Show welcome again if long sleep
      wokeFromButton = false;
    } else if (wokeFromWDT) {
      wokeFromWDT = false;
      // countdown/daily logic handled in loop via watchdogTick
    }
  } else {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
  }
}

