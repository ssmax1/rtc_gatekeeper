// --- Part 1: Includes, Globals, Enums ---

#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <Servo.h>

// LCD pins
const int rs = 4, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Heart Symbol
byte heart[8] = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};


// Single Servo
Servo releaseServo;
const int servoPin = 5;        // signal pin
const int servoPower = A2;     // servo power mosfet gate
const int servoOpen = 95;     // adjust for your mechanism
const int servoClosed = 47;    // adjust for your mechanism
bool servoIsOpen = false;   // tracks manual servo state
static bool comboHandled = false;
long overloadDrop = 50;
long tempMulti = 1;

int diagIndex = 0; 
float currentTempC = 0.0;

// Last success stats
long lastSuccessVcc = 0;
long lastSuccessAvg = 0;
long lastSuccessEnd = 0;
int lastSuccessAttempt = 0;
float lastSuccessTemp = 0.0;



// Last fail stats
long lastFailVcc = 0;
long lastFailAvg = 0;
long lastFailEnd = 0;
int lastFailAttempt = 0;
float lastFailTemp = 0.0;

// Last clost stats
long lastClose_avgDrop = 0;
long lastClose_adaptiveThr = 0;
long lastClose_maxDrop = 0;
bool lastClose_overload = false;

enum PulseOutcome {
  OUT_NONE,
  OUT_LOCK_OPENED,
  OUT_OPENED_RETRY,
  OUT_FAIL_MAX
};

PulseOutcome lastOutcome = OUT_NONE;

const char* const outcomeMsg[] = {
  "",
  "Latch Opened",
  "Opened w/Retry",
  "Retry's Failed"
};

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

// Watchdog flags
volatile bool watchdogTick = false;
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

// Options help scroll
const char optionsHelpMsg[] = "Options: Up/Down to change option or values; Right to edit/confirm;   ";
static unsigned int optionsHelpPos = 0;
static unsigned long lastOptionsScroll = 0;
const unsigned long optionsScrollIntervalMs = 300;

// Sleep tracking using RTC
DateTime lastwelcome;


// --- Part 2: Watchdog, ISRs, Setup ---

ISR(WDT_vect) {
  watchdogTick = true;
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
  // Servo Power Off On Startup
  pinMode(servoPower, INPUT);
  digitalWrite(servoPower, LOW);

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
  lcd.createChar(0, heart);
  lcd.setCursor(0,0); lcd.write(byte(0)); lcd.print(" Welcome   To "); lcd.write(byte(0));
  lcd.setCursor(0,1); lcd.write(byte(0)); lcd.print(" Gate  Keeper "); lcd.write(byte(0));
  delay(1500);
  lcd.clear();
  lcd.setCursor(0,0);  lcd.write(byte(0)); lcd.print("Velkommen  Til"); lcd.write(byte(0));
  lcd.setCursor(0,1);  lcd.write(byte(0)); lcd.print(" Portvogteren "); lcd.write(byte(0));
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
  pinMode(servoPower, INPUT);
  digitalWrite(servoPower, LOW);

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
    lcd.createChar(0, heart);
    lcd.clear();
    lcd.setCursor(0,0); lcd.write(byte(0)); lcd.print(" Welcome   To "); lcd.write(byte(0));
    lcd.setCursor(0,1); lcd.write(byte(0)); lcd.print(" Gate  Keeper "); lcd.write(byte(0));
    delay(1500);
    lcd.clear();
    lcd.setCursor(0,0);  lcd.write(byte(0)); lcd.print("Velkommen  Til"); lcd.write(byte(0));
    lcd.setCursor(0,1);  lcd.write(byte(0)); lcd.print(" Portvogteren "); lcd.write(byte(0));
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
  pinMode(servoPower, OUTPUT);
  digitalWrite(servoPower, HIGH);

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


// --- Part 4: Button Logic ---

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
    case HOME:
      // No action on Up at Home
      break;

    case DIAGNOSTICS:
      diagIndex = (diagIndex + 5) % 6;
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
        } else if (optionsFieldIndex == 1) {   // Overload drop
          overloadDrop += 10;
          if (overloadDrop > 250) overloadDrop = 50;
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
      if ((fieldIndex == 0 || fieldIndex == 1) && !tr.enabled ) tr.enabled = true;
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
    
    case DIAGNOSTICS:
      diagIndex = (diagIndex + 1) % 6;
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
        } else if (optionsFieldIndex == 1) {   // Overload drop
          overloadDrop -= 10;
          if (overloadDrop < 50) overloadDrop = 250;
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
      if (fieldIndex == 0)      tr.hour = (tr.hour + 23) % 24;
      else if (fieldIndex == 1) tr.minute = (tr.minute + 59) % 60;
      if ((fieldIndex == 0 || fieldIndex == 1) && !tr.enabled ) tr.enabled = true;
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

    case DIAGNOSTICS:
      menuState = HOME;
      needsRefresh = true;
      break;    


    case MODE_SELECT:
      // Right selects current item
      if (fieldIndex == 0) { // Gate Times
        menuState = SET_GATETIMES;
        currentTriggerIndex = 0;
        fieldIndex = 0;
      } else if (fieldIndex == 1) { // Countdown
        // If countdown active, go to RUNNING; else to SET_COUNTDOWN
        if (lockActive) menuState = RUNNING_COUNTDOWN;
        else {
          menuState = SET_COUNTDOWN;
          fieldIndex = 0;
        }
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
          // Enter edit mode (Timeout or overload)
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
      menuState = DIAGNOSTICS;
      diagIndex = 0;
      needsRefresh = true;
      break;

    case DIAGNOSTICS:
      menuState = HOME;
      needsRefresh = true;
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
        fieldIndex = 1;
      }
      break;

    case SET_GATETIMES:
      if (fieldIndex > 0) {
        fieldIndex--;
      } else {
        // Back to MODE_SELECT
        menuState = MODE_SELECT;
        fieldIndex = 0; // highlight Gate Times
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


bool servoOpenWithRetry(int maxRetries = 20) {

  auto attemptOpen = [&](int attemptNum) -> bool {
    releaseServo.attach(servoPin);
    delay(300);
    long baseline = readVcc();
    long dropSum = 0;
    long maxDrop = 0;
    int  samples = 0;
    int delayMs = 10;          // starting speed
    const int minDelay = 5;    // fastest allowed
    const int maxDelay = 50;   // slowest allowed
    const int targetDrop = 300;

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
      // --- Adaptive speed control ---
      if (drop > targetDrop ) {
        // too much load → slow down
        delayMs += 5;
        if (delayMs > maxDelay) delayMs = maxDelay;
      } else if (drop < targetDrop - 20) {
        // plenty of headroom → speed up
        delayMs -= 2;
        if (delayMs < minDelay) delayMs = minDelay;
      }


      if (drop > 0) {
        dropSum += drop;
        samples++;
        if (drop > maxDrop) maxDrop = drop;
      }
    }

    delay(800 * tempMulti);
    long endVcc = readVcc();
    releaseServo.detach();

    long avgDrop = (samples > 0) ? dropSum / samples : 0;

    // Updated thresholds for your system
    bool moved      = avgDrop > 20;     // servo definitely drew current
    bool recovered  = baseline - endVcc <= 20;     // Vcc bounced back

    bool success = moved && recovered;

    if (success){
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

    // LCD diagnostics for this attempt
    if (displayActive && lcdReady) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Attempt ");
      lcd.print(attemptNum);
      lcd.print(success ? " OK" : " FAIL");

      lcd.setCursor(0,1);
      lcd.print("A:");
      lcd.print(avgDrop);
      lcd.print(" M:");
      lcd.print(maxDrop);
      lcd.print(" R:");
      lcd.print(baseline - endVcc);
      delay(1000);
    }

    return success;
  };

  // --- Retry loop ---
  for (int attempt = 1; attempt <= maxRetries; attempt++) {

    if (attemptOpen(attempt)) {
      servoIsOpen = true;

      if (displayActive && lcdReady) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Latch Opened");
        lcd.setCursor(0,1); lcd.print("After ");
        lcd.print(attempt);
        lcd.print(" tries");
        delay(600);
        refreshLCD();
      }

      return true;
    }

    // --- Failed attempt: close safely before retrying ---
    safeCloseServo();
    delay(300);
  }

  // --- All attempts failed ---
  if (displayActive && lcdReady) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Open FAILED");
    lcd.setCursor(0,1); lcd.print("Check mechanism");
    delay(2000);
    refreshLCD();
  }
  
  lastOutcome = OUT_FAIL_MAX;
  return false;
}


bool safeCloseServo() {
  const int maxRetries     = 3;
  const int maxRetries_c   = 10;
  lastClose_overload    = false;

  releaseServo.attach(servoPin);

  for (int attempt = 0; attempt < maxRetries; attempt++) {

    long baselineVcc = readVcc();

      // --- ADAPTIVE THRESHOLD BASED ONLY ON LAST SUCCESS ---
    long adaptiveThreshold = lastSuccessAvg + overloadDrop;

      // Reset diagnostics
    lastClose_avgDrop     = 0;
    lastClose_maxDrop     = 0;
        
    lastClose_adaptiveThr = adaptiveThreshold;
    long totalDrop = 0;
    int  samples   = 0;
    int pos_c = servoOpen;
    int attempt_c = 0;
    // --- INNER RETRIES FOR THE ACTUAL CLOSING MOTION ---
    retry_close:
    if ( attempt_c < maxRetries_c) {
      attempt_c++;  
      for (; pos_c >= servoClosed; pos_c--) {

        releaseServo.write(pos_c);
        delay(25 * tempMulti);

        long vccNow = readVcc();
        long drop   = baselineVcc - vccNow;

        // Track max drop
        if (drop > lastClose_maxDrop) {
          lastClose_maxDrop = drop;
        }

        // Track avg drop dynamically
        if (drop >= 0 ) {
        totalDrop += drop;
        samples++;
        lastClose_avgDrop = totalDrop / samples;
        }

        // Overload detection
        if (drop > adaptiveThreshold && pos_c < servoOpen - 3) {
          lastClose_overload = true;
          int backpos = pos_c + 16;
          if (backpos > servoOpen) {
            backpos = servoOpen;
          }
          for (int back = pos_c; back <= backpos; back++) {
            releaseServo.write(back);
            delay(10);
          }
          pos_c = backpos;
          delay(200);
          goto retry_close;
        }

        // Reached closed position
        if (pos_c == servoClosed) {
          servoIsOpen = false;
          delay(800 * tempMulti);
          releaseServo.detach();
          return true;
        }
      }
    }

    // Full reopen before outer retry
    for (int back = pos_c; back <= servoOpen; back++) {
      releaseServo.write(back);
      delay(10);
    }
    delay(2000);
  }

  releaseServo.detach();
  return false;
}


void triggerLock() {
  if (!displayActive && !lcdReady) {
    stagedRestoreAfterButtonWake();
  }

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Gate Triggered!");
  lastMessageStart = millis();
  showingMessage = true;

  // Attach servo
  releaseServo.attach(servoPin);

  // open latch
  servoOpenWithRetry();

  delay(5000);

  // close latch
  bool closedOK = safeCloseServo();
  if (!closedOK) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Latch Overload!");
    lcd.setCursor(0,1); lcd.print("Check mechanism");
    delay(4000);
  }
  


  // Detach to save power
  releaseServo.detach();

}

bool getNextGateTime(int &outHour, int &outMinute) {
  DateTime now = rtc.now();
  int nowMinutes = now.hour() * 60 + now.minute();

  int bestDiff = 24 * 60 + 1;  // larger than any possible diff
  bool found = false;

  for (int t = 0; t < 5; t++) {
    DailyTrigger &tr = dailyTriggers[t];
    if (!tr.enabled) continue;

    int triggerMinutes = tr.hour * 60 + tr.minute;
    int diff = triggerMinutes - nowMinutes;
    if (diff < 0) diff += 24 * 60;  // wrap to next day

    if (diff < bestDiff) {
      bestDiff = diff;
      outHour = tr.hour;
      outMinute = tr.minute;
      found = true;
    }
  }

  return found;
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
      static int homeScreenMode = 0;  // 0=battery, 1=time, 2=next gate

      unsigned long interval = (currentTempC < 0.0f) ? 6000UL : 3000UL;

      if (millis() - lastAlt >= interval) {
        homeScreenMode = (homeScreenMode + 1) % 3;
        lastAlt = millis();
      }

      lcd.setCursor(0,0);
      if (homeScreenMode == 0) {
        // Battery
        long vcc = readVcc();
        float tC = currentTempC;
        lcd.print("B ");
        lcd.print(vcc / 1000.0, 2);
        lcd.print("v T ");
        lcd.print(tC, 1);
        lcd.print("c");
      } else if (homeScreenMode == 1) {
        // Time
        lcd.print("Time ");
        char buf[9];
        sprintf(buf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        lcd.print(buf);
      } else if (homeScreenMode == 2) {
        // Next gate time
        int nh, nm;
        if (getNextGateTime(nh, nm)) {
          lcd.print("Next Gate ");
          char buf[6];
          sprintf(buf, "%02d:%02d", nh, nm);
          lcd.print(buf);
        } else {
          lcd.print("All Times OFF");
        }
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
      if (fieldIndex == 0)      lcd.print("> Gate Times");
      else if (fieldIndex == 1) lcd.print("> Countdown");
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
        lcd.print("Overload ");
        lcd.print(overloadDrop);
        lcd.print("mV");
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

        case DIAGNOSTICS: {
      lcd.clear();
      lcd.noCursor();

      if (diagIndex ==0) {
        lcd.setCursor(0,0);
        lcd.print("Last Outcome:");
        lcd.setCursor(0,1);
        lcd.print(outcomeMsg[lastOutcome]);
      } else if (diagIndex == 1) {
        // Last Successful Open
        lcd.setCursor(0,0);
        lcd.print("V "); lcd.print(lastSuccessVcc);
        lcd.print(" A "); lcd.print(lastSuccessAvg);

        lcd.setCursor(0,1);
        lcd.print("d "); lcd.print(lastSuccessEnd);
        lcd.print(" Attempt "); lcd.print(lastSuccessAttempt);
      } else if (diagIndex == 2) {
        // Last Successful Open
        lcd.setCursor(0,0);
        lcd.print("VF "); lcd.print(lastFailVcc);
        lcd.print(" AF "); lcd.print(lastFailAvg);

        lcd.setCursor(0,1);
        lcd.print("dF "); lcd.print(lastFailEnd);
        lcd.print(" Att "); lcd.print(lastFailAttempt);
      } else if (diagIndex == 3) {
        // Last Successful Open
        lcd.setCursor(0,0);
        lcd.print("Last Okay:");
        lcd.print(lastSuccessTemp, 1);
        lcd.print("c");

        lcd.setCursor(0,1);
        lcd.print("Last Fail:");
        lcd.print(lastFailTemp, 1);
        lcd.print("c");
      } else if (diagIndex == 4) {        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("C avg ");
        lcd.print(lastClose_avgDrop);
        lcd.print(" t ");
        lcd.print(lastClose_adaptiveThr);

        lcd.setCursor(0,1);
        lcd.print("mx ");
        lcd.print(lastClose_maxDrop);
        lcd.print(" OL:");
        lcd.print(lastClose_overload ? "Y" : "N");
      } else if (diagIndex == 5) {
        lcd.setCursor(0,0);
        lcd.print("To Exit Press");
        lcd.setCursor(0,1);
        lcd.print("Right or Left");
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

  if (watchdogTick) {
    watchdogTick = false;

    // Re‑enable TWI/I²C temporarily to talk to RTC
    PRR &= ~_BV(PRTWI);
    Wire.begin();
    delay(10);
    DateTime now = rtc.now();
    currentTempC = rtc.getTemperature();
    tempMulti = (currentTempC < -8.0f) ? 3 : (currentTempC <   0.0f) ? 2 : 1;

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

  // Combo: Up + Down -> toggle servo state safely
  if (digitalRead(btnUp) == LOW && digitalRead(btnDown) == LOW) {


      if (!comboHandled) {
        comboHandled = true;

        // Wake display if needed
        if (!displayActive || !lcdReady) {
          stagedRestoreAfterButtonWake();
        }

        if (servoIsOpen) {
          // Try safe close
          bool ok = safeCloseServo();
          if (!ok) {
            lcd.clear();
            lcd.setCursor(0,0); lcd.print("Latch Overload!");
            lcd.setCursor(0,1); lcd.print("Check Mechanism");
            delay(1500);
          } else {
            lcd.clear();
            lcd.setCursor(0,0); lcd.print("Latch CLOSED");
            delay(600);
          }
        } else {
          // Open normally
          servoOpenWithRetry();

        }

        refreshLCD();
      }

  } else {
    comboHandled = false;
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
  if (millis() - lastButtonPress >= sleepTimeoutMs) {
    sleeping = true;
    preparePinsForSleep();
    PRR |= _BV(PRTWI);
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

