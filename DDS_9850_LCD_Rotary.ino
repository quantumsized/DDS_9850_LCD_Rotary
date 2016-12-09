// include the DDS Library:
#include <DDS.h>
#include <Wire.h>    // I2C-Integrated library
#include <LiquidCrystal_I2C.h> // Display library
//======================================
// AD9850 Module....
// set pin numbers:

const int W_CLK = 8;    // Pin 8 - connect to AD9850 module word load clock pin (CLK)
const int FQ_UD = 9;    // Pin 9 - connect to freq update pin (FQ)
const int DATA = 10;    // Pin 10 - connect to reset pin (RST)
const int RESET = 11;   // Pin 11 - connect to serial data load pin (DATA)

// define LCD address
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#define LCD_Backlight 6 // Define backlight driver pin

// Half-step mode?
#define HALF_STEP
// Arduino pins the encoder is attached to. Attach the center to ground.
#define ROTARY_PIN1 2
#define ROTARY_PIN2 3
// define to enable weak pullups.
#define ENABLE_PULLUPS
const int buttonPin = 4;       // the number of the pushbutton pin
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

#define DIR_CCW 0x10
#define DIR_CW 0x20

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
const unsigned char ttable[6][4] = {
  {0x3 , 0x2, 0x1,  0x0}, {0x23, 0x0, 0x1,  0x0},
  {0x13, 0x2, 0x0,  0x0}, {0x3 , 0x5, 0x4,  0x0},
  {0x3 , 0x3, 0x4, 0x10}, {0x3 , 0x5, 0x3, 0x20}
};
#else
// Use the full-step state table (emits a code at 00 only)
const unsigned char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x10},
  {0x6, 0x5, 0x4,  0x0},
};
#endif
volatile unsigned char state = 0;

/* Call this once in setup(). */
void rotary_init() {
  pinMode(ROTARY_PIN1, INPUT);
  pinMode(ROTARY_PIN2, INPUT);
#ifdef ENABLE_PULLUPS
  digitalWrite(ROTARY_PIN1, HIGH);
  digitalWrite(ROTARY_PIN2, HIGH);
#endif
}

/* Read input pins and process for events. Call this either from a
 * loop or an interrupt (eg pin change or timer).
 *
 * Returns 0 on no event, otherwise 0x80 or 0x40 depending on the direction.
 */
unsigned char rotary_process() {
  unsigned char pinstate = (digitalRead(ROTARY_PIN2) << 1) | digitalRead(ROTARY_PIN1);
  state = ttable[state & 0xf][pinstate];
  return (state & 0x30);
}

double freq = 1000;

boolean setStepVal = false;

double stepVal[] = {.1, 1, 10, 100, 1000, 10000, 100000, 1000000};
double maxFreq = 40000000;
const int stepArrCount = 7;
int currentStepVal = 4;

// Instantiate the DDS...
DDS dds(W_CLK, FQ_UD, DATA, RESET);

void setup() {
  lcd.init(); // initialize LCD
  lcd.backlight();
  pinMode(LCD_Backlight, OUTPUT);
  analogWrite(LCD_Backlight, 64); // Set backlight brightness value: 0-128
  rotary_init();
  pinMode (buttonPin, INPUT_PULLUP); // uses internal pullup resistor alturnatly you can set this externally
  //start wire and lcd
  Wire.begin();
  // start up the DDS...
  dds.init();
  // (Optional) trim if your xtal is not at 125MHz...
  dds.trim(125000000); // enter actual osc freq
  // start the oscillator...
  dds.setFrequency(freq);
  clearLCD();
  lcd.print("Frequency in Hz");
  lcd.setCursor(0, 1);
  lcd.print(freq);
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == LOW) {
        if (setStepVal == false) {
          setStepVal = true;
          clearLCD();
          lcd.print("Step Freq by Val");
          lcd.setCursor(0, 1);
          lcd.print(stepVal[currentStepVal]);
        } else if (setStepVal == true) {
          setStepVal = false;
          clearLCD();
          lcd.print("Frequency in Hz");
          lcd.setCursor(0, 1);
          lcd.print(freq);
        }
      }
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;

  unsigned char result = rotary_process();
  if (result) {
    if (setStepVal == false) {
      lcd.setCursor(0, 0);
      if (result == DIR_CCW) {
        freq = freq + stepVal[currentStepVal];
        if (freq >= maxFreq) {
          freq = maxFreq;
        }
        dds.setFrequency(freq);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(freq);
      } else if (result == DIR_CW) {
        freq = freq - stepVal[currentStepVal];
        if (freq <= 0) {
          freq = 0;
        }
        dds.setFrequency(freq);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(freq);
      }
    } else if (setStepVal == true) {
      if (result == DIR_CCW) {
        currentStepVal = currentStepVal + 1;
        if (currentStepVal >= stepArrCount) {
          currentStepVal = stepArrCount;
        }
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(stepVal[currentStepVal]);
      } else if (result == DIR_CW) {
        currentStepVal = currentStepVal - 1;
        if (currentStepVal <= 0) {
          currentStepVal = 0;
        }
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(stepVal[currentStepVal]);
      }
    }
  }
}
//easy and dirty way to clear the LCD
void clearLCD () {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
}
