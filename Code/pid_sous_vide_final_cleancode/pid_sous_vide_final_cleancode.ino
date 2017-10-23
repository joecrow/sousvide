#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <LCDKeypad.h>
#include <TimerOne.h>
#include <EEPROM.h>

#define RELAY_PIN 13
#define RELAY_GND 12
#define THERMISTORPIN A1
#define BACKLIGHT_PIN 10


// ************************************************
// Define variables
// ************************************************
int CntMode = 1;                                    // Menu mode 0=Auto, 1=Manual
unsigned long LastRun = 0;
unsigned long SerialLastRun = 0;
unsigned long now;
const long Interval = 300;                          // LCD update
const long SerialInterval = 1000;
int WindowSize = 10000;                             // Window size of the slow PWM, have to slow it down for the 50Hz.
unsigned long windowStartTime;
int Debug = 0;                                      // Print Serial Info

// Control parameters
double Setpoint = 55;
double Input = 0;
double Output = 0;
double MaxTemp = 90;                                // Dont need to go over this temp
double MinTemp = 20;

// LCD Parameters
LCDKeypad lcd;
unsigned long timeout = 0;
const long BacklighTimeout = 1800000;               // 30 Minutes
const int BlockDelay = 300;
const int RepeatDelay = 100;
int light = 1;
volatile byte displayBrightness = 1;                // Brightness from 1 to 4

// Custom Characters
byte TempIcon[8] = //icon for termometer
{
  B00100,
  B01010,
  B01010,
  B01110,
  B01110,
  B11111,
  B11111,
  B01110
};

byte OutputIcon[] = {
  0b01010,
  0b01010,
  0b11111,
  0b10001,
  0b01110,
  0b00100,
  0b11100,
  0b10000
};

byte SetpointIcon[8] = {
  0b00000,
  0b01110,
  0b10001,
  0b10101,
  0b10001,
  0b01010,
  0b00100,
  0b00100
};

byte AutoIcon[8] = {
  0b11111,
  0b11111,
  0b10001,
  0b10101,
  0b10001,
  0b10101,
  0b11111,
  0b11111
};

byte ManIcon[8] = {
  0b11111,
  0b11111,
  0b10101,
  0b10001,
  0b10101,
  0b10101,
  0b11111,
  0b11111
};

// PID Parameters
const double Kp = 850;
const double Ki = 0.5;
const double Kd = 0.2;

// Setup PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int PID_Sample_time = 1000;

// Temperature Setup
const int NumSamples = 5;
const long SeriesResistor = 10000;
const long ThermistorNominal = 10000;
const int TemperatureNominal = 25;                  // Temperature of nominal resistance of thermister
const long BCoefficient = 3970;
int samples[NumSamples];

// EEPROM
int SetpointADDR = 0;
int EEPROM_SaveEnable = 0;
long EEPROM_SaveTime = 10000;                       // Delays Between Saves

// ************************************************
// Setup
// ************************************************
void setup()
{
  windowStartTime = millis();
  if (Debug == 1) {
    Serial.begin(115200);
  }
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_GND, OUTPUT);
  digitalWrite(RELAY_GND, LOW);

  double Setpoint_tmp;
  EEPROM.get(SetpointADDR, Setpoint_tmp);
  if (Setpoint_tmp > 0 && Setpoint_tmp < MaxTemp) {
    Setpoint = Setpoint_tmp;
  }

  //initialize the LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.backlight();
  light = 1;
  timeout = millis() + BacklighTimeout;
  
  //LCD Balclight PWM
  Timer1.initialize();
  Timer1.attachInterrupt(lcdBacklightISR, 500);
  setBacklightBrightness(displayBrightness);
  lcd.createChar(1, TempIcon);
  lcd.createChar(2, OutputIcon);
  lcd.createChar(3, SetpointIcon);
  lcd.createChar(4, AutoIcon);
  lcd.createChar(5, ManIcon);

  //turn the PID on
  myPID.SetSampleTime(PID_Sample_time);
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(MANUAL);
}

// ************************************************
// Main Loop
// ************************************************
void loop()
{
  if (Debug == 1) {
    // ************************************************
    // Serial Connection
    // ************************************************
    if (millis() - SerialLastRun >= SerialInterval) {
      SerialLastRun = millis();
      Serial.print(millis() / 1000);
      Serial.print(",");
      Serial.print(Input);
      Serial.print(",");
      Serial.print(Setpoint);
      Serial.print(",");
      Serial.print(Output / WindowSize * 100);
      Serial.print(",");
      Serial.print(Output);
      Serial.print(",");
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki);
      Serial.print(",");
      Serial.println(Kd);
    }
  }

  // ************************************************
  // Temperature Sensing
  // ************************************************
  Input = GetTemp();

  // ************************************************
  // Run PID
  // ************************************************
  myPID.Compute();

  // ************************************************
  // turn the output pin on/off based on pid output
  // ************************************************
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, LOW);
  else digitalWrite(RELAY_PIN, HIGH);

  // ************************************************
  // Buttons and menu
  // ************************************************
  switch (lcd.buttonBlocking(BlockDelay, RepeatDelay)) {

    case KEYPAD_RIGHT:
      timeout = now + BacklighTimeout;
      if (light == 0) {
        LCDWake ();
        break;
      }
      if ( CntMode == 0) {
        Setpoint += 1.0;
        Setpoint = constrain(Setpoint, MinTemp, MaxTemp);
        EEPROM_SaveEnable = 1;
      }
      else if ( CntMode == 1) {
        Output += 5.0 * WindowSize / 100;
        Output = constrain(Output, 0, WindowSize);
      }
      break;

    case KEYPAD_UP:
      timeout = now + BacklighTimeout;
      if (light == 0) {
        LCDWake ();
        break;
      }
      if ( CntMode == 0) {
        Setpoint += 0.1;
        Setpoint = constrain(Setpoint, MinTemp, MaxTemp);
        EEPROM_SaveEnable = 1;
      }
      else if ( CntMode == 1) {
        Output += 1 * WindowSize / 100;
        Output = constrain(Output, 0, WindowSize);
      }
      break;

    case KEYPAD_DOWN:
      timeout = now + BacklighTimeout;
      if (light == 0) {
        LCDWake ();
        break;
      }
      if ( CntMode == 0) {
        Setpoint -= 0.1;
        Setpoint = constrain(Setpoint, MinTemp, MaxTemp);
        EEPROM_SaveEnable = 1;
      }
      else if ( CntMode == 1) {
        Output -= 1 * WindowSize / 100;
        Output = constrain(Output, 0, WindowSize);
      }
      break;

    case KEYPAD_LEFT:
      timeout = now + BacklighTimeout;
      if (light == 0) {
        LCDWake ();
        break;
      }
      if ( CntMode == 0) {
        Setpoint -= 1.0;
        Setpoint = constrain(Setpoint, MinTemp, MaxTemp);
        EEPROM_SaveEnable = 1;
      }
      else if ( CntMode == 1) {
        Output -= 5.0 * WindowSize / 100;
        Output = constrain(Output, 0, WindowSize);
      }
      break;

    case KEYPAD_SELECT:
      timeout = now + BacklighTimeout;
      if (light == 0) {
        LCDWake ();
        break;
      }
      if (  CntMode == 0) {
        myPID.SetMode(MANUAL);
        CntMode = 1;
      }
      else if ( CntMode == 1) {
        CntMode = 0;
        myPID.SetMode(AUTOMATIC);
      }
      break;
    default:
      if (millis() - LastRun >= Interval) {
        LastRun = millis();
        updateLCD();
      }
      break;
  }

  // ************************************************
  // Timers
  // ************************************************
  now = millis();

  // Turn off backlight
  if (now > timeout && light == 1) {
    lcd.noBacklight();
    light = 0;
  }

  if (analogRead(A0) < 1000) {
    updateLCD();
  }

  // Save to EEPROM
  if (now > EEPROM_SaveTime && EEPROM_SaveEnable == 1) {
    EEPROM.put(SetpointADDR, Setpoint );
    EEPROM_SaveEnable = 0;
  }
}

// ************************************************
// Functions
// ************************************************
void updateLCD()
{
  lcd.setCursor(0, 0);
  lcd.write(byte(1));
  lcd.print(" ");
  lcd.print(Input, 2);
  lcd.print((char)223);
  lcd.print("C");

  // Mode Indicator
  lcd.setCursor(15, 0);
  if ( CntMode == 1) {
    lcd.write(byte(5));
  }
  else if ( CntMode == 0) {
    lcd.write(byte(4));
  }

  // Line 2
  lcd.setCursor(0, 1);
  if ( CntMode == 1) {
    lcd.write(byte(2));
    lcd.print(" ");
    lcd.print(Output / WindowSize * 100, 0);
    lcd.print("%");
    lcd.print("          ");
  }
  else if ( CntMode == 0) {
    lcd.write(byte(3));
    lcd.print(" ");
    lcd.print(Setpoint, 2);
    lcd.setCursor(10, 1);
    lcd.write(byte(2));
    lcd.print(" ");
    lcd.print(Output / WindowSize * 100, 0);
    lcd.print("% ");
  }
}

// Read Temperature
float GetTemp() {
  uint8_t i;
  float average;
  // take N samples in a row, with a slight delay
  for (i = 0; i < NumSamples; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }
  // average all the samples out
  average = 0;
  for (i = 0; i < NumSamples; i++) {
    average += samples[i];
  }
  average /= NumSamples;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SeriesResistor / average;

  float steinhart;
  steinhart = average / ThermistorNominal;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TemperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

  return steinhart;
}

// LCD Soft PWM
void lcdBacklightISR()
{
  const byte dutyCycle = 4;
  static byte pulseWidth;
  if (!light)
  {
    return;
  }
  if (pulseWidth > dutyCycle)
  {
    pulseWidth = 0;
    //back light On
    pinMode(BACKLIGHT_PIN, INPUT);
  }
  else if (pulseWidth > displayBrightness)
  {
    //back light off
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, LOW);
  }
  pulseWidth++;
}
void setBacklightBrightness(byte brightness)
{
  displayBrightness = constrain(brightness, 1, 4);
}

// LCD Wake
void LCDWake () {
  lcd.backlight();
  light = 1;
  timeout = now + BacklighTimeout;
}

