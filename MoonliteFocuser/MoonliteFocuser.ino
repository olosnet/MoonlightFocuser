// Moonlite-compatible stepper controller
// Written by George Carlson June 2014.
// This version uses the Tiny 2.0 a uController based on the Atmel ATMEGA32U4.
// hardware for the remote hand control is not supported.
//
// Many thanks to orly.andico@gmail.com, for the original command parser, others for bits on code picked up here and there on the net
// 28BYJ
// Since the MoonLite focuser only works with positive integers, I center (zero) my system at 30000. The range is from
// 0 to 65535, so 30000 is a good round number for the center.
// If the Current Position, or New Position is set to 0, this code will set the values at 30000. The reason for this
// is the system is designed to be set at center, manually focused, then focused in a +/- fashion by the controller.

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define HOME 0
#define MAXCOMMAND 8
#define ONE_WIRE_BUS 2

#define EEPROM_INITIALIZED_IDX 0
#define EEPROM_POS_START_IDX 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define ON_OFF_DISPLAY_BUTTON_PIN 9
#define FORWARD_BUTTTON_PIN 10
#define BACKWARD_BUTTON_PIN 11
#define SWITCH_SPEED_BUTTON_PIN 12

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
char tempString[6];
unsigned int hashCmd;
long pos;
int isRunning = 0;
int speed = 2;

int eoc = 0;
int idx = 0;
long millisLastMove = 0;
long Current = HOME;
long Target = HOME;
long DistanceToGo = 0;
int minSpeed = 2;
int maxSpeed = 20;

// Temperature measurement
float temperature;
int tempTemp;

// Motor connections
int motorPin1 = 7; // Blue - 28BYJ48 pin 1
int motorPin2 = 6; // Pink - 28BYJ48 pin 2
int motorPin3 = 5; // Yellow - 28BYJ48 pin 3
int motorPin4 = 4; // Orange - 28BYJ48 pin 4

// lookup table for motor phase control
int StepTable[8] = {0b01001, 0b00001, 0b00011, 0b00010, 0b00110, 0b00100, 0b01100, 0b01000};
int phase = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);
bool displayInitialized = false;
bool displayOn = true;
bool manualRunning = false;

unsigned long lastMillis = 0;
unsigned long displayLastMillis = 0;
unsigned long displaySwitchLastMillis = 0;
unsigned long handSpeedSwitchMillis = 0;

void writeLongOnEEPROM(int idx, long value)
{
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  EEPROM.write(idx, four);
  EEPROM.write(idx + 1, three);
  EEPROM.write(idx + 2, two);
  EEPROM.write(idx + 3, one);
}

long readLongFromEEPROM(int idx)
{
  long four = EEPROM.read(idx);
  long three = EEPROM.read(idx + 1);
  long two = EEPROM.read(idx + 2);
  long one = EEPROM.read(idx + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void writeBooleanOnEEPROM(int idx, bool value)
{
  EEPROM.write(idx, value);
}

bool readBooleanFromEEPROM(int idx)
{
  byte res = EEPROM.read(idx);
  return (bool)res;
}

void printDisplayInfo()
{

  if(!isRunning && displayInitialized)
  {
    unsigned long currentMillis = millis();
    unsigned long diff = currentMillis - displayLastMillis;
  
    if (displayLastMillis == 0 || diff >= 700)
    {
  
      // Clean buffer
      display.clearDisplay();
  
      if (displayOn)
      {
  
        int nextPos = SCREEN_HEIGHT / 3;
  
        display.setTextColor(WHITE);
  
        display.setTextSize(2);
  
        // Convert float to stringfloat (arduino sprintf don't support float)
        char str_temperature[6];
        dtostrf(temperature, 4, 2, str_temperature);
  
        char tempBuf[20];
        sprintf(tempBuf, "T:%s C", str_temperature);
  
        // Sposto il cursore a metà altezza del display
        display.setCursor(0, 0);
        display.println(tempBuf);
  
        char posBuf[30];
        sprintf(posBuf, "P:%u", Current);
  
        display.setCursor(0, nextPos);
        display.println(posBuf);
  
        char speedBuf[20];
        sprintf(speedBuf, "S:%u", speed);
  
        display.setCursor(0, nextPos * 2);
        display.println(speedBuf);
      }
  
      
  
      display.display();
  
      displayLastMillis = currentMillis;
    }
  }
}

void setRunning(int running) {

  if(running && isRunning != running && displayInitialized) {

      // Clean buffer
      display.clearDisplay();

        
     int nextPos = SCREEN_HEIGHT / 3;
  
     display.setTextColor(WHITE);
     display.setTextSize(2);

     display.setCursor(0, 0);
     display.println("FOCUSING");
     display.setCursor(0, nextPos);
     display.println("IN");
     display.setCursor(0, nextPos * 2);
     display.println("PROGRESS");

     // Applico la pulizia al display
     display.display();
  }

  isRunning = running;
  
}

void forwardstep()
{
  if (Current <= 65535)
  {
    Current++;
    if (++phase > 7)
      phase = 0;

    setOutput(phase);
    for (int i = 0; i < speed >> 1; i++)
    {
      delay(1);
    }
  }
  else {
    setRunning(0);
    setManualRunning(false);
  }
}

void backwardstep()
{

  if (Current >= 1)
  {
    Current--;
    if (--phase < 0)
      phase = 7;

    setOutput(phase);
    for (int i = 0; i < speed >> 1; i++)
    {
      delay(1);
    }
  } 
  else {
    setRunning(0);
    setManualRunning(false);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(StepTable[out], 0));
  digitalWrite(motorPin2, bitRead(StepTable[out], 1));
  digitalWrite(motorPin3, bitRead(StepTable[out], 2));
  digitalWrite(motorPin4, bitRead(StepTable[out], 3));
}

void requestTemperatures()
{

  if (!isRunning)
  {
    unsigned long currentMillis = millis();
    unsigned long diff = currentMillis - lastMillis;
    bool reqTemp = lastMillis == 0 || diff >= 10000; // Aggiorna la temperatura ogni due secondi dal sensore

    if (reqTemp)
    {
      tempSensor.requestTemperatures();
      temperature = tempSensor.getTempCByIndex(0);
      lastMillis = currentMillis;
    }
  }
}

void switchHandSPeed()
{

  unsigned long currentMillis = millis();
  unsigned long diff = currentMillis - handSpeedSwitchMillis;
  bool canRun = (handSpeedSwitchMillis == 0 || diff >= 200);

  if (canRun)
  {
    switch (speed)
    {
    case 2:
      speed = 4;
      break;
    case 4:
      speed = 8;
      break;
    case 8:
      speed = 10;
      break;
    case 10:
      speed = 20;
      break;
    default:
      speed = 2;
    }

    handSpeedSwitchMillis = currentMillis;
  }
}

void switchDisplayOnOff()
{
  unsigned long currentMillis = millis();
  unsigned long diff = currentMillis - displaySwitchLastMillis;
  bool canRun = (displaySwitchLastMillis == 0 || diff >= 200) && displayInitialized;

  if (canRun)
  {
    displayOn = !displayOn;
    displaySwitchLastMillis = currentMillis;
  }
}

int GetTemp(void)
{
  temperature = tempSensor.getTempCByIndex(0);
  // The returned temperature is in degrees Celcius * 2 for benifit on MoonLite drivers
  return (temperature * 2);
}

long hexstr2long(char *line)
{
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}

void setManualRunning(bool mr)
{

  if (!mr && mr != manualRunning)
  {
  }

  manualRunning = mr;

  if (!mr)
  {
    writeLongOnEEPROM(EEPROM_POS_START_IDX, Current);
    writeBooleanOnEEPROM(EEPROM_INITIALIZED_IDX, true);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// Clear eeprom
void reset()
{
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0);
  }
}

// start of program
void setup()

{

  Serial.begin(9600);
  tempSensor.begin();

  // setup the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  analogReference(DEFAULT);
  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();

  // Verify if eeprom is used
  bool eepromUsed = readBooleanFromEEPROM(EEPROM_INITIALIZED_IDX);

  // If used, read position from eeprom
  if (eepromUsed)
  {
    Current = readLongFromEEPROM(EEPROM_POS_START_IDX);
  }

  // Display Init
  displayInitialized = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if (displayInitialized)
  {
    // Pulisco il buffer
    display.clearDisplay();

    // Applico la pulizia al display
    display.display();
  }
}

// Forever Loop
void loop()
{

  DistanceToGo = Target - Current; /* compute remaining distance to go */

  if (!Serial.available())
  {
    // run the stepper if there's no pending command and if there are pending movements

    if (isRunning)
    {

      if (DistanceToGo > 0)
        forwardstep();

      if (DistanceToGo < 0)
        backwardstep();

      millisLastMove = millis(); /* reset idle timer */
    }

    else
    { /* Check to see if idle time is up */

      if ((millis() - millisLastMove) > 5000)
      {
        // if so, turn off motor
        digitalWrite(motorPin1, 0);
        digitalWrite(motorPin2, 0);
        digitalWrite(motorPin3, 0);
        digitalWrite(motorPin4, 0);
      }
    }

    if (DistanceToGo == 0)
    {
      // if motion is complete
      setRunning(0);
      writeLongOnEEPROM(EEPROM_POS_START_IDX, Current);
      writeBooleanOnEEPROM(EEPROM_INITIALIZED_IDX, true);
    }
  }

  else
  {

    // read the command until the terminating # character

    while (Serial.available() && !eoc)
    {

      inChar = Serial.read();
      if (inChar != '#' && inChar != ':')
      {
        line[idx++] = inChar;

        if (idx >= MAXCOMMAND)
          idx = MAXCOMMAND - 1;
      }

      else
      {
        if (inChar == '#')
          eoc = 1;
      }
    }

  } // end if (!Serial.available())

  // process the command we got

  if (eoc)
  {

    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);

    if (len >= 2)
      strncpy(cmd, line, 2);

    if (len > 2)
      strncpy(param, line + 2, len - 2);

    memset(line, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup

    // :C# is to start a temperature conversion, doesn't require any response (we don't use it)

    hashCmd = (byte(cmd[0]) | (byte(cmd[1]) << 8)); /* combine the two command charaters into an unsigned int */

    switch (hashCmd)
    {
    // GP command Get current position
    case ('P' << 8 | 'G'):
      pos = Current;
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
      break;

    case ('T' << 8 | 'G'):
      // GT command Get Temperature
      tempTemp = temperature * 2;
      sprintf(tempString, "%04X", tempTemp);
      Serial.print(tempString);
      Serial.print("#");
      break;

    case ('I' << 8 | 'G'):
      // GI command 01 if motor running, 00 if not
      if (DistanceToGo != 0)
        Serial.print("01#");
      else
        Serial.print("00#");
      break;

    case ('B' << 8 | 'G'):
      // GB command Get current backlight value, always 00
      Serial.print("00#");
      break;

    case ('H' << 8 | 'P'):
      // PH command Find motor home
      Current = HOME;
      setRunning(1);
      break;

    case ('V' << 8 | 'G'):
      // GV command Get software version, always 10
      Serial.print("10#");
      break;

    case ('N' << 8 | 'G'):
      // GN command Get new (target) position
      pos = Target;
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
      break;

    case ('C' << 8 | 'G'):
      // GC command Get temerature coefficient, always 2
      Serial.print("02#");
      break;

    case ('D' << 8 | 'G'):
      // GD command Get motor speed
      sprintf(tempString, "%02X", speed);
      Serial.print(tempString);
      Serial.print("#");
      break;

    case ('D' << 8 | 'S'):
      // SD command Set motor speed
      speed = hexstr2long(param);
      if (speed < minSpeed)
        speed = minSpeed;
      if (speed > maxSpeed)
        speed = maxSpeed;
      break;

    case ('H' << 8 | 'G'):
      // GH command Get half step mode, always 00
      Serial.print("00#");
      break;

    case ('P' << 8 | 'S'):
      // SP command Set current position
      pos = hexstr2long(param);
      if (pos == 0)
        pos = HOME;
      Current = pos;
      break;

    case ('N' << 8 | 'S'):
      // SN command Set new position
      pos = hexstr2long(param);
      if (pos == 0)
        pos = HOME;
      Target = pos;
      break;

    case ('G' << 8 | 'F'):
      // FG command Start motor command
      setRunning(1);
      break;

    case ('Q' << 8 | 'F'):
      // FQ command Stop motor command
      setRunning(0);
      writeLongOnEEPROM(EEPROM_POS_START_IDX, Current);
      writeBooleanOnEEPROM(EEPROM_INITIALIZED_IDX, true);
      break;
    }

  } // end process command
  else
  {
    // Write on display only if not execute command
    requestTemperatures();
    printDisplayInfo();

    bool forwardButtonPressed = digitalRead(FORWARD_BUTTTON_PIN);
    bool backwardButtonPressed = digitalRead(BACKWARD_BUTTON_PIN);
    bool switchSpeedButtonPressed = digitalRead(SWITCH_SPEED_BUTTON_PIN);
    bool displayOnOffPressed = digitalRead(ON_OFF_DISPLAY_BUTTON_PIN);

    if (forwardButtonPressed)
    {
      if (isRunning)
      {
        setRunning(0);
      }

      setManualRunning(true);
      forwardstep();
    }
    else if (backwardButtonPressed)
    {
      if (isRunning)
      {
        setRunning(0);
      }

      setManualRunning(true);
      backwardstep();
    }
    else if (switchSpeedButtonPressed)
    {
      switchHandSPeed();
      setManualRunning(false);
    }
    else if (displayOnOffPressed)
    {
      switchDisplayOnOff();
      setManualRunning(false);
    }
    else
    {
      setManualRunning(false);
    }
  }

} // end forever loop
