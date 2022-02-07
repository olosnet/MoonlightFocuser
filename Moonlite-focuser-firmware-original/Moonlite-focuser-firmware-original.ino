// Moonlite-compatible stepper controller
// Written by George Carlson June 2014.
// This version uses the Tiny 2.0 a uController based on the Atmel ATMEGA32U4.
// hardware for the remote hand control is not supported.
//
// Many thanks to orly.andico@gmail.com, for the original command parser, others for bits on code picked up here and there on the net
//28BYJ
// Since the MoonLite focuser only works with positive integers, I center (zero) my system at 30000. The range is from
// 0 to 65535, so 30000 is a good round number for the center.
// If the Current Position, or New Position is set to 0, this code will set the values at 30000. The reason for this
// is the system is designed to be set at center, manually focused, then focused in a +/- fashion by the controller.

#define RUNLED 11 /* Amber LED lights whenever motor is active, 11 for the Teensy */
#define HOME 30000
#define MAXCOMMAND 8

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
int Current = HOME;
int Target = HOME;
int DistanceToGo = 0;
int minSpeed = 2;
int maxSpeed = 20;
int testPin = 12;

// Temperature measurement
int temperatureChannel = 0;
unsigned int wADC;
double temperature;
int tempTemp;
#define SCALE 0.488 /* ADC>centigrade scale for my particular Teensy */
#define OFFSET 100 /* 2 * 50 offset for TPM-36 */

// Remote hand controller NOT USED ON TEENSY VERSION (but could be)
int handController = 1;
unsigned int rADC;
int speedTable[16] = {2, 2, 4, 8, 10, 20, 0, 0, 0, 0, 20, 10, 8, 4, 2, 2};
int goTable[16] = { -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};

// Motor connections
int motorPin1 = 7; // Blue - 28BYJ48 pin 1
int motorPin2 = 6; // Pink - 28BYJ48 pin 2
int motorPin3 = 5; // Yellow - 28BYJ48 pin 3
int motorPin4 = 4; // Orange - 28BYJ48 pin 4
// Red - 28BYJ48 pin 5 (VCC)

// lookup table for motor phase control
int StepTable[8] = {0b01001, 0b00001, 0b00011, 0b00010, 0b00110, 0b00100, 0b01100, 0b01000};
int phase = 0;

void forwardstep() {

  Current++;
  if (++phase > 7) phase = 0;
  //stepper1.move(1);
  setOutput(phase);
  for (int i = 0; i < speed >> 1; i++) {
    delay(1);
  }
}

void backwardstep()
{
  Current--;
  if (--phase < 0) phase = 7;

  setOutput(phase);
  for (int i = 0; i < speed >> 1; i++) {
    delay(1);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(StepTable[out], 0));
  digitalWrite(motorPin2, bitRead(StepTable[out], 1));
  digitalWrite(motorPin3, bitRead(StepTable[out], 2));
  digitalWrite(motorPin4, bitRead(StepTable[out], 3));
}

int GetTemp(void)
{

  wADC = analogRead(temperatureChannel);
  wADC = 20;
  if (wADC < 20)
    wADC = 20; /* no temperture sensor attached (avoid noise)*/
  temperature = ((wADC * SCALE) - OFFSET);

  // The returned temperature is in degrees Celcius * 2 for benifit on MoonLite drivers
  return (temperature);
}

void readHandController()
{
  //digitalWrite(testPin,LOW);
  //rADC = analogRead(handController); /* read twice, keep second reading */
  rADC = analogRead(handController);
  analogReference(DEFAULT);

  if (rADC > 20)
    // if hand controller is connected
  {
    rADC = rADC >> 6;
    if (goTable[rADC])
    {
      speed = speedTable[rADC];
      Target = Target + goTable[rADC];
      isRunning = 1;
      //digitalWrite(RUNLED,HIGH);
    }
  }
  // digitalWrite(testPin,HIGH);
}

long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// start of program

void setup()

{

  Serial.begin(9600);



  //setup the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  //pinMode(RUNLED,OUTPUT); /* yellow LED */
  //pinMode(testPin, OUTPUT);



  analogReference(DEFAULT);
  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();

}

// Forever Loop
void loop() {

  // readHandController(); Not used in Teensy Version

  DistanceToGo = Target - Current; /* compute remaining distance to go */

  if (!Serial.available()) {
    // run the stepper if there's no pending command and if there are pending movements

    if (isRunning) {

      if (DistanceToGo > 0)
        forwardstep();

      if (DistanceToGo < 0)
        backwardstep();

      millisLastMove = millis(); /* reset idle timer */
    }

    else { /* Check to see if idle time is up */

      if ((millis() - millisLastMove) > 5000) {
        // if so, turn off motor
        digitalWrite(motorPin1, 0);
        digitalWrite(motorPin2, 0);
        digitalWrite(motorPin3, 0);
        digitalWrite(motorPin4, 0);
      }

    }

    if (DistanceToGo == 0) {
      // if motion is complete
      //digitalWrite(RUNLED,LOW);
      isRunning = 0;

    }

  }

  else {

    // read the command until the terminating # character

    while (Serial.available() && !eoc) {

      inChar = Serial.read();
      if (inChar != '#' && inChar != ':') {
        line[idx++] = inChar;

        if (idx >= MAXCOMMAND)
          idx = MAXCOMMAND - 1;
      }

      else {
        if (inChar == '#')
          eoc = 1;
      }
    }

  } // end if (!Serial.available())



  // process the command we got

  if (eoc) {
    //digitalWrite(testPin,LOW);

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

    switch (hashCmd) {
      // GP command Get current position
      case ('P'<<8 | 'G'):
        pos = Current;
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('T'<<8 | 'G'):
        // GT command Get Temperature
        tempTemp = GetTemp();
        sprintf(tempString, "%04X", tempTemp);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('I'<<8 | 'G'):
        // GI command 01 if motor running, 00 if not
        if (DistanceToGo != 0)
          Serial.print("01#");
        else
          Serial.print("00#");
        break;

      case ('B'<<8 | 'G'):
        // GB command Get current backlight value, always 00
        Serial.print("00#");
        break;

      case ('H'<<8 | 'P'):
        // PH command Find motor home
        Current = HOME;
        isRunning = 1;
        //digitalWrite(RUNLED,HIGH);
        break;

      case ('V'<<8 | 'G'):
        // GV command Get software version, always 10
        Serial.print("10#");
        break;

      case ('N'<<8 | 'G'):
        // GN command Get new (target) position
        pos = Target;
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('C'<<8 | 'G'):
        // GC command Get temerature coefficient, always 2
        Serial.print("02#");
        break;

      case ('D'<<8 | 'G'):
        // GD command Get motor speed
        sprintf(tempString, "%02X", speed);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('D'<<8 | 'S'):
        // SD command Set motor speed
        speed = hexstr2long(param);
        if (speed < minSpeed)
          speed = minSpeed;
        if (speed > maxSpeed)
          speed = maxSpeed;
        break;

      case ('H'<<8 | 'G'):
        // GH command Get half step mode, always 00
        Serial.print("00#");
        break;

      case ('P'<<8 | 'S'):
        // SP command Set current position
        pos = hexstr2long(param);
        if (pos == 0)
          pos = HOME;
        Current = pos;
        break;

      case ('N'<<8 | 'S'):
        // SN command Set new position
        pos = hexstr2long(param);
        if (pos == 0)
          pos = HOME;
        Target = pos;
        break;

      case ('G'<<8 | 'F'):
        // FG command Start motor command
        isRunning = 1;
        //digitalWrite(RUNLED,HIGH);
        break;

      case ('Q'<<8 | 'F'):
        // FQ command Stop motor command
        isRunning = 0;
        //digitalWrite(RUNLED,LOW);
        break;

    }
    //digitalWrite(testPin,HIGH);

  } // end process command

} // end forever loop




