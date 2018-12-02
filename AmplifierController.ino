#include <stdlib.h>
#include <SPI.h>

#include "TM1637.h"

#define ENC_PIN_A 2 // digital pin 2
#define ENC_PIN_B 3 // digital pin 3
#define ENC_BTN_PIN 7

#define CLK 4 //pins definitions for TM1637
#define DIO 5
#define CH_PIN 9
#define PW_PIN 8
#define IR_PIN 6
#define TEST_DELAY 2000
#include <IRremote.h>

#define INIT_GAIN 120  //-36dB
#define MAX_VOLUME 212 //+10db
#define MIN_VOLUME 30  //-80db

#define SEG_A 0b00000001
#define SEG_B 0b00000010
#define SEG_C 0b00000100
#define SEG_D 0b00001000
#define SEG_E 0b00010000
#define SEG_F 0b00100000
#define SEG_G 0b01000000

TM1637 tm1637(CLK, DIO);

const int PGA2311SlaveSelectPin = 10;

volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile byte encoderPos = 0;
volatile byte oldEncPos = 0;
volatile byte reading = 0;

int nGain = INIT_GAIN;
int lastIrValue;
int repeatCount = 1;
bool mute = false;
bool powerOn = false;
bool chanelB = false;
bool btnDown = false;

byte displayUpdate;

bool sleep = false;
uint16_t timer3 = 0;
int inc = 1;

IRrecv irrecv(IR_PIN);
decode_results results;

int8_t muteData[] = {SEG_G, SEG_G, SEG_G, SEG_G};
int8_t chBdata[] =
    {
        0x0,
        SEG_G | SEG_E | SEG_D,
        SEG_F | SEG_E | SEG_G | SEG_C,
        SEG_A | SEG_B | SEG_G | SEG_E | SEG_D};

int8_t chAdata[] =
    {
        0x0,
        SEG_G | SEG_E | SEG_D,
        SEG_F | SEG_E | SEG_G | SEG_C,
        SEG_B | SEG_C};

enum RemoteHex
{
  VOL_UP = 0x807F48B7,
  VOL_DOWN = 0x2BD17744,
  CH_UP = 0x55E1CDE4,
  CH_DOWN = 0x8857AE20,
  MUTE = 0x807F609F,
  POWER = 0x8D6CD0C8,
  REPEAT_LAST = 0xFFFFFFFF
};

unsigned long lastAction = millis();
unsigned long btnPressedTime = 0;

void setup()
{
  Serial.begin(115200);
  //Serial.begin(57600);
  while (!Serial)
    ;

  Serial.println("INIT");

  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  pinMode(ENC_PIN_A, INPUT_PULLUP); // set pinA as an input
  pinMode(ENC_PIN_B, INPUT_PULLUP); // set pinB as an input

  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, rising edge signal
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, rising edge signal

  tm1637.init(D4056A);
  tm1637.clearDisplay();

  irrecv.enableIRIn(); // запуск приемника

  pinMode(PGA2311SlaveSelectPin, OUTPUT);
  pinMode(CH_PIN, OUTPUT);
  pinMode(PW_PIN, OUTPUT);

  digitalWrite(PW_PIN, LOW);
  digitalWrite(CH_PIN, LOW);

  SPI.begin();

  SetGain(nGain);
}

void PinA()
{
  cli();                      //stop interrupts
  reading = PIND & B00001100; //read all eight pin values then strip away all but pinA and pinB's values

  //check that we have both pins at detent (HIGH)
  if (reading == B00001100 && aFlag)
  {
    encoderPos++;
    VolumeUp(1);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100)
  {
    bFlag = 1;
  }
  sei(); //restart interrupts
}

void PinB()
{
  cli();
  reading = PIND & B00001100;

  //check that we have both pins at detent (HIGH)
  if (reading == B00001100 && bFlag)
  {
    encoderPos--;
    VolumeDown(1);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000)
    aFlag = 1;
  sei();
}

void UpdateDisplay()
{
  if (!powerOn)
  {
    return;
  }
  float gain = 31.5 - 0.5 * (255 - float(nGain));
  tm1637.display((int)gain);
}

void DisplayChanel()
{
  if (chanelB)
  {
    tm1637.displayRaw(chBdata);
    digitalWrite(CH_PIN, HIGH);
  }
  else
  {
    tm1637.displayRaw(chAdata);
    digitalWrite(CH_PIN, LOW);
  }
}

void Sleep()
{
  if (powerOn)
  {
    if ((millis() - lastAction) > 500)
    {

      inc = 1;
      lastIrValue = 0;
    }

    if (sleep)
    {
      if (mute)
      {
        if ((millis() - lastAction) > 3000)
        {
          timer3 %= 8;

          int8_t data[4] = {};

          switch (timer3)
          {
          case 0:
            data[0] = SEG_F | SEG_E;
            break;
          case 1:
            data[0] = SEG_B | SEG_C;
            break;
          case 2:
            data[1] = SEG_F | SEG_E;
            break;
          case 3:
            data[1] = SEG_B | SEG_C;
            break;
          case 4:
            data[2] = SEG_F | SEG_E;
            break;
          case 5:
            data[2] = SEG_B | SEG_C;
            break;
          case 6:
            data[3] = SEG_F | SEG_E;
            break;
          case 7:
            data[3] = SEG_B | SEG_C;
            break;
          }

          tm1637.displayRaw(data);

          timer3++;

          lastAction = millis();
        }
      }
    }
    else
    {
      if ((millis() - lastAction) > 5000)
      {
        tm1637.set(1); //Decrease brightness
        if (mute)
        {
          tm1637.displayRaw(muteData);
        }
        else
        {
          DisplayChanel();
        }
        sleep = true;
        lastAction = millis();
      }
    }
  }
}

void loop()
{
  if (digitalRead(ENC_BTN_PIN) == LOW)
  {
    if ((millis() - btnPressedTime > 250))
    {
      btnDown = true;
      btnPressedTime = millis();
      lastAction = millis();
      timer3 = 0;
      sleep = false;
      if (powerOn)
      {
        SwitchChanel();
      }
      else
      {
        SwitchPower();
      }
    }
  }

  if (oldEncPos != encoderPos)
  {
    Serial.println(encoderPos);
    oldEncPos = encoderPos;
    lastAction = millis();
    timer3 = 0;
    sleep = false;
  }

  if (ReadIR())
  {
    lastAction = millis();
    timer3 = 0;
    sleep = false;
  }

  Sleep();
}

bool ReadIR()
{
  bool result = false;
  if (irrecv.decode(&results))
  {
    result = Decode(results.value);
    Serial.println(results.value, HEX);
    irrecv.resume();
  }
  return result;
}

bool Decode(int value)
{
  bool result = false;
  switch (value)
  {
  case VOL_UP:
    VolumeUp(inc);
    lastIrValue = value;
    result = true;
    break;

  case VOL_DOWN:
    VolumeDown(inc);
    lastIrValue = value;
    result = true;
    break;

  case CH_UP:
    SwitchChanel();
    lastIrValue = 0;
    result = true;
    break;

  case CH_DOWN:
    SwitchChanel();
    lastIrValue = 0;
    result = true;
    break;

  case POWER:
    SwitchPower();
    lastIrValue = 0;
    result = true;
    break;

  case MUTE:
    Mute();
    lastIrValue = 0;
    result = true;
    break;

  case REPEAT_LAST:
    inc = 2;
    Decode(lastIrValue);
    result = true;
    break;
  }
  return result;
}

void VolumeUp(int inc)
{
  if (!powerOn)
  {
    return;
  }

  Serial.println("+");

  mute = false;
  if (nGain < MAX_VOLUME)
  {
    nGain = nGain + inc;
    nGain = nGain > MAX_VOLUME ? MAX_VOLUME : nGain;
    tm1637.set(BRIGHTEST);
    SetGain(nGain);
  }
}
void VolumeDown(int inc)
{
  if (!powerOn)
  {
    return;
  }

  Serial.println("-");

  mute = false;
  if (nGain > MIN_VOLUME)
  {
    nGain = nGain - inc;
    nGain = nGain < MIN_VOLUME ? MIN_VOLUME : nGain;
    tm1637.set(BRIGHTEST);
    SetGain(nGain);
  }
}

void Mute()
{
  if (!powerOn)
  {
    return;
  }
  mute = !mute;
  if (mute)
  {
    tm1637.set(BRIGHTEST);
    SetGain(0);
    tm1637.displayRaw(muteData);
  }
  else
  {
    SetGain(nGain);
  }
}

void SwitchPower()
{
  Serial.println("POWER");
  powerOn = !powerOn;
  if (powerOn)
  {
    tm1637.set(BRIGHTEST);
    SetGain(nGain);
    digitalWrite(PW_PIN, HIGH);
  }
  else
  {
    tm1637.set(BRIGHT_DARKEST); //BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    tm1637.clearDisplay();
    digitalWrite(PW_PIN, LOW);
  }
}

void SetGain(int gain)
{
  digitalWrite(PGA2311SlaveSelectPin, LOW);
  SPI.transfer(gain); // right channel
  SPI.transfer(gain); // left channel
  digitalWrite(PGA2311SlaveSelectPin, HIGH);

  UpdateDisplay();
}

void SwitchChanel()
{
  if (!powerOn)
  {
    return;
  }
  Serial.println("CHANEL");
  chanelB = !chanelB;
  tm1637.set(BRIGHTEST);
  DisplayChanel();
}
