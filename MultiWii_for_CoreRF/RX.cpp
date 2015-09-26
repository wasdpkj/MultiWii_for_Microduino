#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
#if defined(SERIAL_SUM_PPM)
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502
}; // interval [1000;2000]
#else
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502
}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
static uint8_t rcChannel[RC_CHANS] = {
  SERIAL_SUM_PPM
};
#else // Standard Channel order
static uint8_t rcChannel[RC_CHANS]  = {
  ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, AUX3PIN, AUX4PIN
};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {
  PCINT_RX_BITS
}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

void rxInt(void);

/**************************************************************************************/
/***************                   Core RF Setup                   ********************/
/**************************************************************************************/
byte getChecksum(byte length, byte cmd, byte mydata[])
{
  //三个参数分别为： 数据长度  ，  指令代码  ，  实际数据数组
  byte checksum = 0;
  checksum ^= (length & 0xFF);
  checksum ^= (cmd & 0xFF);
  for (int i = 0; i < length; i++)
  {
    checksum ^= (mydata[i] & 0xFF);
  }
  return checksum;
}

int p;
int16_t read16(byte* _Buf) {
  byte _Buf1 = _Buf[p++];
  byte _Buf2 = _Buf[p++];
  int16_t r = (_Buf1 & 0xff) | ((_Buf2 << 8) & 0xff00);
  return r;
}

void read_data(int _num, byte* _buf) {
  p = 0;
  _num /= 2;
  int16_t _bufin[_num];
  for (int i = 0; i < _num; i++)  _bufin[i] = read16(_buf);
  for (int a = 0; a < _num; a++)  rcValue[a] = _bufin[a];
}

byte inChar, inCache;
byte buffer[128];
unsigned long num = 0;

boolean sta = false;
boolean error = false;

/*
[head,2byte,0xAA 0xBB] [type,1byte,0xCC] [data,16byte] [body,1byte(from getChecksum())]
 Example:
 AA BB CC 1A 01 1A 01 1A 01 2A 01 3A 01 4A 01 5A 01 6A 01 0D **
 */
void RF_data()
{
  while (RF.available()) {
    inCache = inChar;
    inChar = RF.read();
    //delayMicroseconds(200);
    buffer[num++] = inChar;

    //step.2
    if (sta) {
      sta = false;
      error = boolean(inChar != 0xCC);
      num = 0;
    }

    //step.1
    if (inChar == 0xBB && inCache == 0xAA)  sta = true;

    //step.3
    //get_data
    if (num  == (16 + 1)) {
      inCache = buffer[16];
      buffer[16] = NULL;
      inChar = getChecksum(16, 200, buffer);
      if (!error && inCache == inChar)
      {
        read_data(16, buffer);
        //clear FailSafe counter  - added by MIS
        if (failsafeCnt > 20) failsafeCnt -= 20;
        else failsafeCnt = 0;
        return;
      }
      num = 0;
    }
  }
}

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  // Init PPM SUM RX
#if defined(SERIAL_SUM_PPM)
  PPM_PIN_INTERRUPT;
#endif
  // Init Sektrum Satellite RX
}



/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// PPM_SUM at THROTTLE PIN on MEGA boards
#if defined(PPM_ON_THROTTLE) && defined(MEGA) && defined(SERIAL_SUM_PPM)
ISR(PCINT2_vect) {
  rxInt();
}
#endif

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)
void rxInt(void) {
  uint16_t now, diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;
#if defined(FAILSAFE)
  static uint8_t GoodPulses;
#endif

  now = micros();
  sei();
  diff = now - last;
  last = now;
  if (diff > 3000) chan = 0;
  else {
    if (900 < diff && diff < 2200 && chan < RC_CHANS ) { //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
      rcValue[chan] = diff;
#if defined(FAILSAFE)
      if (chan < 4 && diff > FAILSAFE_DETECT_TRESHOLD) GoodPulses |= (1 << chan); // if signal is valid - mark channel as OK
      if (GoodPulses == 0x0F) {                                            // If first four chanells have good pulses, clear FailSafe counter
        GoodPulses = 0;
        if (failsafeCnt > 20) failsafeCnt -= 20;
        else failsafeCnt = 0;
      }
#endif
    }
    chan++;
  }
}
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG;        // Let's restore interrupt state

  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan, a;

  RF_data();  //pkj

  rc4ValuesIndex++;
  if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;
  for (chan = 0; chan < RC_CHANS; chan++) {
#if defined(FAILSAFE)
    uint16_t rcval = readRawRC(chan);
    if (rcval > FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED) {     // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
      rcData4Values[chan][rc4ValuesIndex] = rcval;                      // In disarmed state allow always update for easer configuration.
    }
#else
    rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
#endif
    rcDataMean[chan] = 0;
    for (a = 0; a < 4; a++) rcDataMean[chan] += rcData4Values[chan][a];
    rcDataMean[chan] = (rcDataMean[chan] + 2) >> 2;
    if ( rcDataMean[chan] < (uint16_t)rcData[chan] - 3)  rcData[chan] = rcDataMean[chan] + 2;
    if ( rcDataMean[chan] > (uint16_t)rcData[chan] + 3)  rcData[chan] = rcDataMean[chan] - 2;
    if (chan < 8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
      rcSerialCount --;
#if defined(FAILSAFE)
      failsafeCnt = 0;
#endif
      if (rcSerial[chan] > 900) {
        rcData[chan] = rcSerial[chan];
      } // only relevant channels are overridden
    }
  }
}
