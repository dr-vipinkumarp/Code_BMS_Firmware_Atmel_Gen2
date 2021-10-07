#include <Arduino.h>
#include <Wire.h>          // for TI comms
#include <avr/boot.h>      // for accessing unique serial number identifier of the ATMega328PB
#include <avr/sleep.h>     // for power saving
#include <avr/interrupt.h> // for power saving

#include <defines.h>
#include <pack_state.h>
#include <packet.h>

#include <ArduinoJson.h>

// #define DEBUG_OUTPUT
#define LIVE_DATA

#ifdef DEBUG_OUTPUT
#include <debug.h>
#endif

static const uint32_t BAUD_RATE = 115200;
static const uint8_t BYTES_PER_MILLIS = BAUD_RATE / 1000 / 10;                              // assuming 10 bits per byte for serial framing overhead
static const uint8_t DISCOVERY_MSG_LENGTH = 33;                                             // // {"PackID":"5042394E3530690E1023"}
static const uint8_t DISCOVERY_MSG_SEND_TIME = DISCOVERY_MSG_LENGTH / BYTES_PER_MILLIS * 2; // double the window size to ensure no overlaps
static const uint16_t DISCOVERY_WINDOW_MAX_TIME = 2500;                                     // discovery window is 2.5 seconds long
static const uint16_t DISCOVERY_WINDOW_MAX_SLOT = DISCOVERY_WINDOW_MAX_TIME / DISCOVERY_MSG_SEND_TIME;

TI ti(TI_Addr);
PackState packState(ti);
Packet packet;

byte AVR_Signature[10];
uint32_t sample_timer = 100000; // init with a large value to immediately trigger a pack data read event on boot

char incomingBuffer[50];
int bufferIndex = 0;
bool NewPacket = false;
int Output = 1;

// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt = 1;

//================================================================
// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect)
{
  if (f_wdt == 0)
  {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    f_wdt = 1;
  }
}

//================================================================
// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer()
{
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1 << WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /**
	 *	Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
	 *	WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
	 *	0    0    0    0    |   2K cycles   | 16 ms
	 *	0    0    0    1    |   4K cycles   | 32 ms
	 *	0    0    1    0    |   8K cycles   | 64 ms
	 *	0    0    1    1    |  16K cycles   | 0.125 s
	 *	0    1    0    0    |  32K cycles   | 0.25 s
	 *	0    1    0    1    |  64K cycles   | 0.5 s
	 *	0    1    1    0    |  128K cycles  | 1.0 s
	 *	0    1    1    1    |  256K cycles  | 2.0 s
	 *	1    0    0    0    |  512K cycles  | 4.0 s
	 *	1    0    0    1    | 1024K cycles  | 8.0 s
	*/
  WDTCSR = (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}

//================================================================
void enterSleep()
{
  // sleep to save power
  set_sleep_mode(SLEEP_MODE_IDLE);
  cli();
  sleep_enable();
  sei();
  sleep_cpu();
  // The program will continue from here after the WDT timeout
  sleep_disable();
  power_all_enable();
  sei();
}

//================================================================
void initRNG()
{
  // initiate random number generator
  // get the last 4 bits from the AVR_Signature and convert them to an int
  // xor that int with a random value from the seed
  int serial_lsb = *((int *)&AVR_Signature[10 - 4 - 1]); // 10-4-1 == index to last 4 bits
  int rand_seed = analogRead(A0) ^ serial_lsb;
  randomSeed(rand_seed);
}

void getAvrSignature()
{
#define SIGRD 5
#if defined(SIGRD) || defined(RSIG)
  // Versions.AVR_Dev_ID[0] = boot_signature_byte_get(0);
  // Versions.AVR_Dev_ID[1] = boot_signature_byte_get(2);
  // Versions.AVR_Dev_ID[2] = boot_signature_byte_get(4);
  for (uint8_t i = 14; i < 24; i += 1)
  {
    AVR_Signature[i - 14] = boot_signature_byte_get(i); // Read my own unique ID
  }
#endif
}

//================================================================
void setup()
{
  getAvrSignature();

  packet.setAvrSignature(AVR_Signature);

  Serial.begin(BAUD_RATE);

  initRNG();

  Wire.begin(); // connection to TI BMS Chipset
  //setupWatchDogTimer();   // for waking up from sleep

  // if (PullTIVersionData()) {
  //   //good
  // }
  // else {
  //   // flag fault with BMS
  // }
}

bool isAddressedToMe(char *incomingAddress)
{
  const char *myAddr = packet.packId();

  // if packet addressed to me
  bool addressedToMe = true;

  for (int i = 0; i < PACK_ID_SIZE - 1; i++)
  {
    if (incomingAddress[i] != myAddr[i])
    {
      addressedToMe = false;
    }
  }

  return addressedToMe;
}

void handleIdentify()
{
  // pick a random slot to send our msg in
  initRNG(); // reinit RNG, as if we've collided, it might be because we have the same random seed as another pack
  uint32_t chosen_slot = random(0, DISCOVERY_WINDOW_MAX_SLOT);
  uint32_t rand_delay = DISCOVERY_MSG_SEND_TIME * chosen_slot;
  delay(rand_delay);

  // send just pack ID response
  serializeJson(packet.identityPacket(), Serial); // TODO: change this from Serial to a buffer out_str
}

void handleDataRequest()
{
  // send a full data packet
  serializeJson(packet.dataPacket(packState), Serial); // TODO: change this from Serial to a buffer out_str
}

//================================================================
void loop()
{

  //Wait until the watchdog have triggered a wake up.
  // if(f_wdt != 1) {
  // 	return;
  // }

  //Serial.print("A");

  //if sample timer elapsed
  if (sample_timer >= sample_rate)
  {
    sample_timer = 0;

#ifdef LIVE_DATA
    packState.refresh();
#endif

#ifdef DEBUG_OUTPUT
    printDebug(packState, ti);
#endif
  }
  sample_timer++;

  // check bus for data requests
  while (Serial.available())
  { // read bus

    char incoming = Serial.read();
    //{"PackID":"5042394E3530690E1023"}
    //{"PackID":"3430524C344769182529","Output":0}
    //{"PackID":"5042394E3530690E1023","Output":0}         // example messages for copy-paste testing
    //{\"PackID\":\"5042394E3530690E1025\"}

    if (incoming == '{')
    { // then this is a new packet
      NewPacket = true;
      bufferIndex = 0;
      incomingBuffer[bufferIndex] = incoming;
    }
    else if (NewPacket && (incoming != 92))
    { // if we're reading in a new packet and the incoming char isn't an escape character
      bufferIndex++;
      incomingBuffer[bufferIndex] = incoming;
      if ((bufferIndex >= 43) or (incoming == '}'))
      { // if we've read in enough characters to have received the PackID field, ignore the rest (this will need to be changed to add external ON/OFF commands and so on)
        incomingBuffer[bufferIndex++] = '}';
        NewPacket = false;
        bufferIndex = 0;

        //Serial.println(incomingBuffer);
        //Serial.flush();
        //while (Serial.available()) { Serial.read(); }

        // parse incoming data
        StaticJsonDocument<20> incoming_packet;
        DeserializationError error = deserializeJson(incoming_packet, incomingBuffer);
        // if (error) {  // spew error report onto the bus (could make comms messy)
        //   Serial.print(F("deserializeJson() failed: "));
        //   Serial.println(error.f_str());
        //   return;
        // }

        if (error)
          continue; // proceed with next iteration of loop

        char *PackAddr = incoming_packet["PackID"];
        bool addressedToMe = isAddressedToMe(PackAddr);

        if (PackAddr[0] == '?')
        {
          addressedToMe = true; // to re-discover a silent pack
          handleIdentify();
        }
        else if (addressedToMe)
        {
          handleDataRequest();
        }

        if (!addressedToMe)
          continue;

        Serial.flush();
        delay(10);
        while (Serial.available())
        {
          Serial.read();
        }

        // and finally, send FET enable command to TI (with Pre- CHG/DCHG configured for safety)
        Output = incoming_packet["Output"];
        if (Output)
        {
          //Serial.println("Output on");
          ti.fetOn();
        }
        else if (!Output)
        {
          //Serial.println("Output off");
          ti.fetOff();
        }

        //{"PackID":"5042394E3530690E1023","Output":0}
        //{"PackID":"5042394E3530690E1023","Output":1}
        //{"PackID":"?"}
      }
    }
  }

  // // clear the flag so we can run above code again after the MCU wake up
  // f_wdt = 0;

  // // Re-enter sleep mode.
  // enterSleep();
}
