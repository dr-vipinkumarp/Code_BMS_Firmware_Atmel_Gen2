#include <Arduino.h>
#include <Wire.h>       // for TI comms
#include <avr/boot.h>   // for accessing unique serial number identifier of the ATMega328PB
#include <avr/sleep.h>  // for power saving
#include <avr/interrupt.h>  // for power saving

#include <defines.h>

#include <ArduinoJson.h>

//#include <MqttSerial.h>
//#include <limero.h>

// #include <pack2master.pb.h>
// #include <pb.h>
// #include <pb_encode.h>
// #include <pb_decode.h>

//#define DEBUG_OUTPUT
#define LIVE_DATA
#define OUTPUT_DATA

struct Version_Info {

  word TI_DevType;
  //byte FirmwareVersionsBuffer[10];
  word TI_DevNum;
  word TI_FirmwareVersion;
  word TI_ChemistryID;
  //byte TI_FirmwareType[2];
  //byte TI_CEDV[2];
  //byte TI_HW_Ver[2];
  //byte AVR_Dev_ID[3];
  //byte AVR_FirmwareVersion[2];
  byte AVR_Signature[10];
  
};
  
struct Pack_State {

  // safety variables - priority items
  word SafetyAlert_AB;
  word SafetyAlert_CD;
  word SafetyStatus_AB;
  word SafetyStatus_CD;
  word OpStatus_A;
  word OpStatus_B;
  word PFStatus_AB;
  word PFStatus_CD;
  word PFAlert;
  word BattMode;
  word BattStatus;
  byte TempStatus;
  byte ChgStatus;
  word GaugeStatus;
  word MfrStatus;
  word AFEStatus;

  // control variables - influence hardware operations
  word HostFET;
  word GPIOStatus;
  word GPIOControl;
  
  // data variables - for reporting
  //uint16_t Temperature;
  uint16_t Voltage;
  uint16_t VAuxVoltage;
  uint16_t ExtAvgVoltage;
  int16_t Current;         // signed int Changed by Viv to signed
  int16_t AvgCurrent;      // signed int Changed by Viv to signed
  uint16_t CycleCount;
  uint16_t CapacityRemaining;
  uint16_t CapacityFull;
  uint16_t RuntimeToEmpty;
  uint16_t AvgTimeToEmpty;
  uint16_t AvgTimeToFull;

  uint8_t RelativeStateOfCharge;
  uint8_t AbsoluteStateOfCharge;
  uint8_t StateOfHealth;

  uint16_t CellVoltage1;
  uint16_t CellVoltage2;
  uint16_t CellVoltage3;
  uint16_t CellVoltage4;
  uint16_t CellVoltage5;
  uint16_t CellVoltage6;
  uint16_t CellVoltage7;
  uint16_t CellVoltage8;
  uint16_t CellVoltage9;
  uint16_t CellVoltage10;
  uint16_t CellVoltage11;
  uint16_t CellVoltage12;
  uint16_t CellVoltage13;
  uint16_t CellVoltage14;
  uint16_t CellVoltage15;

  uint16_t TS1Temp;
  uint16_t TS2Temp;
  uint16_t TS3Temp;
  uint16_t CellTemp;
  uint16_t FETTemp;
  uint16_t InternalTemp;
  
};

struct Version_Info Versions;
struct Pack_State Pack;

bool master_found = false;
char PackID[(10 * 2) + 1];

uint32_t sample_timer = 100000;   // init with a large value to immediately trigger a pack data read event on boot

StaticJsonDocument<480> packet;
char incomingBuffer[50];
int bufferIndex = 0;
bool NewPacket = false;
int CAN_collisions_detected = 0;
int Output = 1;

// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt=1;



//================================================================
// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {
	if(f_wdt == 0) {
		// here we can implement a counter the can set the f_wdt to true if
		// the watchdog cycle needs to run longer than the maximum of eight
		// seconds.
		f_wdt=1;
	}
}


//================================================================
bool TI_Read(byte CMD, int LEN, byte IN_DATA[]){ // FIX -- Pass pointers for data variable

  Wire.beginTransmission(TI_Addr);
  Wire.write(CMD);
  
  // FIX -- if LEN not zero then send follow-up commands to prompt TI to spew data, else command was a one-shot with a predictable response
  Wire.endTransmission(false);
  Wire.requestFrom(TI_Addr, LEN, true);  // first byte through will usually be a "LENGTH" value
    
    for(int i = 0; i < LEN; i++){        // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
      IN_DATA[i] = Wire.read();          // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
  // else
  // Wire.endTransmission(true);
  // FIX -- read whatever was predictable about a CMD with a zero length reply
  
  return 1;  
}


void FET_ON() {
  byte do_it[] = { 0x2B, 0x97, 0x11 };
  byte turn_on[] = { 0x2B, 0x03, 0x00 };
  Wire.beginTransmission(TI_Addr);
  Wire.write(do_it, 3);
  Wire.endTransmission(true);
  Wire.beginTransmission(TI_Addr);
  Wire.write(turn_on, 3);
  Wire.endTransmission(true);
}

void FET_OFF() {
  byte do_it[] = { 0x2B, 0x97, 0x11 };
  byte turn_off[] = { 0x2B, 0x01, 0x00 };
  Wire.beginTransmission(TI_Addr);
  Wire.write(do_it, 3);
  Wire.endTransmission(true);
  Wire.beginTransmission(TI_Addr);
  Wire.write(turn_off, 3);
  Wire.endTransmission(true);
}


//================================================================
bool TI_Write(byte CMD, int LEN, byte OUT_DATA[]){ // FIX -- Pass pointers for data variable

  Wire.beginTransmission(TI_Addr);
  Wire.write(CMD);

  // FIX -- if LEN not zero then send follow-up commands to prompt TI to spew data, else command was a one-shot with a predictable response
  Wire.endTransmission(false);
  //Wire.requestFrom(TI_Addr, LEN, true);
  //Wire.write(LEN);  // not sure if this is needed, have to eavesdrop comms to see if this helps or breaks

  // do these need to be reversed?
    for(int i = 0; i < LEN; i++){        // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
      Wire.write(OUT_DATA[i]);          // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
  // else
  Wire.endTransmission(false);
  // FIX -- read whatever was predictable about a CMD with a zero length reply
  
  return 1;  
}


//================================================================
bool TI_BlockRead(byte CMD[], int LEN, byte IN_DATA[]) {

  Wire.beginTransmission(TI_Addr);
  Wire.write(0x00);   // manufacturer access
  Wire.write(0x02);
  Wire.write(0x00); // because reversed
  Wire.write(0x44);   // read block
  Wire.endTransmission(false);

  //LEN = Wire.read();  // first byte probably length (try it)
  Wire.requestFrom(TI_Addr, LEN, true);

    for(int i = 0; i < LEN; i++){        // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
      IN_DATA[i] = Wire.read();          // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
  //Wire.endTransmission(true);
}


//================================================================
bool PullTIVersionData() {

byte InBuf[15];

  // so far no luck with this

  if (TI_Read(CMD_DeviceType, LEN_IN_DeviceType, InBuf)) { Versions.TI_DevType = (InBuf[1]<<8) + InBuf[2];
  if (TI_BlockRead(CMD_FirmwareVersion, LEN_IN_FirmwareVersion, InBuf)) {
    Versions.TI_DevNum = (InBuf[2]<<8) + InBuf[1];
    Versions.TI_FirmwareVersion = (InBuf[4]<<8) + InBuf[3];
  if (TI_Read(CMD_CHEM_ID, LEN_CHEM_ID, InBuf)) { Versions.TI_ChemistryID = (InBuf[2]<<8) + InBuf[1];
  
    return 1;
  }}}

  return 0;

}


//================================================================
bool PullData(){//struct Incoming_Flags FLAGS, struct Incoming_Data DATA){

  byte InBuff[35];

  // Flags  
  if (TI_Read(CMD_SafetyAlert, LEN_IN_SafetyAlert, InBuff)) { 
    Pack.SafetyAlert_AB = (InBuff[2]<<8) + InBuff[1]; // Safety Alert
    Pack.SafetyAlert_CD = (InBuff[4]<<8) + InBuff[3];
  if (TI_Read(CMD_SafetyStatus, LEN_IN_SafetyStatus, InBuff)) { 
    Pack.SafetyStatus_AB = (InBuff[2]<<8) + InBuff[1]; // Safety Status
    Pack.SafetyStatus_CD = (InBuff[4]<<8) + InBuff[3];
  if (TI_Read(CMD_OpStatus, LEN_IN_OpStatus, InBuff)) { 
    Pack.OpStatus_A = (InBuff[2]<<8) + InBuff[1]; // Operational Status
    Pack.OpStatus_B = (InBuff[4]<<8) + InBuff[3];
  if (TI_Read(CMD_PFStatus, LEN_IN_OpStatus, InBuff)) { 
    Pack.PFStatus_AB = (InBuff[2]<<8) + InBuff[1]; // Permanent Fail Status
    Pack.PFStatus_CD = (InBuff[4]<<8) + InBuff[3];
  if (TI_Read(CMD_BattMode, LEN_IN_BattMode, InBuff)) { Pack.BattMode = (InBuff[1]<<8) + InBuff[0]; // Battery Mode  
  if (TI_Read(CMD_BattStatus, LEN_IN_BattStatus, InBuff)) { Pack.BattStatus = (InBuff[1]<<8) + InBuff[0]; // Battery Status
  if (TI_Read(CMD_ChgStatus, LEN_IN_ChgStatus, InBuff)) { 
    Pack.TempStatus = InBuff[1]; // Temp Status
    Pack.ChgStatus = InBuff[2]; // Charge Status           // WARNING -- This 2x 8bit register pair is reversed
  if (TI_Read(CMD_GaugeStatus, LEN_IN_GaugeStatus, InBuff)) { Pack.GaugeStatus = (InBuff[2]<<8) + InBuff[1]; // Gauge Status
  if (TI_Read(CMD_MfrStatus, LEN_IN_MfrStatus, InBuff)) { Pack.MfrStatus = (InBuff[2]<<8) + InBuff[1]; // Manufacturer Status
  if (TI_Read(CMD_AFEStatus, LEN_IN_AFEStatus, InBuff)) { Pack.AFEStatus = (InBuff[2]<<8) + InBuff[1]; // Analog Front End Status

  // GPIO
  if (TI_Read(CMD_HostFET, LEN_IN_HostFET, InBuff)) { Pack.HostFET = (InBuff[1]<<8) + InBuff[0]; // Host FET control
  if (TI_Read(CMD_GPIOStatus, LEN_IN_GPIOStatus, InBuff)) { Pack.GPIOStatus = (InBuff[1]<<8) + InBuff[0]; // GPIO Status
  if (TI_Read(CMD_GPIOControl, LEN_IN_GPIOControl, InBuff)) { Pack.GPIOControl = (InBuff[1]<<8) + InBuff[0]; // GPIO Control

  // Pack Readings
  if (TI_Read(CMD_Voltage, LEN_IN_Voltage, InBuff)) { Pack.Voltage = (InBuff[1]<<8) + InBuff[0]; // Voltage
  if (TI_Read(CMD_Current, LEN_IN_Current, InBuff)) { Pack.Current = (InBuff[1]<<8) + InBuff[0]; // Current
  if (TI_Read(CMD_AvgCurrent, LEN_IN_AvgCurrent, InBuff)) { Pack.AvgCurrent = (InBuff[1]<<8) + InBuff[0]; // AvgCurrent
  if (TI_Read(CMD_CapRemain, LEN_IN_CapRemain, InBuff)) { Pack.CapacityRemaining = (InBuff[1]<<8) + InBuff[0]; // Remaining Capacity
  if (TI_Read(CMD_CapFull, LEN_IN_CapFull, InBuff)) { Pack.CapacityFull = (InBuff[1]<<8) + InBuff[0]; // Full Capacity
  if (TI_Read(CMD_RunTTE, LEN_IN_RunTTE, InBuff)) { Pack.RuntimeToEmpty = (InBuff[1]<<8) + InBuff[0]; // Run Time to Empty
  if (TI_Read(CMD_AvgTTE, LEN_IN_AvgTTE, InBuff)) { Pack.AvgTimeToEmpty = (InBuff[1]<<8) + InBuff[0]; // Avg Time to Empty
  if (TI_Read(CMD_AvgTTF, LEN_IN_AvgTTF, InBuff)) { Pack.AvgTimeToFull = (InBuff[1]<<8) + InBuff[0]; // Avg Time to Empty
  if (TI_Read(CMD_CycleCount, LEN_IN_CycleCount, InBuff)) { Pack.CycleCount = (InBuff[1]<<8) + InBuff[0]; // Cycle count
  if (TI_Read(CMD_RelSoC, LEN_IN_RelSoC, InBuff)) { Pack.RelativeStateOfCharge = InBuff[1]; //<<8) + Data.RelSoC[0]; // Relative State of Charge
  if (TI_Read(CMD_AbsSoC, LEN_IN_AbsSoC, InBuff)) { Pack.AbsoluteStateOfCharge = InBuff[1]; //<<8) + Data.AbsSoC[0]; // Absolute State of Charge
  if (TI_Read(CMD_SoH, LEN_IN_SoH, InBuff)) { Pack.StateOfHealth = InBuff[1]; //<<8) + Data.SoH[0]; // State of Health
  //if (TI_Read(CMD_Temperature, LEN_IN_Temperature, Temperature)) { // Temperature -- This is an averaged value anyway, DAStatus2 contains full detail
  //if (TI_Read(CMD_VAuxVoltage, LEN_IN_VAuxVoltage, VAuxVoltage)) { // VAuxVoltage -- DAStatus2 contains
  //if (TI_Read(CMD_ExtAvgVoltage, LEN_IN_ExtAvgVoltage, ExtAvgVoltage)) { // ExtAvgVoltage -- DAStatus2 contains

  if (TI_Read(CMD_DAStatus1, LEN_IN_DAStatus1, InBuff)) { // Cell Voltages Data Block

    Pack.CellVoltage1 = (InBuff[2]<<8) + InBuff[1];
    Pack.CellVoltage2 = (InBuff[4]<<8) + InBuff[3];
    Pack.CellVoltage3 = (InBuff[6]<<8) + InBuff[5];
    Pack.CellVoltage4 = (InBuff[8]<<8) + InBuff[7];
    Pack.CellVoltage5 = (InBuff[10]<<8) + InBuff[9];
    Pack.CellVoltage6 = (InBuff[12]<<8) + InBuff[11];
    Pack.CellVoltage7 = (InBuff[14]<<8) + InBuff[13];
    Pack.CellVoltage8 = (InBuff[16]<<8) + InBuff[15];
    Pack.CellVoltage9 = (InBuff[18]<<8) + InBuff[17];
    Pack.CellVoltage10 = (InBuff[20]<<8) + InBuff[19];
    Pack.CellVoltage11 = (InBuff[22]<<8) + InBuff[21];
    Pack.CellVoltage12 = (InBuff[24]<<8) + InBuff[23];
    Pack.CellVoltage13 = (InBuff[26]<<8) + InBuff[25];
    Pack.CellVoltage14 = (InBuff[28]<<8) + InBuff[27];
    Pack.CellVoltage15 = (InBuff[30]<<8) + InBuff[29];

  if (TI_Read(CMD_DAStatus2, LEN_IN_DAStatus2, InBuff)) { // Temperatures Data Block

    Pack.ExtAvgVoltage = (InBuff[2]<<8) + InBuff[1];
    Pack.VAuxVoltage = (InBuff[4]<<8) + InBuff[3];
    Pack.TS1Temp = (InBuff[6]<<8) + InBuff[5];
    Pack.TS2Temp = (InBuff[8]<<8) + InBuff[7];
    Pack.TS3Temp = (InBuff[10]<<8) + InBuff[9];
    Pack.CellTemp = (InBuff[12]<<8) + InBuff[11];
    Pack.FETTemp = (InBuff[14]<<8) + InBuff[13];
    Pack.InternalTemp = (InBuff[16]<<8) + InBuff[15];

        return 1;
  }}}}}}}}}}}}}}}}}}}}}}}}}}}//}}}
  return 0;
}


//================================================================
// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
	// The MCU Status Register (MCUSR) is used to tell the cause of the last
	// reset, such as brown-out reset, watchdog reset, etc.
	// NOTE: for security reasons, there is a timed sequence for clearing the
	// WDE and changing the time-out configuration. If you don't use this
	// sequence properly, you'll get unexpected results.

	// Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
	MCUSR &= ~(1<<WDRF);

	// Configure the Watchdog timer Control Register (WDTCSR)
	// The WDTCSR is used for configuring the time-out, mode of operation, etc

	// In order to change WDE or the pre-scaler, we need to set WDCE (This will
	// allow updates for 4 clock cycles).

	// Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
	// bit must be set in order to change WDE or the watchdog pre-scalers.
	// Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
	// clock cycles then it will be reset by hardware.
	WDTCSR |= (1<<WDCE) | (1<<WDE);

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
	WDTCSR  = (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
	// Enable the WD interrupt (note: no reset).
	WDTCSR |= _BV(WDIE);
}


//================================================================
void enterSleep() {
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
void setup() {

#define SIGRD 5
#if defined(SIGRD) || defined(RSIG)
    // Versions.AVR_Dev_ID[0] = boot_signature_byte_get(0);
    // Versions.AVR_Dev_ID[1] = boot_signature_byte_get(2);
    // Versions.AVR_Dev_ID[2] = boot_signature_byte_get(4);
    for (uint8_t i = 14; i < 24; i += 1) {
        Versions.AVR_Signature[i-14] = boot_signature_byte_get(i);  // Read my own unique ID
    }
#endif

  char *ptr = &PackID[0];

  for (int i = 0; i < 10; i++) {
      /* "sprintf" converts each byte in the "buf" array into a 2 hex string
      * characters appended with a null byte, for example 10 => "0A\0".
      *
      * This string would then be added to the output array starting from the
      * position pointed at by "ptr". For example if "ptr" is pointing at the 0
      * index then "0A\0" would be written as output[0] = '0', output[1] = 'A' and
      * output[2] = '\0'.
      *
      * "sprintf" returns the number of chars in its output excluding the null
      * byte, in our case this would be 2. So we move the "ptr" location two
      * steps ahead so that the next hex string would be written at the new
      * location, overriding the null byte from the previous hex string.
      *
      * We don't need to add a terminating null byte because it's been already 
      * added for us from the last hex string. */  
      ptr += sprintf(ptr, "%02X", Versions.AVR_Signature[i]);
  }
  packet["PackID"] = PackID;
  // {"PackID","5042394E3530690E1023"}

  Serial.begin(115200);
  
  #ifdef DEBUG_OUTPUT
    Serial.print("PackID: ");
    Serial.println(PackID);
  #endif

  // initiate random number generator
  int rand_seed = analogRead(A0);
  srand(rand_seed);

  Wire.begin();     // connection to TI BMS Chipset
  //setupWatchDogTimer();   // for waking up from sleep

  // if (PullTIVersionData()) {
  //   //good
  // }
  // else {
  //   // flag fault with BMS
  // }
}


//================================================================
void loop() {
  
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
    if (PullData()) {
      
      packet["SA_AB"] = Pack.SafetyAlert_AB;
      packet["SA_CD"] = Pack.SafetyAlert_CD;
      packet["SS_AB"] = Pack.SafetyStatus_AB;
      packet["SS_CD"] = Pack.SafetyStatus_CD;
      packet["OS_A"] = Pack.OpStatus_A;
      packet["OS_B"] = Pack.OpStatus_B;
      packet["PFS_AB"] = Pack.PFStatus_AB;
      packet["PFS_CD"] = Pack.PFStatus_CD;
      packet["PFA"] = Pack.PFAlert;
      packet["BM"] = Pack.BattMode;
      packet["BS"] = Pack.BattStatus;
      packet["TS"] = Pack.TempStatus;
      packet["CS"] = Pack.ChgStatus;
      packet["GS"] = Pack.GaugeStatus;
      packet["MS"] = Pack.MfrStatus;
      packet["AFES"] = Pack.AFEStatus;

      // control variables - influence hardware operations
      packet["HFET"] = Pack.HostFET;
      packet["GPIOS"] = Pack.GPIOStatus;
      packet["GPIOC"] = Pack.GPIOControl;
      
      // data variables - for reporting
      //uint16_t Temperature;
      packet["PV"] = Pack.Voltage;
      packet["VAV"] = Pack.VAuxVoltage;
      packet["EAvgV"] = Pack.ExtAvgVoltage;
      packet["C"] = Pack.Current;
      packet["AvgC"] = Pack.AvgCurrent;
      packet["Cycles"] = Pack.CycleCount;
      packet["CapRemain"] = Pack.CapacityRemaining;
      packet["CapFull"] = Pack.CapacityFull;
      packet["RTE"] = Pack.RuntimeToEmpty;
      packet["ATTE"] = Pack.AvgTimeToEmpty;
      packet["ATTF"] = Pack.AvgTimeToFull;

      packet["RelSoC"] = Pack.RelativeStateOfCharge;
      packet["AbsSoC"] = Pack.AbsoluteStateOfCharge;
      packet["SoH"] = Pack.StateOfHealth;

      packet["CV1"] = Pack.CellVoltage1;
      packet["CV2"] = Pack.CellVoltage2;
      packet["CV3"] = Pack.CellVoltage3;
      packet["CV4"] = Pack.CellVoltage4;
      packet["CV5"] = Pack.CellVoltage5;
      packet["CV6"] = Pack.CellVoltage6;
      packet["CV7"] = Pack.CellVoltage7;
      packet["CV8"] = Pack.CellVoltage8;
      packet["CV9"] = Pack.CellVoltage9;
      packet["CV10"] = Pack.CellVoltage10;
      packet["CV11"] = Pack.CellVoltage11;
      packet["CV12"] = Pack.CellVoltage12;
      packet["CV13"] = Pack.CellVoltage13;
      packet["CV14"] = Pack.CellVoltage14;
      packet["CV15"] = Pack.CellVoltage15;

      packet["TS1Temp"] = Pack.TS1Temp;
      packet["TS2Temp"] = Pack.TS2Temp;
      packet["TS3Temp"] = Pack.TS3Temp;
      packet["CellTemp"] = Pack.CellTemp;
      packet["FETTemp"] = Pack.FETTemp;
      packet["InTemp"] = Pack.InternalTemp;
      packet["CANcol"] = CAN_collisions_detected;
      packet["FWVer"] = Atmel_FW_Version;
      //serializeJson(packet, Serial);
      //Serial.println();
    }
    else {
      // flag fault with BMS
    }
   #endif 

    if (!master_found) {  // if no master yet, chirp our own ID on a semi-random delay

      // first make sure FET outputs are definitely off
      // send FET OFF command
      // byte State = 0x01;
      // //TI_Read(CMD_FET_CONTROL, 1, State);
      // //State &= ~(1UL << 2);   // clear bit 1 to disable DCHG FET
      // TI_Write(CMD_FET_CONTROL, 2, CMD_FET_CONTROL_ACCESS);
      // TI_Write(CMD_FET_CONTROL, 1, State);

      int rand_delay = analogRead(A0);
      delay(rand_delay);  // random-ish delay for collision avoidance

      // serial2mqtt
      // Serial.print("[0,\"dst/");
      // Serial.print(PackID);
      // Serial.println("\"]");

      // JSON output with CAN collision detection
      char out_str[50];
      sprintf(out_str, "{\"PackID\":\"%s\"}", PackID);
      Serial.print(out_str);
      Serial.flush(); // finish sending
      delay(10);
      char in_str[50];
      int i = 0;
      while (Serial.available()) {
        if (i <= 49) { in_str[i] = Serial.read(); }
        else { Serial.read(); }
        i++;
      }
      bool collision = false;
      for(i=0;i==49;i++) {
        if (out_str[i] != in_str[i]) collision = true;
      }
      if (collision) {
        CAN_collisions_detected++;
      }
    }

    #ifdef DEBUG_OUTPUT
      Serial.println();
      
      Serial.print(F("Device Type: \t\t"));
      Serial.println(Versions.TI_DevType, HEX);
      Serial.print(F("Device Number: \t\t"));
      Serial.println(Versions.TI_DevNum, HEX);
      Serial.print(F("TI FW Version: \t\t"));
      Serial.println(Versions.TI_FirmwareVersion, HEX);

      Serial.print(F("Master Found: \t\t"));
      Serial.println(master_found);
      Serial.print(F("Safety Alert AB & CD: \t"));
      Serial.print(Pack.SafetyAlert_AB, HEX);
      Serial.print(F(","));
      Serial.println(Pack.SafetyAlert_CD, HEX);

      Serial.print(F("Safety Status AB & CD: \t"));
      Serial.print(Pack.SafetyStatus_AB, HEX);
      Serial.print(F(","));
      Serial.println(Pack.SafetyStatus_CD, HEX);

      Serial.print(F("Op Status A & B: \t"));
      Serial.printf("%04X", Pack.OpStatus_A);
      //Serial.print(Pack.OpStatus_A, HEX);
      Serial.print(F(","));
      Serial.println(Pack.OpStatus_B, HEX);

      Serial.print(F("PermaFail Stat AB & CD:\t"));
      Serial.print(Pack.PFStatus_AB, HEX);
      Serial.print(F(","));
      Serial.println(Pack.PFStatus_CD, HEX);

      Serial.print(F("PermaFail Alert: \t"));
      Serial.println(Pack.PFAlert, HEX);

      Serial.print(F("Batt Mode: \t\t"));
      Serial.println(Pack.BattMode, HEX);

      Serial.print(F("Batt Status: \t\t"));  
      Serial.println(Pack.BattStatus, HEX);
      
      Serial.print(F("Temperature Status: \t"));
      Serial.println(Pack.TempStatus, HEX);
      
      Serial.print(F("Charge Status: \t\t"));
      Serial.println(Pack.ChgStatus, HEX);
      
      Serial.print(F("Gauge Status: \t\t"));
      Serial.println(Pack.GaugeStatus, HEX);
      
      Serial.print(F("Manufacturer Status: \t"));
      Serial.println(Pack.MfrStatus, HEX);

      Serial.print(F("AnalogFrontEnd Status: \t"));
      Serial.println(Pack.AFEStatus, HEX);

      Serial.print(F("HostFET: \t\t"));
      Serial.println(Pack.HostFET, HEX);

      Serial.print(F("GPIO Status: \t\t"));
      Serial.println(Pack.GPIOStatus, HEX);

      Serial.print(F("GPIO Control: \t\t"));
      Serial.println(Pack.GPIOControl, HEX);

      Serial.print(F("Pack Voltage: \t\t"));
      Serial.println(Pack.Voltage);

      Serial.print(F("Aux Voltage: \t\t"));
      Serial.println(Pack.VAuxVoltage);

      Serial.print(F("Cell Avg Voltage: \t"));
      Serial.println(Pack.ExtAvgVoltage);

      Serial.print(F("Pack Current: \t\t"));
      Serial.println(Pack.Current);

      Serial.print(F("Avg Current: \t\t"));
      Serial.println(Pack.AvgCurrent);

      Serial.print(F("Cycle Count: \t\t"));
      Serial.println(Pack.CycleCount);

      Serial.print(F("Remaining Capacity: \t"));
      Serial.println(Pack.CapacityRemaining);

      Serial.print(F("Full Capacity: \t\t"));
      Serial.println(Pack.CapacityFull);

      Serial.print(F("Runtime to Empty: \t"));
      Serial.println(Pack.RuntimeToEmpty);

      Serial.print(F("Avg Time to Empty: \t"));
      Serial.println(Pack.AvgTimeToEmpty);

      Serial.print(F("Avg Time to Full: \t"));
      Serial.println(Pack.AvgTimeToFull);

      Serial.print(F("Relative SoC: \t\t"));
      Serial.println(Pack.RelativeStateOfCharge);

      Serial.print(F("Absolute SoC: \t\t"));
      Serial.println(Pack.AbsoluteStateOfCharge);

      Serial.print(F("State of Health: \t"));
      Serial.println(Pack.StateOfHealth);

      Serial.print(F("Cell Voltages: \t\t"));
      Serial.print(Pack.CellVoltage1);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage2);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage3);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage4);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage5);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage6);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage7);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage8);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage9);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage10);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage11);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage12);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage13);
      Serial.print(F(", "));
      Serial.print(Pack.CellVoltage14);
      Serial.print(F(", "));
      Serial.println(Pack.CellVoltage15);

      Serial.print(F("Temperatures: \t\t"));
      Serial.print(Pack.TS1Temp);
      Serial.print(F(", "));
      Serial.print(Pack.TS2Temp);
      Serial.print(F(", "));
      Serial.println(Pack.TS3Temp);

      Serial.print(F("Cell Temp: \t\t"));
      Serial.println(Pack.CellTemp);

      Serial.print(F("FET Temp: \t\t"));
      Serial.println(Pack.FETTemp);

      Serial.print(F("Internal Temp: \t\t"));
      Serial.println(Pack.InternalTemp);
    #endif

  }
  sample_timer++;




  
  // check bus for data requests
  while (Serial.available()) { // read bus

    char incoming = Serial.read();
    //{"PackID":"5042394E3530690E1023"}
    //{"PackID":"3430524C344769182529","Output":0}
    //{"PackID":"5042394E3530690E1023","Output":0}         // example messages for copy-paste testing
    //{\"PackID\":\"5042394E3530690E1025\"}
    
    if (incoming == '{') { // then this is a new packet
      NewPacket = true;
      bufferIndex = 0;
      incomingBuffer[bufferIndex] = incoming;

    }
    else if (NewPacket && (incoming != 92)) { // if we're reading in a new packet and the incoming char isn't an escape character
      bufferIndex++;
      incomingBuffer[bufferIndex] = incoming;
      if ((bufferIndex >= 43) or (incoming == '}')) {  // if we've read in enough characters to have received the PackID field, ignore the rest (this will need to be changed to add external ON/OFF commands and so on)
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
        
        if (!error) {
          const char* PackAddr = incoming_packet["PackID"];
        
          // if packet addressed to me
          bool addressedToMe = true;
          for (int i=0;i<sizeof(PackID)-1;i++) {
            if (PackAddr[i] != PackID[i]) {
              addressedToMe = false;
            }
          }
          if (PackAddr[0] == '?') {   // if we've received a general broadcast to re-identify
            // int rand_delay = analogRead(A0);  // then wait for some random time before replying
            // delay(rand_delay);  // random-ish delay for collision avoidance
            float rand_perc = ((float) rand()) / ((float) RAND_MAX);
            int rand_delay = 1000.0 * rand_perc;
            delay(rand_delay);
            addressedToMe = true; // to re-discover a silent pack
          }
          if (addressedToMe) {  // if addressed to me, set master_found flag, reply with a data packet, and make sure FETs are on
            master_found = true;
            
            // reply with packet
            //Serial.println("Matches my ID");

            // serial2mqtt
            // Serial.print("[1,\"src/");
            // Serial.print(PackID);
            // Serial.print("\",\"");

          #ifdef OUTPUT_DATA
            //char out_str[600];
            serializeJson(packet, Serial);  // TODO: change this from Serial to a buffer out_str
            //serializeJson(packet, out_str);  // TODO: change this from Serial to a buffer out_str
            //Serial.print(out_str);
            // flush incoming serial buffer and/or compare against outgoing
            Serial.flush();
            delay(10);
            while (Serial.available()) {Serial.read();}
            //Serial.print("\",0,1]");
            //Serial.print("\"]");
          #endif

            // and finally, send FET enable command to TI (with Pre- CHG/DCHG configured for safety)
            Output = incoming_packet["Output"];
            if (Output) {
              //Serial.println("Output on");
              FET_ON();
              // // send FET ON command
              // byte State = 0x03;
              // //TI_Read(CMD_FET_CONTROL, 1, State);
              // //State |= 1UL << 2;      // set bit 1 to enable DCHG FET
              // TI_Write(CMD_FET_CONTROL, 2, CMD_FET_CONTROL_ACCESS);
              // TI_Write(CMD_FET_CONTROL, 1, State);
            }
            else if (!Output) {
              //Serial.println("Output off");
              FET_OFF();
              // send FET OFF command
              // byte State = 0x01;
              // //TI_Read(CMD_FET_CONTROL, 1, State);
              // //State &= ~(1UL << 2);   // clear bit 1 to disable DCHG FET
              // TI_Write(CMD_FET_CONTROL, 2, CMD_FET_CONTROL_ACCESS);
              // TI_Write(CMD_FET_CONTROL, 1, State);
            }

            //{"PackID":"5042394E3530690E1023","Output":0}
            //{"PackID":"5042394E3530690E1023","Output":1}
            //{"PackID":"?"}
          }
        }
      }
    }
  }

  // // clear the flag so we can run above code again after the MCU wake up
	// f_wdt = 0;

	// // Re-enter sleep mode.
	// enterSleep();

}

