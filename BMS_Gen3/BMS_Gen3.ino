/*
  Switch BMS Expanded Data Exporter - Rolling into some kind of Gen1 Packetised Data Export Solution

  Early version Homer example source for requesting data from the Texas Instruments BQ78350 controller
  over the I2C interface (SMBus) and exporting it on the serial interface for development purposes.
  Includes PJON wrapper for fixed addressed communication.

  Last Modified: 7th Aug 2018
  This application version: v0.7
  Connects to: BMS_OpCentre v0.3 or later
 
*/

/*
 * TI BQ78350-R1 useful command set (21Jul2018):
 * 
 * SBS Commands: 
 * 
 * 0x03 - BatteryMode - 2 bytes, HEX, Battery Mode Status and Control Flag Bits
 * 0x08 - Temperature - 2 bytes, unsigned int, 0.1 degree Kelvin units
 * 0x09 - Voltage - 2 bytes, unsigned int, mV
 * 0x0A - Current - 2 bytes, signed int, 10mA units
 * 0x0B - AvgCurrent - 2 bytes, signed int, 10mA units
 * 0x0D - Relative SoC - 1 byte, unsigned int, %
 * 0x0E - Absolute SoC - 1 byte, unsigned int, %
 * 0x0F - Remaining Capacity - 2 bytes, unsigned int, mAh
 * 0x10 - Full Charge Capacity - 2 bytes, unsigned int, mAh
 * 0x11 - Run Time to Empty - 2 bytes, unsigned int, minutes
 * 0x12 - Avg Time to Empty - 2 bytes, unsigned int, minutes
 * 0x13 - Avg Time to Full - 2 bytes, unsigned int, minutes
 * 0x16 - BatteryStatus - 2 bytes, Battery Status Flag bits
 * 0x17 - CycleCount - 2 bytes, unsigned int, # of cycles
 * 0x1C - SerialNumber - 2 bytes, HEX
 * 0x2B - Host FET control - 2 bytes, HEX, FET Control Flag bits
 * 0x2C - GPIOStatus - 2 bytes, HEX, GPIO Status Flag bits
 * 0x2D - GPIOControl - 2 bytes, HEX, GPIO Control Flag bits
 * 0x2E - VAUX Voltage - 2 bytes, unsigned int, mV
 * 0x2F - Authenticate - 20+1 bytes, HEX, round-trip SHA-1 authentication process, generate HMAC1, test HMAC2 = HMAC3
 * 0x31
 * to F - Cell Voltages 15 down to 1, 2 bytes, unsigned int, mV
 * 0x4D - ExtAvgCellVoltage - 2 bytes, unsigned int, mV
 * 0x4F - StateOfHealth - 1 byte, unsigned int, %
 * 0x50 - SafetyAlert - 4+1 bytes, HEX, Safety Alert Flag bits
 * 0x51 - SafetyStatus - 4+1 bytes, HEX, Safety Status Flag bits
 * 0x52 - PermanentFailAlert - 2+1 bytes, HEX, Permanent Fail Alert Flag bits
 * 0x53 - PermanentFailStatus - 4+1 bytes, HEX, Permanent Fail Status Flag bits
 * 0x54 - OperationStatus - 4+1 bytes, HEX, Operation Status Flag bits
 * 0x55 - ChargingStatus - 2+1 bytes, HEX, Temperature Range Flag bits, Charging Status Flag bits
 * 0x56 - GaugingStatus - 2+1 bytes, HEX, Gauging Status Flag bits
 * 0x57 - ManufacturingStatus - 2+1 bytes, HEX, Manufacturing Status Flag and Control bits
 * 0x58 - AFEStatus - 2+1 bytes, HEX, Analog Front End SYS_STAT (0x00) Flag bits
 * 0x59 - AFEConfig - 10+1 bytes, Analog Front End Config CELLBAL1,2,3, SYS_CTRL1,2, PROTECT1,2,3, OV_TRIP, UV_TRIP, CC_CFG (0x01 to 0x0B) Flag bits
 * 0x60
 * to 6 - Lifetime Data Block 1 to 7, 32+1 bytes, see BQ78350-R1 Tech Reference for detail, Section 10.2 Lifetimes
 * 0x71 - DAStatus1 - 30+2 bytes (32+1?), Cell Voltages 0-15, final 2 bytes are reserved junk, mV
 * 0x72 - DAStatus2 - 16+1 bytes (32+1?), ExtAvgCellVoltage, VAuxVoltage, TS1, TS2, TS3, Cell Temp, FET Temp, Internal Temp
 * 0x80 - CUV Snapshot - 30+2 bytes (32+1?), Cell Voltages 0-15 at last UV event, mV
 * 0x81 - COV Snapshot - 30+2 bytes (32+1?), Cell Voltages 0-15 at last OV event, mV
 * 
 * Manufacturer Access Commands:
 * 
 * 0x0001 - Device Type - 2 bytes, IC part #
 * 0x0002 - Firmware Version - 10 bytes, Device Number, Version, Firmware Type, CEDV Version, Reserved
 * 0x0003 - Hardware Version - 2 bytes, Hardware Version
 * 0x0010 - Shutdown Mode (send twice)
 * 0x0011 - Sleep Mode
 * 0x0012 - Device Reset
 * 0x0030 - Seal Device
 * 0x0035 - Security Keys
 * 0x0037 - Authentication Key
 * 0x0041 - Device Reset
 * 0x01yy - DFlash Access Row Address
 * 0x0F00 - ROM Programming Mode - TI Firmware update mechanism?
 * 
 */

#define BMS_FIRMWARE_VERSION 0x0007
#define MY_PJON_ADDR 2 // PJON_NOT_ASSIGNED
#define TI_Addr 0x0B    // Hard coded address of TI BQ78350 on SMBus / I2C

#define PJON_PACKET_MAX_LENGTH 90
//#define CMD_
//#define LEN_

// Useful discrete values
#define CMD_Temperature 0x08 // Temperature
#define LEN_IN_Temperature 2
#define LEN_OUT_Temperature 2
#define CMD_Voltage 0x09 // Voltage
#define LEN_IN_Voltage 2
#define LEN_OUT_Voltage 2
#define CMD_Current 0x0A // Current
#define LEN_IN_Current 2
#define LEN_OUT_Current 2
#define CMD_AvgCurrent 0x0B // Avg Current
#define LEN_IN_AvgCurrent 2
#define LEN_OUT_AvgCurrent 2
#define CMD_VAuxVoltage 0x2E
#define LEN_IN_VAuxVoltage 2
#define LEN_OUT_VAuxVoltage 2
#define CMD_ExtAvgVoltage 0x4D
#define LEN_IN_ExtAvgVoltage 2
#define LEN_OUT_ExtAvgVoltage 2

// Data blocks
#define CMD_DAStatus1 0x71 // Cell voltages data block
#define LEN_IN_DAStatus1 33
#define LEN_OUT_DAStatus1 15
#define CMD_DAStatus2 0x72 // Temperatures data block
#define LEN_IN_DAStatus2 15
#define LEN_OUT_DAStatus2 7

// Capacity estimates
#define CMD_RelSoC 0x0D // Relative State of Charge
#define LEN_IN_RelSoC 1
#define LEN_OUT_RelSoC 1
#define CMD_AbsSoC 0x0E // Absolute State of Charge
#define LEN_IN_AbsSoC 1
#define LEN_OUT_AbsSoC 1
#define CMD_CapRemain 0x0F // Remaining Capacity
#define LEN_IN_CapRemain 2
#define LEN_OUT_CapRemain 2
#define CMD_CapFull 0x10 // Full Capacity
#define LEN_IN_CapFull 2
#define LEN_OUT_CapFull 2
#define CMD_RunTTE 0x11 // Run Time to Empty
#define LEN_IN_RunTTE 2
#define LEN_OUT_RunTTE 2
#define CMD_AvgTTE 0x12 // Avg Time To Empty
#define LEN_IN_AvgTTE 2
#define LEN_OUT_AvgTTE 2
#define CMD_AvgTTF 0x13 // Avg Time To Full
#define LEN_IN_AvgTTF 2
#define LEN_OUT_AvgTTF 2
#define CMD_SoH 0x4F // State of Health
#define LEN_IN_SoH 1
#define LEN_OUT_SoH 1

// Status flags and controls
#define CMD_BattMode 0x03 // Battery Mode
#define LEN_IN_BattMode 2
#define LEN_OUT_BattMode 2
#define CMD_BattStatus 0x16 // Battery Status
#define LEN_IN_BattStatus 2
#define LEN_OUT_BattStatus 2
#define CMD_CycleCount 0x17 // Cycle Count
#define LEN_IN_CycleCount 2
#define LEN_OUT_CycleCount 2
#define CMD_SafetyAlert 0x50 // Safety Alert
#define LEN_IN_SafetyAlert 5
#define LEN_OUT_SafetyAlert 4
#define CMD_SafetyStatus 0x51 // Safety Status
#define LEN_IN_SafetyStatus 5
#define LEN_OUT_SafetyStatus 4
#define CMD_PFAlert 0x52 // Permanent Fail Alert
#define LEN_IN_PFAlert 3
#define LEN_OUT_PFAlert 2
#define CMD_PFStatus 0x53 // Permanent Fail Status
#define LEN_IN_PFStatus 5
#define LEN_OUT_PFStatus 4
#define CMD_OpStatus 0x54 // Operations Status
#define LEN_IN_OpStatus 5
#define LEN_OUT_OpStatus 4
#define CMD_ChgStatus 0x55 // Charge Status
#define LEN_IN_ChgStatus 3
#define LEN_OUT_ChgStatus 2
#define CMD_GaugeStatus 0x56 // Gauging Status
#define LEN_IN_GaugeStatus 3
#define LEN_OUT_GaugeStatus 2
#define CMD_MfrStatus 0x57 // Manufacturer's Status
#define LEN_IN_MfrStatus 3
#define LEN_OUT_MfrStatus 2
#define CMD_AFEStatus 0x58 // Analog Front End Status
#define LEN_IN_AFEStatus 3
#define LEN_OUT_AFEStatus 2
#define CMD_AFEConfig 0x59 // Analog Front End Config data block
#define LEN_IN_AFEConfig 11 // 10?
#define LEN_OUT_AFEConfig 10 // 10?

// Security & Versions
#define CMD_SerialNum 0x1C // serial number
#define LEN_IN_SerialNum 2
#define LEN_OUT_SerialNum 2
#define CMD_Authenticate 0x2F // authenticate
#define LEN_IN_Authenticate 21
#define LEN_OUT_Authenticate 20
#define CMD_DeviceType 0x0001
#define LEN_IN_DeviceType 2
#define LEN_OUT_DeviceType 2
#define CMD_FirmwareVersion 0x0002
#define LEN_IN_FirmwareVersion 10
#define LEN_OUT_FirmwareVersion 8
#define CMD_HardwareVersion 0x0003
#define LEN_IN_HardwareVersion 2
#define LEN_OUT_HardwareVersion 2

// Hardware controls
#define CMD_HostFET 0x2B // host FET control
#define LEN_IN_HostFET 2
#define LEN_OUT_HostFET 2
#define CMD_GPIOStatus 0x2C // GPIOStatus
#define LEN_IN_GPIOStatus 2
#define LEN_OUT_GPIOStatus 2
#define CMD_GPIOControl 0x2D // GPIOControl
#define LEN_IN_GPIOControl 2
#define LEN_OUT_GPIOControl 2

// Fault reports
#define CMD_CUVSnap 0x80 // CUV Snapshot
#define LEN_IN_CUVSnap 33
#define LEN_OUT_CUVSnap 32
#define CMD_COVSnap 0x81 // COV Snapshot
#define LEN_IN_COVSnap 33
#define LEN_OUT_COVSnap 32

// Firmware updates?
// #define // ROM programming mode

#define Cycle_Time 4000   // move this to permanent settings storage in EEPROM and pull it back up on startup
#define Safety_Cycles 500
#define Data_Cycles_Hi 1000
#define Data_Cycles_Lo 5000

// These arrays are all 1 byte larger than strictly necessary due to errors discovered in TI reference documentation
// for expected responses with and without leading and trailing bytes included, etc.
//byte AVR_Serial[]=     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//byte SerialNum[]=      { 0x00, 0x00, 0x00 };
byte Authenticate[]=   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00 };
/*
byte Temperature[]=    { 0x00, 0x00, 0x00 };
byte Voltage[]=        { 0x00, 0x00, 0x00 };
byte VAuxVoltage[]=    { 0x00, 0x00, 0x00 };
byte ExtAvgVoltage[]=  { 0x00, 0x00, 0x00 };
byte Current[]=        { 0x00, 0x00, 0x00 };
byte AvgCurrent[]=     { 0x00, 0x00, 0x00 };
byte CycleCount[]=     { 0x00, 0x00, 0x00 };
byte CapRemain[]=      { 0x00, 0x00, 0x00 };
byte CapFull[]=        { 0x00, 0x00, 0x00 };
byte RunTTE[]=         { 0x00, 0x00, 0x00 };
byte AvgTTE[]=         { 0x00, 0x00, 0x00 };
byte AvgTTF[]=         { 0x00, 0x00, 0x00 };

byte SafetyAlert[]=    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte SafetyStatus[]=   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte OpStatus[]=       { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte PFStatus[]=       { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte PFAlert[]=        { 0x00, 0x00, 0x00 };
byte BattMode[]=       { 0x00, 0x00, 0x00 };
byte BattStatus[]=     { 0x00, 0x00, 0x00 };
byte ChgStatus[]=      { 0x00, 0x00, 0x00 };
byte GaugeStatus[]=    { 0x00, 0x00, 0x00 };
byte MfrStatus[]=      { 0x00, 0x00, 0x00 };
byte AFEStatus[]=      { 0x00, 0x00, 0x00 };
byte HostFET[]=        { 0x00, 0x00, 0x00 };
byte GPIOStatus[]=     { 0x00, 0x00, 0x00 };
byte GPIOControl[]=    { 0x00, 0x00, 0x00 };

byte RelSoC[]=         { 0x00, 0x00 };
byte AbsSoC[]=         { 0x00, 0x00 };
byte SoH[]=            { 0x00, 0x00 };

byte Cell_Volt_Data[]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00 };
byte Cell_Temp_Data[]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00 };
*/                         

byte CUVSnap[]=        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00 };
byte COVSnap[]=        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00 };                         

// Rebuild data structures here for incoming data from TI chipset and outgoing data packets

struct Version_Info {

  byte TI_DevType[2];
  byte FirmwareVersionsBuffer[10];
  byte TI_DevNum[2];
  byte TI_FirmwareVersion[2];
  byte TI_FirmwareType[2];
  byte TI_CEDV[2];
  byte TI_HW_Ver[2];
  byte AVR_Dev_ID[3];
  byte AVR_FirmwareVersion[2];
  byte AVR_Signature[10];
  
};
  
struct Incoming_Flags {

  byte SafetyAlert[5];
  byte SafetyStatus[5];
  byte OpStatus[5];
  byte PFStatus[5];
  byte PFAlert[3];
  byte BattMode[3];
  byte BattStatus[3];
  byte ChgStatus[3];
  byte GaugeStatus[3];
  byte MfrStatus[3];
  byte AFEStatus[3];
  byte HostFET[3];
  byte GPIOStatus[3];
  byte GPIOControl[3];

};

struct Incoming_Data {

  byte Temperature[3];
  byte Voltage[3];
  byte VAuxVoltage[3];
  byte ExtAvgVoltage[3];
  byte TempTop[3];
  byte TempMiddle[3];
  byte TempBottom[3];
  byte Current[3];
  byte AvgCurrent[3];
  byte CycleCount[3];
  byte CapRemain[3];
  byte CapFull[3];
  byte RunTTE[3];
  byte AvgTTE[3];
  byte AvgTTF[3];

  byte RelSoC[2];
  byte AbsSoC[2];
  byte SoH[2];

  byte Cell_Volt_Data[34];
  byte Cell_Temp_Data[18];

};

/*struct Export_Header {

  // my unique identifier
  byte Address[10];
  // destination identifier
  // message priority?
  // length of data to come?
  // error checking?
  
};*/

struct Export_Data {

  // safety variables - priority items
  word SafetyAlert_A;
  word SafetyAlert_B;
  word SafetyStatus_A;
  word SafetyStatus_B;
  word OpStatus_A;
  word OpStatus_B;
  word PFStatus_A;
  word PFStatus_B;
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
  uint16_t Current;         // signed int
  uint16_t AvgCurrent;      // signed int
  uint16_t CycleCount;
  uint16_t CapRemain;
  uint16_t CapFull;
  uint16_t RunTTE;
  uint16_t AvgTTE;
  uint16_t AvgTTF;

  uint8_t RelSoC;
  uint8_t AbsSoC;
  uint8_t SoH;

  uint16_t Cell_Volt_Data[15];
  uint16_t Cell_Temp_Data[7];
  
};

struct Version_Info Versions;
struct Incoming_Flags Flags;    // Make this "Flags[];" if we want to create an array of Flag records - Queue, most likely?
struct Incoming_Data Data;
//struct Export_Header Header;
//struct Export_Data Packet;

byte cmdbuffer[50]; // [255]; ?
int cmdbuf_len = 0;
bool fresh_buffer = false;


// Included libraries

#include <Wire.h> // for I2C communications with TI Chipset (SMBus)
#include <avr/boot.h>   // for accessing unique serial number identifier of the ATMega328PB
#include <PJON.h>   // serial comms wrapper with auto address negotiation, packetised transfer with CRC, etc.


//uint8_t bus_id[] = {0, 0, 0, 1};
// <Strategy name> bus(bus_id, device id)
PJON<ThroughSerial> bus(MY_PJON_ADDR);



void setup() {

  // we've arrived here after the bootloader
  // check registers to investigate reason for the reset/restart/reboot
  // load persistent settings from the EEPROM and get myself set up for operations


#define SIGRD 5
#if defined(SIGRD) || defined(RSIG)
    Versions.AVR_Dev_ID[0] = boot_signature_byte_get(0);
    Versions.AVR_Dev_ID[1] = boot_signature_byte_get(2);
    Versions.AVR_Dev_ID[2] = boot_signature_byte_get(4);
    for (uint8_t i = 14; i < 24; i += 1) {
        Versions.AVR_Signature[i-14] = boot_signature_byte_get(i);  // Read my own unique ID
    }
#endif

  // start communications interfaces
  
  Wire.begin();     // connection to TI BMS Chipset
  //Wire.onReceive(receiveEvent); // register event

  Serial.begin(19200);  // compiled for a 16MHz Nano, running at 8MHz on Switch BMS gives real baudrate of 9600.
  bus.strategy.set_serial(&Serial);
  //bus.set_error(error_handler);
  bus.set_receiver(receiver_handler);
  bus.strategy.set_enable_RS485_pin(12);
  bus.set_synchronous_acknowledge(false);
  
  bus.begin();

  // if I have an existing address, confirm it with the bus master using my unique serial ID in case I've been hot swapped
  // if I don't have an address, request a new one from the master
  //bus.acquire_id_master_slave();
  // store my address in EEPROM settings
  
  PullVersionInfo();
  
}


void loop() {

  byte incoming_cmd = 0;
  int Check_Serial_Timer = 0;

  // FIX -- add some watchdog timer management for avoiding getting stuck in comms handling loops between reading battery safety data

  // request fresh battery health data
    if (RequestData())    // FIX -- variable passing
      {
      
      // check for safety issues and handle them
      
      // until I'm next due to request fresh data, listen for serial commands and react to them
      while (Check_Serial_Timer < Cycle_Time)
        {
          // check serial queue for a data request command
          bus.update();
          //uint16_t response = 
          bus.receive();
          
          // if data request, SendPacket();
          //if (response == PJON_ACK)
          //if (Serial.available() > 0) 
          if (fresh_buffer)
          {
            // read incoming serial data
            //incoming_cmd = Serial.read();
            //Serial.println(incoming_cmd);
            //fresh_buffer = true;
          //}

            //if (fresh_buffer)  // redundant check? == PJON_ACK anyway?
            //{
              // parse incoming serial data and react to it
              ParsePayload(incoming_cmd);     // FIX -- variable passing

              bus.reply(cmdbuffer, cmdbuf_len);
            //bus.update();

              //SendBuffer();
              fresh_buffer = false;
              
            //}
            
            // switch case for operational flags after parsing payload?
            // if a successful data transmission event occurred exit loop to go refresh data now
            Check_Serial_Timer = Cycle_Time;
            
          }
        Check_Serial_Timer++;
        
        } // end time delay loop
      Check_Serial_Timer = 0; // reset loop variable after successful exit
      
      }
    else
      {
        //Serial.println("Failed to build incoming data packet. Retrying...");
        // failed to communicate successfully with TI chipset to refresh battery data
        // probably try again a couple of times and then
        // maybe try to boot the TI chipset and try again
        // maybe try to reset the TI chipset and try again
        // after everything has failed announce this to the bus and go to safety shutdown, wait for commands
      }
       
    //delay(Cycle_Time);
}



bool PullVersionInfo(){//struct Incoming_Flags FLAGS, struct Incoming_Data DATA){

    // FIX -- Fill in detail

  if (TI_Interact(CMD_DeviceType, LEN_IN_DeviceType, Versions.TI_DevType)) {
  if (TI_Interact(CMD_FirmwareVersion, LEN_IN_FirmwareVersion, Versions.FirmwareVersionsBuffer)) {      

  Versions.TI_DevNum[0] = Versions.FirmwareVersionsBuffer[0];
  Versions.TI_DevNum[1] = Versions.FirmwareVersionsBuffer[1];
  Versions.TI_FirmwareVersion[0] = Versions.FirmwareVersionsBuffer[2];
  Versions.TI_FirmwareVersion[1] = Versions.FirmwareVersionsBuffer[3];
  Versions.TI_FirmwareType[0] = Versions.FirmwareVersionsBuffer[4];
  Versions.TI_FirmwareType[1] = Versions.FirmwareVersionsBuffer[5];
  Versions.TI_CEDV[0] = Versions.FirmwareVersionsBuffer[6];
  Versions.TI_CEDV[1] = Versions.FirmwareVersionsBuffer[7];

  if (TI_Interact(CMD_HardwareVersion, LEN_IN_HardwareVersion, Versions.TI_HW_Ver)) {
  //if (TI_Interact(CMD_Authenticate, LEN_IN_Authenticate, Authenticate)) { // Authenticate

  Versions.AVR_FirmwareVersion[0] = BMS_FIRMWARE_VERSION >> 8;
  Versions.AVR_FirmwareVersion[1] = BMS_FIRMWARE_VERSION;

    return 1;
  }}}
  return 0;
}


bool RequestData(){//struct Incoming_Flags FLAGS, struct Incoming_Data DATA){

  // FIX -- Pass pointers for data variables
  
  if (TI_Interact(CMD_SafetyAlert, LEN_IN_SafetyAlert, Flags.SafetyAlert)) {  // Safety Alert
  if (TI_Interact(CMD_SafetyStatus, LEN_IN_SafetyStatus, Flags.SafetyStatus)) {  // Safety Status
  if (TI_Interact(CMD_BattMode, LEN_IN_BattMode, Flags.BattMode)) {  // Battery Mode  
  if (TI_Interact(CMD_BattStatus, LEN_IN_BattStatus, Flags.BattStatus)) {  // Battery Status
  if (TI_Interact(CMD_OpStatus, LEN_IN_OpStatus, Flags.OpStatus)) {  // Operational Status
  if (TI_Interact(CMD_ChgStatus, LEN_IN_ChgStatus, Flags.ChgStatus)) {  // Charge Status           // WARNING -- This 2x 8bit register pair is reversed
  if (TI_Interact(CMD_GaugeStatus, LEN_IN_GaugeStatus, Flags.GaugeStatus)) {  // Gauge Status
  if (TI_Interact(CMD_MfrStatus, LEN_IN_MfrStatus, Flags.MfrStatus)) {  // Manufacturer Status
  if (TI_Interact(CMD_AFEStatus, LEN_IN_AFEStatus, Flags.AFEStatus)) {  // Analog Front End Status

  if (TI_Interact(CMD_HostFET, LEN_IN_HostFET, Flags.HostFET)) { // Host FET control
  if (TI_Interact(CMD_GPIOStatus, LEN_IN_GPIOStatus, Flags.GPIOStatus)) { // GPIO Status
  if (TI_Interact(CMD_GPIOControl, LEN_IN_GPIOControl, Flags.GPIOControl)) { // GPIO Control

  //if (TI_Interact(CMD_Temperature, LEN_IN_Temperature, Temperature)) { // Temperature -- This is an averaged value anyway, DAStatus2 contains full detail
  if (TI_Interact(CMD_Voltage, LEN_IN_Voltage, Data.Voltage)) { // Voltage
  //if (TI_Interact(CMD_VAuxVoltage, LEN_IN_VAuxVoltage, VAuxVoltage)) { // VAuxVoltage -- DAStatus2 contains
  //if (TI_Interact(CMD_ExtAvgVoltage, LEN_IN_ExtAvgVoltage, ExtAvgVoltage)) { // ExtAvgVoltage -- DAStatus2 contains
  if (TI_Interact(CMD_Current, LEN_IN_Current, Data.Current)) {  // Current
  if (TI_Interact(CMD_AvgCurrent, LEN_IN_AvgCurrent, Data.AvgCurrent)) {  // AvgCurrent

  if (TI_Interact(CMD_RelSoC, LEN_IN_RelSoC, Data.RelSoC)) { // Relative State of Charge
  if (TI_Interact(CMD_AbsSoC, LEN_IN_AbsSoC, Data.AbsSoC)) { // Absolute State of Charge
  if (TI_Interact(CMD_CapRemain, LEN_IN_CapRemain, Data.CapRemain)) { // Remaining Capacity
  if (TI_Interact(CMD_CapFull, LEN_IN_CapFull, Data.CapFull)) { // Full Capacity
  if (TI_Interact(CMD_RunTTE, LEN_IN_RunTTE, Data.RunTTE)) { // Run Time to Empty
  if (TI_Interact(CMD_AvgTTE, LEN_IN_AvgTTE, Data.AvgTTE)) { // Avg Time to Empty
  if (TI_Interact(CMD_AvgTTF, LEN_IN_AvgTTF, Data.AvgTTF)) { // Avg Time to Empty
  if (TI_Interact(CMD_CycleCount, LEN_IN_CycleCount, Data.CycleCount)) { // Cycle count
  if (TI_Interact(CMD_SoH, LEN_IN_SoH, Data.SoH)) { // State of Health
    
  if (TI_Interact(CMD_DAStatus1, LEN_IN_DAStatus1, Data.Cell_Volt_Data)) { // Cell Voltages Data Block
  if (TI_Interact(CMD_DAStatus2, LEN_IN_DAStatus2, Data.Cell_Temp_Data)) { // Temperatures Data Block

        return 1;
  }}}}}}}}}}}}}}}}}}}}}}}}}}//}}}
  return 0;
}


bool TI_Interact(byte CMD, int LEN, byte IN_DATA[]){ // FIX -- Pass pointers for data variable

  Wire.beginTransmission(TI_Addr);
  Wire.write(CMD);
  
  // FIX -- if LEN not zero then send follow-up commands to prompt TI to spew data, else command was a one-shot with a predictable response
  Wire.endTransmission(false);
  Wire.requestFrom(TI_Addr, LEN, true);
    
    for(int i = 0; i < LEN; i++){        // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
      IN_DATA[i] = Wire.read();          // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
  // else
  // Wire.endTransmission(true);
  // FIX -- read whatever was predictable about a CMD with a zero length reply
  
  return 1;  
}


void BuildFlagPayload(){//struct Incoming_Flags FLAGS, struct Incoming_Data DATA, struct Export_Header HEADER, struct Export_Data PACKET){

  // Crawl incoming data structures and pull bytes to assemble outgoing packets
  // Rather than copy data from one memory location to another, hand pointers between structures and hold data in memory only once?
  // Maybe using pointers doesn't scale well for storing queues of data records during comms outages?

  // safety variables - priority items
  
  cmdbuffer[3] = Flags.SafetyAlert[2];//<<8;  // SafetyAlert_A
  cmdbuffer[4] = Flags.SafetyAlert[1];
  cmdbuffer[5] = Flags.SafetyAlert[4];//<<8;  // SafetyAlert_B
  cmdbuffer[6] = Flags.SafetyAlert[3];
  cmdbuffer[7] = Flags.SafetyStatus[2];//<<8; // SafetyStatus_A
  cmdbuffer[8] = Flags.SafetyStatus[1];
  cmdbuffer[9] = Flags.SafetyStatus[4];//<<8; // SafetyStatus_B
  cmdbuffer[10] = Flags.SafetyStatus[3];
  cmdbuffer[11] = Flags.OpStatus[2];//<<8;    // OpStatus_A
  cmdbuffer[12] = Flags.OpStatus[1];
  cmdbuffer[13] = Flags.OpStatus[4];//<<8;    // OpStatus_B
  cmdbuffer[14] = Flags.OpStatus[3];
  cmdbuffer[15] = Flags.PFStatus[2];//<<8;    // PFStatus_A
  cmdbuffer[16] = Flags.PFStatus[1];
  cmdbuffer[17] = Flags.PFStatus[4];//<<8;    // PFStatus_B
  cmdbuffer[18] = Flags.PFStatus[3];
  cmdbuffer[19] = Flags.PFAlert[2];//<<8;    // PFAlert
  cmdbuffer[20] = Flags.PFAlert[1];
  cmdbuffer[21] = Flags.BattMode[1];//<<8;    // BattMode
  cmdbuffer[22] = Flags.BattMode[0];
  cmdbuffer[23] = Flags.BattStatus[1];//<<8;    // BattStatus
  cmdbuffer[24] = Flags.BattStatus[0];
  cmdbuffer[25] = Flags.ChgStatus[2];//<<8;   // TempStatus
  cmdbuffer[26] = Flags.ChgStatus[1];          // ChgStatus
  cmdbuffer[27] = Flags.GaugeStatus[2];//<<8;  // GaugeStatus
  cmdbuffer[28] = Flags.GaugeStatus[1];
  //Packet.MfrStatus = Flags.MfrStatus[2]<<8;
  //Packet.MfrStatus |= Flags.MfrStatus[1];
  //Packet.AFEStatus = Flags.AFEStatus[2]<<8;
  //Packet.AFEStatus |= Flags.AFEStatus[1];

  // control variables - influence hardware operations
  cmdbuffer[29] = Flags.HostFET[2];//<<8;    // HostFET
  cmdbuffer[30] = Flags.HostFET[1];
  cmdbuffer[31] = Flags.GPIOStatus[2];//<<8;    // GPIOStatus
  cmdbuffer[32] = Flags.GPIOStatus[1];
  cmdbuffer[33] = Flags.GPIOControl[2];//<<8;  // GPIOControl
  cmdbuffer[34] = Flags.GPIOControl[1];
    
}


void BuildDataPayload(){//struct Incoming_Flags FLAGS, struct Incoming_Data DATA, struct Export_Header HEADER, struct Export_Data PACKET){

  // Crawl incoming data structures and pull bytes to assemble outgoing packets
  // Rather than copy data from one memory location to another, hand pointers between structures and hold data in memory only once?
  // Maybe using pointers doesn't scale well for storing queues of data records during comms outages?

  // data variables - for reporting
  //Packet.Temperature = Data.Temperature[1]<<8;
  //Packet.Temperature |= Data.Temperature[0];
  cmdbuffer[3] = Data.Voltage[1];//<<8;
  cmdbuffer[4] = Data.Voltage[0];
  //Packet.VAuxVoltage = Data.VAuxVoltage[1]<<8;
  //Packet.VAuxVoltage |= Data.VAuxVoltage[0];
  //Packet.ExtAvgVoltage = Data.ExtAvgVoltage[1]<<8;
  //Packet.ExtAvgVoltage |= Data.ExtAvgVoltage[0];
  cmdbuffer[5] = Data.Current[1];//<<8;
  cmdbuffer[6] = Data.Current[0];
  cmdbuffer[7] = Data.AvgCurrent[1];//<<8;
  cmdbuffer[8] = Data.AvgCurrent[0];
  cmdbuffer[9] = Data.CycleCount[1];//<<8;
  cmdbuffer[10] = Data.CycleCount[0];
  cmdbuffer[11] = Data.CapRemain[1];//<<8;
  cmdbuffer[12] = Data.CapRemain[0];
  cmdbuffer[13] = Data.CapFull[1];//<<8;
  cmdbuffer[14] = Data.CapFull[0];
  cmdbuffer[15] = Data.RunTTE[1];//<<8;
  cmdbuffer[16] = Data.RunTTE[0];
  cmdbuffer[17] = Data.AvgTTE[1];//<<8;
  cmdbuffer[18] = Data.AvgTTE[0];
  cmdbuffer[19] = Data.AvgTTF[1];//<<8;
  cmdbuffer[20] = Data.AvgTTF[0];

  cmdbuffer[21] = Data.RelSoC[0];
  cmdbuffer[22] = Data.AbsSoC[0];
  cmdbuffer[23] = Data.SoH[0];

}


void BuildCellDataPayload(){

  // FIX -- these loops are broken
  //Serial.write(2);
  for(int x = 0; x < LEN_IN_DAStatus1; x=x+2){                             // for all possible 15 battery cells
    cmdbuffer[x+3] = Data.Cell_Volt_Data[x+2];//<<8;
    cmdbuffer[x+4] = Data.Cell_Volt_Data[x+1];
    //Serial.write(x);
  }
  //Serial.write(3);
  for(int x = 0; x < LEN_IN_DAStatus2; x=x+2){             
    cmdbuffer[(x+3)+(LEN_IN_DAStatus1-1)] = Data.Cell_Temp_Data[x+2];//<<8; // cmdbuffer index offset by existing data
    cmdbuffer[(x+4)+(LEN_IN_DAStatus1-1)] = Data.Cell_Temp_Data[x+1];
  }
  //Serial.write(4);
    
}


void BuildVersionDataPayload(){

  cmdbuffer[3] = Versions.TI_DevType[0];
  cmdbuffer[4] = Versions.TI_DevType[1];
  cmdbuffer[5] = Versions.TI_DevNum[0];
  cmdbuffer[6] = Versions.TI_DevNum[1];
  cmdbuffer[7] = Versions.TI_FirmwareVersion[0];
  cmdbuffer[8] = Versions.TI_FirmwareVersion[1];
  cmdbuffer[9] = Versions.TI_FirmwareType[0];
  cmdbuffer[10] = Versions.TI_FirmwareType[1];
  cmdbuffer[11] = Versions.TI_CEDV[0];
  cmdbuffer[12] = Versions.TI_CEDV[1];
  cmdbuffer[13] = Versions.TI_HW_Ver[0];
  cmdbuffer[14] = Versions.TI_HW_Ver[1];
  cmdbuffer[15] = Versions.AVR_Dev_ID[0];
  cmdbuffer[16] = Versions.AVR_Dev_ID[1];
  cmdbuffer[17] = Versions.AVR_Dev_ID[2];
  cmdbuffer[18] = Versions.AVR_FirmwareVersion[0];
  cmdbuffer[19] = Versions.AVR_FirmwareVersion[1];
  for (int i = 0; i < 10; i++){
    cmdbuffer[i+20] = Versions.AVR_Signature[i];
  }

}


void ParsePayload(byte incoming_cmd)
{

  // this is actually a switch case for the full command set after the payload has been extracted
  // first byte of the payload will be the command from the master
  switch(cmdbuffer[0]){  
  //switch(incoming_cmd){

  // === Data Request Commands ===================
  case 0xDA: // Safety Flags Data Packet request
              cmdbuffer[0] = 0xDA;  // re-write this to the buffer in case it came through corrupted, don't just reply OK!
              cmdbuffer[1] = 0xAA;  // ACK cmd 0xDA
              cmdbuffer[2] = 0x20;  // 32 bytes to follow
              BuildFlagPayload();   // 32 bytes of data added to cmdbuffer
              cmdbuf_len = 35;
              break;
    
  case 0xDD: // Specific Data Value request
              cmdbuffer[0] = 0xDD;
              cmdbuffer[1] = 0xAA;  // ACK cmd 0xDD
              cmdbuffer[2] = 0x15;  // 21 bytes to follow
              BuildDataPayload();
              cmdbuf_len = 24;
              break;
  
  case 0xDC: // Complete Live Data Packet request
              //Serial.write(1);
              cmdbuffer[0] = 0xDC;
              cmdbuffer[1] = 0xAA;
              cmdbuffer[2] = 0x2E;  // 46 bytes to follow
              BuildCellDataPayload();   // FIX -- variable passing
              cmdbuf_len = 49;
              // bus.send payload
              // check for ACK? resend?
              break;
  
  case 0xDF: // Report firmware and hardware versions request
              cmdbuffer[0] = 0xDF;
              cmdbuffer[1] = 0xAA;
              cmdbuffer[2] = 0x1D;  // 29 bytes to follow
              BuildVersionDataPayload();   // FIX -- variable passing
              cmdbuf_len = 32;
              break;

/*  case 0xD0:{ // Lifetime data packet request
              break;
  } // end 0xD0

  case 0xD1:{ // UnderVoltage data packet request
              break;
  } // end 0xD1

  case 0xD2:{ // OverVoltage data packet request
              break;
  } // end 0xD2

  case 0xD3:{ // Permanent Fail data packet request
              break;
  } // end 0xD3

  case 0xD9:{ // Upload stored data packets request
              break;
  } // end 0xD9

  // === Bootstate Commands ====================
  case 0xBB:{ // Boot TI command
              // drive the correct pin to the correct state to trigger the TI boot
              // float the pin to avoid affecting cell voltage readings
              break;
  } // end 0xBB

  case 0xBF:{ // Shutdown TI command
              // send the shutdown command twice in quick succession - required by TI
              break;
  } // end 0xBF

  case 0xB0:{ // Reset TI command
              break;
  } // end 0xB0

  case 0xB1:{ // Reset BMS command
              // that's a reboot me command!
              // set watchdog timer to timeout
              // wait
              break; // <-- we'll never get here anyway
  } // end 0xB1

  case 0xB2:{ // Sleep BMS command
              // go to low power mode for me and the TI hardware
              break;
  } // end 0xB2

  case 0xB3:{ // Wake BMS command
              // I'm not even sure why this is here. technically the BMS should wake out of low power mode to handle the comms.
              // Maybe this just tries to wake / boot the TI hardware again
              break;
  } // end 0xB3

  // === Access Commands =====================
  case 0xA0:{ // Turn power indicator display off
              break;
  } // end 0xA0

  case 0xA1:{ // Turn power indicator display on
              break;
  } // end 0xA1

  case 0xAF:{ // FET control command
              break;
  } // end 0xAF

  case 0xAC:{ // GPIO control command
              break;
  } // end 0xAC

  // === Edit Commands ======================
  case 0xEE:{ // Edit cycle time command
              // if the next two bytes is a zero, reply with the existing cycle time command
              // else set the cycle time to the value of the next two bytes
              break;
  } // end 0xEE

  case 0xE0:{ // clear stored offline historical data command
              break;
  } // end 0xE0

  case 0xE1:{ // clear lifetime data command
              break;
  } // end 0xE1

  case 0xEA:{ // Authenticate device command
              break;
  } // end 0xEA

  case 0xEF:{ // Firmware update trigger
              // next byte dictates TI firmware or AVR firmware
              break;
  } // end 0xEF
*/
  //  === Catch-all ========================
  default:  // respond with command byte followed by 0xFF to indicate unrecognised command
            // buildpayload payload[0] concat 0xFF
            cmdbuffer[0] = incoming_cmd;  // re-write this to the buffer in case it came through corrupted, don't just reply OK!
            cmdbuffer[1] = 0xFF;
            cmdbuf_len = 2;
            //Serial.println("OK");
            break;
  
  } // end switch-case

  fresh_buffer = false;
  
}


void receiver_handler(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  
  // if packet_info.receiver_id matches bus.device_id() OR if it's a bus broadcast packet, I need to deal with it
  // else it's for someone else and I'll just flag it as invalid and abandon it
  //if (packet_info.receiver_id == MY_PJON_ADDR)
  //{
  /*if(payload[0] == 0xDD) {
    digitalWrite(13, HIGH);
    delay(30);
    digitalWrite(13, LOW);
    //bus.reply("A", 1);
  } */ 
  for(uint16_t i = 0; i < length; i++) {
    //Serial.print(
    cmdbuffer[i] = payload[i];
    // copy out the payload, flag it as valid and return to main loop to parse it
    cmdbuf_len = i;
  }
  cmdbuf_len++;
  fresh_buffer = true;
  //}
}


void error_handler(uint8_t code, uint16_t data, void *custom_pointer) {
  if(code == PJON_CONNECTION_LOST) {
    //Serial.print("Connection lost with device ");
    //Serial.println((uint8_t)bus.packets[data].content[0], DEC);
    // store what was outbound, discard what was inbound
    // re-establish connection
    // re-request inbound
    // wait for request to transmit outbound
  }
  if(code == PJON_PACKETS_BUFFER_FULL) {
    // flush it? 
  }
  if(code == PJON_CONTENT_TOO_LONG) {
    // this shouldn't happen?
  }
  if(code == PJON_ID_ACQUISITION_FAIL) {
/*    if(data == PJON_ID_ACQUIRE)
      //Serial.println("Multi-master auto addressing procedure failed."); // we arent running multi-master so dont need this
    if(data == PJON_ID_CONFIRM)
      //Serial.println("Master-slave id confirmation procedure failed."); // handle this
    if(data == PJON_ID_NEGATE)
      //Serial.println("Master-slave id release procedure failed."); // acquire fresh address
    if(data == PJON_ID_REQUEST)
      //Serial.println("Master-slave id request procedure failed."); // re-attempt request
  */}
}


bool SendBuffer(){//struct Export_Header HEADER, struct Export_Data PACKET){

  for (int x = 0; x < cmdbuf_len; x++){
    Serial.write(cmdbuffer[x]);
    //Serial.print(cmdbuffer[x], HEX);
    //Serial.print(",");
  }
  //Serial.println();
  /*Serial.print(Header.Address[9], HEX);

  Serial.print(" - ");

  Serial.print(Packet.SafetyAlert_A, HEX);
  Serial.print("+");
  Serial.print(Packet.SafetyAlert_B, HEX);
  Serial.print(",");
  Serial.print(Packet.SafetyStatus_A, HEX);
  Serial.print("+");
  Serial.print(Packet.SafetyStatus_B, HEX);
  Serial.print(",");
  Serial.print(Packet.OpStatus_A, HEX);
  Serial.print("+");
  Serial.print(Packet.OpStatus_B, HEX);
  Serial.print(",");
  Serial.print(Packet.PFStatus_A, HEX);
  Serial.print("+");
  Serial.print(Packet.PFStatus_B, HEX);
  Serial.print(",");
  Serial.print(Packet.PFAlert, HEX);
  Serial.print(",");
  Serial.print(Packet.BattMode, HEX);
  Serial.print(",");
  Serial.print(Packet.BattStatus, HEX);
  Serial.print(",");
  Serial.print(Packet.TempStatus, HEX);
  Serial.print(",");
  Serial.print(Packet.ChgStatus, HEX);
  Serial.print(",");
  Serial.print(Packet.GaugeStatus, HEX);
  Serial.print(",");
  Serial.print(Packet.MfrStatus, HEX);
  Serial.print(",");
  //Serial.print(Packet.AFEStatus, HEX);
  //Serial.print(",");

  Serial.print(Packet.HostFET, HEX);
  Serial.print(",");
  Serial.print(Packet.GPIOStatus, HEX);
  Serial.print(",");
  Serial.print(Packet.GPIOControl, HEX);
  Serial.print(",");

  Serial.write(Data.Voltage[1]);
  Serial.write(Data.Voltage[0]);
  
  //Serial.print(Packet.Voltage);
  Serial.print(",");
  Serial.print(Packet.Current);
  Serial.print(",");
  Serial.print(Packet.AvgCurrent);
  Serial.print(",");
  Serial.print(Packet.CapFull);
  Serial.print(",");
  Serial.print(Packet.CapRemain);
  Serial.print(",");
  Serial.print(Packet.RunTTE);
  Serial.print(",");
  Serial.print(Packet.AvgTTE);
  Serial.print(",");
  Serial.print(Packet.AvgTTF);
  Serial.print(",");
  Serial.print(Packet.RelSoC);
  Serial.print(",");
  Serial.print(Packet.AbsSoC);
  Serial.print(",");
  Serial.print(Packet.SoH);
  
  for(int x = 0; x < LEN_OUT_DAStatus1; x++){                             // for all possible 15 battery cells
    Serial.print(",");
    Serial.print(Packet.Cell_Volt_Data[x]);
  }
  for(int x = 0; x < LEN_OUT_DAStatus2; x++){                             // for all possible 15 battery cells
    Serial.print(",");
    Serial.print(Packet.Cell_Temp_Data[x]);
  }*/
  
  return 1;
  
}


/*bool ExportTextPacket(){

  // FIX -- Pass pointers for data variables

  if (Serial_Output_2BCO("Serial #       : ", LEN_SerialNum, SerialNum)) { // Serial #
  //if (Serial_Output_2BCO("Authenticate   : ", LEN_Authenticate, Authenticate)) { // Authenticate
  
  if (Serial_Output_2BCO("Alert Flags    : ", LEN_SafetyAlert, Flags.SafetyAlert)) { // Safety Alert
  if (Serial_Output_2BCO("Safety Status  : ", LEN_SafetyStatus, Flags.SafetyStatus)) { // Safety Status
  if (Serial_Output_2BCO("Battery Mode   : ", LEN_BattMode, Flags.BattMode)) { // Battery Mode
  if (Serial_Output_2BCO("Battery Status : ", LEN_BattStatus, Flags.BattStatus)) { // Battery Status
  if (Serial_Output_2BCO("Ops Status     : ", LEN_OpStatus, Flags.OpStatus)) { // Operational Status
  if (Serial_Output_2BCO("Chg Status     : ", LEN_ChgStatus, Flags.ChgStatus)) {  // Charge Status
  if (Serial_Output_2BCO("Gau Status     : ", LEN_GaugeStatus, Flags.GaugeStatus)) {  // Gauge Status
  if (Serial_Output_2BCO("Mfr Status     : ", LEN_MfrStatus, Flags.MfrStatus)) {  // Manufacturer Status
  if (Serial_Output_2BCO("AFE Status     : ", LEN_AFEStatus, Flags.AFEStatus)) {  // Analog Front End Status

  if (Serial_Output_2BCO("FET Controls   : ", LEN_HostFET, Flags.HostFET)) { // Host FET Control
  if (Serial_Output_2BCO("GPIO Status    : ", LEN_GPIOStatus, Flags.GPIOStatus)) { // GPIO Status
  if (Serial_Output_2BCO("GPIO Control   : ", LEN_GPIOControl, Flags.GPIOControl)) { // GPIO Control
    
  //if (Serial_Output_2BCO("Temperature    : ", LEN_Temperature, Temperature)) { // Temperature
  if (Serial_Output_2BCO("Voltage        : ", LEN_Voltage, Data.Voltage)) { // Pack Voltage
  //if (Serial_Output_2BCO("Aux Voltage    : ", LEN_VAuxVoltage, VAuxVoltage)) { // VAuxVoltage
  //if (Serial_Output_2BCO("Avg Cell Volts : ", LEN_ExtAvgVoltage, ExtAvgVoltage)) { // ExtAvgVoltage
  if (Serial_Output_2BCO("Amps           : ", LEN_Current, Data.Current)) { // Current
  if (Serial_Output_2BCO("Avg Amps       : ", LEN_AvgCurrent, Data.AvgCurrent)) { // AvgCurrent
    
  if (Serial_Output_2BCO("Relative SoC   : ", LEN_RelSoC, Data.RelSoC)) { // Relative State of Charge
  if (Serial_Output_2BCO("Absolute SoC   : ", LEN_AbsSoC, Data.AbsSoC)) { // Absolute State of Charge
  if (Serial_Output_2BCO("Capacity Left  : ", LEN_CapRemain, Data.CapRemain)) { // Remaining Capacity
  if (Serial_Output_2BCO("Capacity Full  : ", LEN_CapFull, Data.CapFull)) { // Full Capacity
  if (Serial_Output_2BCO("Runtime 2 Empty: ", LEN_RunTTE, Data.RunTTE)) { // RunTTE
  if (Serial_Output_2BCO("Avg to Empty   : ", LEN_AvgTTE, Data.AvgTTE)) { // AvgTTE
  if (Serial_Output_2BCO("Avg to Full    : ", LEN_AvgTTF, Data.AvgTTF)) { // AvgTTF
  if (Serial_Output_2BCO("Cycle Count    : ", LEN_CycleCount, Data.CycleCount)) { // Cycle Count
  if (Serial_Output_2BCO("State of Health: ", LEN_SoH, Data.SoH)) { // State of Health

  if (Serial_Output_2BCO("Cell Voltages  : ", LEN_DAStatus1, Data.Cell_Volt_Data)) { // Cell Voltages Data Block
  if (Serial_Output_2BCO("Temperatures   : ", LEN_DAStatus2, Data.Cell_Temp_Data)) { // Temperatures Data Block
  
        return 1;
  }}}}}}}}}}}}}}}}}}}}}}}}}}}//}}}}
  return 0;
}


bool ExportBinaryPacket(){

  // FIX -- Pass pointers for data variables

  if (Serial_Output_2BCO("", LEN_SerialNum, SerialNum)) { // Serial #
  //if (Serial_Output_2BCO("", LEN_Authenticate, Authenticate)) { // Authenticate
  
  if (Serial_Output_2BCO("", LEN_SafetyAlert, Flags.SafetyAlert)) { // Safety Alert
  if (Serial_Output_2BCO("", LEN_SafetyStatus, Flags.SafetyStatus)) { // Safety Status
  if (Serial_Output_2BCO("", LEN_BattMode, Flags.BattMode)) { // Battery Mode
  if (Serial_Output_2BCO("", LEN_BattStatus, Flags.BattStatus)) { // Battery Status
  if (Serial_Output_2BCO("", LEN_OpStatus, Flags.OpStatus)) { // Operational Status
  if (Serial_Output_2BCO("", LEN_ChgStatus, Flags.ChgStatus)) {  // Charge Status
  if (Serial_Output_2BCO("", LEN_GaugeStatus, Flags.GaugeStatus)) {  // Gauge Status
  if (Serial_Output_2BCO("", LEN_MfrStatus, Flags.MfrStatus)) {  // Manufacturer Status
  if (Serial_Output_2BCO("", LEN_AFEStatus, Flags.AFEStatus)) {  // Analog Front End Status

  if (Serial_Output_2BCO("", LEN_HostFET, Flags.HostFET)) { // Host FET Control
  if (Serial_Output_2BCO("", LEN_GPIOStatus, Flags.GPIOStatus)) { // GPIO Status
  if (Serial_Output_2BCO("", LEN_GPIOControl, Flags.GPIOControl)) { // GPIO Control
    
  //if (Serial_Output_2BCO("", LEN_Temperature, Temperature)) { // Temperature
  if (Serial_Output_2BCO("", LEN_Voltage, Data.Voltage)) { // Pack Voltage
  //if (Serial_Output_2BCO("", LEN_VAuxVoltage, VAuxVoltage)) { // VAuxVoltage
  //if (Serial_Output_2BCO("", LEN_ExtAvgVoltage, ExtAvgVoltage)) { // ExtAvgVoltage
  if (Serial_Output_2BCO("", LEN_Current, Data.Current)) { // Current
  if (Serial_Output_2BCO("", LEN_AvgCurrent, Data.AvgCurrent)) { // AvgCurrent
    
  if (Serial_Output_2BCO("", LEN_RelSoC, Data.RelSoC)) { // Relative State of Charge
  if (Serial_Output_2BCO("", LEN_AbsSoC, Data.AbsSoC)) { // Absolute State of Charge
  if (Serial_Output_2BCO("", LEN_CapRemain, Data.CapRemain)) { // Remaining Capacity
  if (Serial_Output_2BCO("", LEN_CapFull, Data.CapFull)) { // Full Capacity
  if (Serial_Output_2BCO("", LEN_RunTTE, Data.RunTTE)) { // RunTTE
  if (Serial_Output_2BCO("", LEN_AvgTTE, Data.AvgTTE)) { // AvgTTE
  if (Serial_Output_2BCO("", LEN_AvgTTF, Data.AvgTTF)) { // AvgTTF
  if (Serial_Output_2BCO("", LEN_CycleCount, Data.CycleCount)) { // Cycle Count
  if (Serial_Output_2BCO("", LEN_SoH, Data.SoH)) { // State of Health

  if (Serial_Output_2BCO("", LEN_DAStatus1, Data.Cell_Volt_Data)) { // Cell Voltages Data Block
  if (Serial_Output_2BCO("", LEN_DAStatus2, Data.Cell_Temp_Data)) { // Temperatures Data Block
  
        return 1;
  }}}}}}}}}}}}}}}}}}}}}}}}}}}//}}}}
  return 0;
}


bool Serial_Output_2BCO(String TITLE, int LEN, byte DATA[]){

    // Serial Output 2 Byte Corrected Order
    // For exporting data in pairs of bytes (eg. int values) from a larger array of bytes with reversed order 

    Serial.print(TITLE);

  // FIX -- Rework this as a CASE statement

    if (LEN == 1){                                  // These are usually unsigned int % numerical values eg. State of Charge 100%
      Serial.print(DATA[0], DEC);
      Serial.print(" DEC,");      
      //Serial.print(DATA[0], HEX);
      //Serial.print(" HEX,");   
      //Serial.print(DATA[0], BIN);
      //Serial.print(" BIN");   
    }
    
    if (LEN == 2){                                  // These are usually unsigned int numerical values but can also be 1x 16 bit registers (Battery Mode & Status)
      unsigned int DATA_2BCO = DATA[1]<<8;
      DATA_2BCO |= DATA[0];

      Serial.print(DATA_2BCO, DEC);
      Serial.print(" DEC,");      
      //Serial.print(DATA_2BCO, HEX);
      //Serial.print(" HEX,");   
      //Serial.print(DATA_2BCO, BIN);
      //Serial.print(" BIN");   
    }

    if (LEN == 3){                                  // These are usually HEX format 1x 16 bit registers containing status or control flags
      unsigned int DATA_2BCO = DATA[2]<<8;
      DATA_2BCO |= DATA[1];

      //Serial.print(DATA_2BCO, DEC);
      //Serial.print(" DEC,");      
      Serial.print(DATA_2BCO, HEX);
      Serial.print(" HEX,");   
      //Serial.print(DATA_2BCO, BIN);
      //Serial.print(" BIN");   
    }
        
    if (LEN == 5){                                  // These are usually HEX format 2x 16 bit registers containing status or control flags
      unsigned int UpperRegister = DATA[2]<<8;
      UpperRegister |= DATA[1];
      unsigned int LowerRegister = DATA[4]<<8;
      LowerRegister |= DATA[3];
  
      Serial.print(UpperRegister, HEX);
      Serial.print(",");
      Serial.print(LowerRegister, HEX);
      Serial.print(" HEX, ");
      //Serial.print(UpperRegister, BIN);
      //Serial.print(",");
      //Serial.print(LowerRegister, BIN);
      //Serial.print(" BIN");  
    }
    
    if (LEN > 5){                                   // These are usually blocks of data
      // Make 1x int from 2x bytes with correct LSB/MSB order and output to serial for all bytes in data block
      for(int x = 1; x < LEN-1; x=x+2){             // First byte of array seems to be occupied by a captured ACK or something so not zero indexed
        unsigned int DATA_2BCO = DATA[x];                  // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
        DATA_2BCO |= (DATA[x+1]<<8);

        Serial.print(DATA_2BCO);
        Serial.print(",");
      }
    }
        
    if (TITLE.length() > 0) {
      Serial.println();
    }

    return 1;
    
}
*/


  
/* OLD COPY-PASTA BS
// function that executes whenever data is received from TI chipset
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
*/
