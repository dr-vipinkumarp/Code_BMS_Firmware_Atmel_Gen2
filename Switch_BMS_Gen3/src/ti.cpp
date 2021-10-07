#include <Wire.h>

#include <ti.h>

TI::TI(int addr) : _addr(addr)
{
    // _pullVersionData();
}

void TI::read(byte CMD, int LEN, byte IN_DATA[])
{ // FIX -- Pass pointers for data variable

    Wire.beginTransmission(_addr);
    Wire.write(CMD);

    // FIX -- if LEN not zero then send follow-up commands to prompt TI to spew data, else command was a one-shot with a predictable response
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, LEN, true); // first byte through will usually be a "LENGTH" value

    for (int i = 0; i < LEN; i++)
    {                             // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
        IN_DATA[i] = Wire.read(); // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
    // else
    // Wire.endTransmission(true);
    // FIX -- read whatever was predictable about a CMD with a zero length reply
}

void TI::write(byte CMD, int LEN, byte OUT_DATA[])
{ // FIX -- Pass pointers for data variable

    Wire.beginTransmission(_addr);
    Wire.write(CMD);

    // FIX -- if LEN not zero then send follow-up commands to prompt TI to spew data, else command was a one-shot with a predictable response
    Wire.endTransmission(false);
    //Wire.requestFrom(_addr, LEN, true);
    //Wire.write(LEN);  // not sure if this is needed, have to eavesdrop comms to see if this helps or breaks

    // do these need to be reversed?
    for (int i = 0; i < LEN; i++)
    {                            // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
        Wire.write(OUT_DATA[i]); // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
    // else
    Wire.endTransmission(false);
    // FIX -- read whatever was predictable about a CMD with a zero length reply
}

void TI::blockRead(byte CMD[], int LEN, byte IN_DATA[])
{

    Wire.beginTransmission(_addr);
    Wire.write(0x00); // manufacturer access
    Wire.write(0x02);
    Wire.write(0x00); // because reversed
    Wire.write(0x44); // read block
    Wire.endTransmission(false);

    //LEN = Wire.read();  // first byte probably length (try it)
    Wire.requestFrom(_addr, LEN, true);

    for (int i = 0; i < LEN; i++)
    {                             // Array is zero indexed but first byte read seems to be an ACK before real data on block reads
        IN_DATA[i] = Wire.read(); // FIX -- Handle for exception if variable passed as DATA is insufficient length for value passed to LEN
    }
    //Wire.endTransmission(true);
}

void TI::_pullVersionData()
{
    // TODO: Fix up
    read(CMD_DeviceType, LEN_IN_DeviceType, _devType);
    blockRead(CMD_FirmwareVersion, LEN_IN_FirmwareVersion, _firmwareVersion);
    read(CMD_CHEM_ID, LEN_CHEM_ID, _chemistryId);
}

const byte TI::_cmdExecute[] = {0x2B, 0x97, 0x11};
const byte TI::_cmdOn[] = {0x2B, 0x03, 0x00};
const byte TI::_cmdOff[] = {0x2B, 0x01, 0x00};

void TI::fetOn()
{
  Wire.beginTransmission(_addr);
  Wire.write(_cmdExecute, 3);
  Wire.endTransmission(true);
  Wire.beginTransmission(_addr);
  Wire.write(_cmdOn, 3);
  Wire.endTransmission(true);
}

void TI::fetOff()
{
  Wire.beginTransmission(_addr);
  Wire.write(_cmdExecute, 3);
  Wire.endTransmission(true);
  Wire.beginTransmission(_addr);
  Wire.write(_cmdOff, 3);
  Wire.endTransmission(true);
}

word TI::devType()
{
    return (_devType[1] << 8) + _devType[2];
}

word TI::devNum()
{
    return (_firmwareVersion[2] << 8) + _firmwareVersion[1];
}

word TI::firmwareVersion()
{
    return (_firmwareVersion[4] << 8) + _firmwareVersion[3];
}

word TI::chemistryId()
{
    return (_chemistryId[2] << 8) + _chemistryId[1];
}
