#pragma once
#include <Arduino.h>
#include <defines.h>

class TI
{
public:
    TI(int addr);

    void read(byte CMD, int LEN, byte IN_DATA[]);
    void write(byte CMD, int LEN, byte OUT_DATA[]);
    void blockRead(byte CMD[], int LEN, byte IN_DATA[]);
    void fetOn();
    void fetOff();

    word devType();
    word devNum();
    word firmwareVersion();
    word chemistryId();

private:
    static const byte _cmdExecute[];
    static const byte _cmdOn[];
    static const byte _cmdOff[];

    int _addr;

    byte _devType[LEN_IN_DeviceType];
    byte _firmwareVersion[LEN_IN_FirmwareVersion];
    byte _chemistryId[LEN_CHEM_ID];

    void _pullVersionData();
};