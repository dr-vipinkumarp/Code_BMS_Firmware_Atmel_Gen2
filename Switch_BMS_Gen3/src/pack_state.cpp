#include <Arduino.h>
#include <Wire.h> // for TI comms
#include <defines.h>
#include <pack_state.h>

PackState::PackState(const TI &ti) : _ti(ti)
{
}

word PackState::safetyAlertAB()
{
    return (_safetyAlert[2] << 8) + _safetyAlert[1];
}

word PackState::safetyAlertCD()
{
    return (_safetyAlert[4] << 8) + _safetyAlert[3];
}

word PackState::safetyStatusAB()
{
    return (_safetyStatus[2] << 8) + _safetyStatus[1];
}

word PackState::safetyStatusCD()
{
    return (_safetyStatus[4] << 8) + _safetyStatus[3];
}

word PackState::opStatusA()
{
    return (_opStatus[2] << 8) + _opStatus[1];
}

word PackState::opStatusB()
{
    return (_opStatus[4] << 8) + _opStatus[3];
}

word PackState::pfStatusAB()
{
    return (_pfStatus[2] << 8) + _pfStatus[1];
}

word PackState::pfStatusCD()
{
    return (_pfStatus[4] << 8) + _pfStatus[3];
}

word PackState::battMode()
{
    return (_battMode[1] << 8) + _battMode[0];
}

word PackState::battStatus()
{
    return (_battStatus[1] << 8) + _battStatus[0];
}

byte PackState::tempStatus()
{
    return _chgStatus[1];
}

byte PackState::chgStatus()
{
    return _chgStatus[2];
}

word PackState::gaugeStatus()
{
    return (_gaugeStatus[2] << 8) + _gaugeStatus[1];
}

word PackState::mfrStatus()
{
    return (_mfrStatus[2] << 8) + _mfrStatus[1];
}

word PackState::afeStatus()
{
    return (_afeStatus[2] << 8) + _afeStatus[1];
}

// control variables - influence hardware operations
word PackState::hostFET()
{
    return (_hostFET[1] << 8) + _hostFET[0];
}

word PackState::gpioStatus()
{
    return (_gpioStatus[1] << 8) + _gpioStatus[0];
}

word PackState::gpioControl()
{
    return (_gpioControl[1] << 8) + _gpioControl[0];
}

// data variables - for reporting
uint16_t PackState::voltage()
{
    return (_voltage[1] << 8) + _voltage[0];
}

uint16_t PackState::vAuxVoltage()
{
    return (_daStatus2[4] << 8) + _daStatus2[3];
}

uint16_t PackState::extAvgVoltage()
{
    return (_daStatus2[2] << 8) + _daStatus2[1];
}

int16_t PackState::current()
{
    return (_current[1] << 8) + _current[0];
}

int16_t PackState::avgCurrent()
{
    return (_avgCurrent[1] << 8) + _avgCurrent[0];
}

uint16_t PackState::cycleCount()
{
    return (_cycleCount[1] << 8) + _cycleCount[0];
}

uint16_t PackState::capacityRemaining()
{
    return (_capRemain[1] << 8) + _capRemain[0];
}

uint16_t PackState::capacityFull()
{
    return (_capFull[1] << 8) + _capFull[0];
}

uint16_t PackState::runtimeToEmpty()
{
    return (_runTTE[1] << 8) + _runTTE[0];
}

uint16_t PackState::avgTimeToEmpty()
{
    return (_avgTTE[1] << 8) + _avgTTE[0];
}

uint16_t PackState::avgTimeToFull()
{
    return (_avgTTF[1] << 8) + _avgTTF[0];
}

uint8_t PackState::relativeStateOfCharge()
{
    return _relSoc[1];
}

uint8_t PackState::absoluteStateOfCharge()
{
    return _absSoc[1];
}

uint8_t PackState::stateOfHealth()
{
    return _soh[1];
}

uint16_t PackState::cellVoltage1()
{
    return (_daStatus1[2] << 8) + _daStatus1[1];
}

uint16_t PackState::cellVoltage2()
{
    return (_daStatus1[4] << 8) + _daStatus1[3];
}

uint16_t PackState::cellVoltage3()
{
    return (_daStatus1[6] << 8) + _daStatus1[5];
}

uint16_t PackState::cellVoltage4()
{
    return (_daStatus1[8] << 8) + _daStatus1[7];
}

uint16_t PackState::cellVoltage5()
{
    return (_daStatus1[10] << 8) + _daStatus1[9];
}

uint16_t PackState::cellVoltage6()
{
    return (_daStatus1[12] << 8) + _daStatus1[11];
}

uint16_t PackState::cellVoltage7()
{
    return (_daStatus1[14] << 8) + _daStatus1[13];
}

uint16_t PackState::cellVoltage8()
{
    return (_daStatus1[16] << 8) + _daStatus1[15];
}

uint16_t PackState::cellVoltage9()
{
    return (_daStatus1[18] << 8) + _daStatus1[17];
}

uint16_t PackState::cellVoltage10()
{
    return (_daStatus1[20] << 8) + _daStatus1[19];
}

uint16_t PackState::cellVoltage11()
{
    return (_daStatus1[22] << 8) + _daStatus1[21];
}

uint16_t PackState::cellVoltage12()
{
    return (_daStatus1[24] << 8) + _daStatus1[23];
}

uint16_t PackState::cellVoltage13()
{
    return (_daStatus1[26] << 8) + _daStatus1[25];
}

uint16_t PackState::cellVoltage14()
{
    return (_daStatus1[28] << 8) + _daStatus1[27];
}

uint16_t PackState::cellVoltage15()
{
    return (_daStatus1[30] << 8) + _daStatus1[29];
}

uint16_t PackState::ts1Temp()
{
    return (_daStatus2[6] << 8) + _daStatus2[5];
}

uint16_t PackState::ts2Temp()
{
    return (_daStatus2[8] << 8) + _daStatus2[7];
}

uint16_t PackState::ts3Temp()
{
    return (_daStatus2[10] << 8) + _daStatus2[9];
}

uint16_t PackState::cellTemp()
{
    return (_daStatus2[12] << 8) + _daStatus2[11];
}

uint16_t PackState::fetTemp()
{
    return (_daStatus2[14] << 8) + _daStatus2[13];
}

uint16_t PackState::internalTemp()
{
    return (_daStatus2[16] << 8) + _daStatus2[15];
}

void PackState::refresh()
{
    _ti.read(CMD_SafetyAlert, LEN_IN_SafetyAlert, _safetyAlert);
    _ti.read(CMD_SafetyStatus, LEN_IN_SafetyStatus, _safetyStatus);
    _ti.read(CMD_OpStatus, LEN_IN_OpStatus, _opStatus);
    _ti.read(CMD_PFStatus, LEN_IN_PFStatus, _pfStatus);
    _ti.read(CMD_BattMode, LEN_IN_BattMode, _battMode);
    _ti.read(CMD_BattStatus, LEN_IN_BattStatus, _battStatus);
    _ti.read(CMD_ChgStatus, LEN_IN_ChgStatus, _chgStatus);
    _ti.read(CMD_GaugeStatus, LEN_IN_GaugeStatus, _gaugeStatus);
    _ti.read(CMD_MfrStatus, LEN_IN_MfrStatus, _mfrStatus);
    _ti.read(CMD_AFEStatus, LEN_IN_AFEStatus, _afeStatus);
    _ti.read(CMD_HostFET, LEN_IN_HostFET, _hostFET);
    _ti.read(CMD_GPIOStatus, LEN_IN_GPIOStatus, _gpioStatus);
    _ti.read(CMD_GPIOControl, LEN_IN_GPIOControl, _gpioControl);
    _ti.read(CMD_Voltage, LEN_IN_Voltage, _voltage);
    _ti.read(CMD_Current, LEN_IN_Current, _current);
    _ti.read(CMD_AvgCurrent, LEN_IN_AvgCurrent, _avgCurrent);
    _ti.read(CMD_CapRemain, LEN_IN_CapRemain, _capRemain);
    _ti.read(CMD_CapFull, LEN_IN_CapFull, _capFull);
    _ti.read(CMD_RunTTE, LEN_IN_RunTTE, _runTTE);
    _ti.read(CMD_AvgTTE, LEN_IN_AvgTTE, _avgTTE);
    _ti.read(CMD_AvgTTF, LEN_IN_AvgTTF, _avgTTF);
    _ti.read(CMD_CycleCount, LEN_IN_CycleCount, _cycleCount);
    _ti.read(CMD_RelSoC, LEN_IN_RelSoC, _relSoc);
    _ti.read(CMD_AbsSoC, LEN_IN_AbsSoC, _absSoc);
    _ti.read(CMD_SoH, LEN_IN_SoH, _soh);
    _ti.read(CMD_DAStatus1, LEN_IN_DAStatus1, _daStatus1);
    _ti.read(CMD_DAStatus2, LEN_IN_DAStatus2, _daStatus2);
}
