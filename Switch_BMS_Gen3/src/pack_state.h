#include <Arduino.h>
#include <defines.h>
#include <ti.h>

class PackState
{
public:
    PackState(const TI &ti);

    // safety variables - priority items
    word safetyAlertAB();
    word safetyAlertCD();
    word safetyStatusAB();
    word safetyStatusCD();
    word opStatusA();
    word opStatusB();
    word pfStatusAB();
    word pfStatusCD();
    word battMode();
    word battStatus();
    byte tempStatus();
    byte chgStatus();
    word gaugeStatus();
    word mfrStatus();
    word afeStatus();

    // control variables - influence hardware operations
    word hostFET();
    word gpioStatus();
    word gpioControl();

    // data variables - for reporting
    uint16_t voltage();
    uint16_t vAuxVoltage();
    uint16_t extAvgVoltage();
    int16_t current();
    int16_t avgCurrent();
    uint16_t cycleCount();
    uint16_t capacityRemaining();
    uint16_t capacityFull();
    uint16_t runtimeToEmpty();
    uint16_t avgTimeToEmpty();
    uint16_t avgTimeToFull();

    uint8_t relativeStateOfCharge();
    uint8_t absoluteStateOfCharge();
    uint8_t stateOfHealth();

    uint16_t cellVoltage1();
    uint16_t cellVoltage2();
    uint16_t cellVoltage3();
    uint16_t cellVoltage4();
    uint16_t cellVoltage5();
    uint16_t cellVoltage6();
    uint16_t cellVoltage7();
    uint16_t cellVoltage8();
    uint16_t cellVoltage9();
    uint16_t cellVoltage10();
    uint16_t cellVoltage11();
    uint16_t cellVoltage12();
    uint16_t cellVoltage13();
    uint16_t cellVoltage14();
    uint16_t cellVoltage15();

    uint16_t ts1Temp();
    uint16_t ts2Temp();
    uint16_t ts3Temp();
    uint16_t cellTemp();
    uint16_t fetTemp();
    uint16_t internalTemp();

    private:
    TI _ti;

    byte _safetyAlert[LEN_IN_SafetyAlert];
    byte _safetyStatus[LEN_IN_SafetyStatus];
    byte _opStatus[LEN_IN_OpStatus];
    byte _pfStatus[LEN_IN_PFStatus];
    byte _battMode[LEN_IN_BattMode];
    byte _battStatus[LEN_IN_BattStatus];
    byte _chgStatus[LEN_IN_ChgStatus];
    byte _gaugeStatus[LEN_IN_GaugeStatus];
    byte _mfrStatus[LEN_IN_MfrStatus];
    byte _afeStatus[LEN_IN_AFEStatus];
    byte _hostFET[LEN_IN_HostFET];
    byte _gpioStatus[LEN_IN_GPIOStatus];
    byte _gpioControl[LEN_IN_GPIOControl];
    byte _voltage[LEN_IN_Voltage];
    byte _current[LEN_IN_Current];
    byte _avgCurrent[LEN_IN_AvgCurrent];
    byte _capRemain[LEN_IN_CapRemain];
    byte _capFull[LEN_IN_CapFull];
    byte _runTTE[LEN_IN_RunTTE];
    byte _avgTTE[LEN_IN_AvgTTE];
    byte _avgTTF[LEN_IN_AvgTTF];
    byte _cycleCount[LEN_IN_CycleCount];
    byte _relSoc[LEN_IN_RelSoC];
    byte _absSoc[LEN_IN_AbsSoC];
    byte _soh[LEN_IN_SoH];
    byte _daStatus1[LEN_IN_DAStatus1];
    byte _daStatus2[LEN_IN_DAStatus2];

    void _refreshBuffers();
};
