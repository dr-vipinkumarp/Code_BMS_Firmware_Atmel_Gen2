#include <packet.h>

char *Packet::packId()
{
    return _packId;
}

void Packet::setAvrSignature(byte *signature)
{
    _signature = signature;

    char *ptr = &_packId[0];

    for (int i = 0; i < 10; i++)
    {
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
        ptr += sprintf(ptr, "%02X", _signature[i]);
    }
}

void Packet::_clearPacket()
{
    _doc.clear();
    _doc["PackID"] = packId();
}

PacketJson &Packet::identityPacket()
{
    _clearPacket();
    return _doc;
}

PacketJson &Packet::dataPacket(PackState &packState)
{
    _clearPacket();

    _doc["FWVer"] = Atmel_FW_Version;

    _doc["SA_AB"] = packState.safetyAlertAB();
    _doc["SA_CD"] = packState.safetyAlertCD();
    _doc["SS_AB"] = packState.safetyStatusAB();
    _doc["SS_CD"] = packState.safetyStatusCD();
    _doc["OS_A"] = packState.opStatusA();
    _doc["OS_B"] = packState.opStatusB();
    _doc["PFA_AB"] = packState.pfAlertAB();
    _doc["PFA_CD"] = packState.pfAlertCD();
    _doc["PFS_AB"] = packState.pfStatusAB();
    _doc["PFS_CD"] = packState.pfStatusCD();
    _doc["BM"] = packState.battMode();
    _doc["BS"] = packState.battStatus();
    _doc["TS"] = packState.tempStatus();
    _doc["CS"] = packState.chgStatus();
    _doc["GS"] = packState.gaugeStatus();
    _doc["MS"] = packState.mfrStatus();
    _doc["AFES"] = packState.afeStatus();

    // control variables - influence hardware operations
    _doc["HFET"] = packState.hostFET();
    _doc["GPIOS"] = packState.gpioStatus();
    _doc["GPIOC"] = packState.gpioControl();

    // data variables - for reporting
    //uint16_t Temperature;
    _doc["PV"] = packState.voltage();
    _doc["VAV"] = packState.vAuxVoltage();
    _doc["EAvgV"] = packState.extAvgVoltage();
    _doc["C"] = packState.current();
    _doc["AvgC"] = packState.avgCurrent();
    _doc["Cycles"] = packState.cycleCount();
    _doc["CapRemain"] = packState.capacityRemaining();
    _doc["CapFull"] = packState.capacityFull();
    _doc["RTE"] = packState.runtimeToEmpty();
    _doc["ATTE"] = packState.avgTimeToEmpty();
    _doc["ATTF"] = packState.avgTimeToFull();

    _doc["RelSoC"] = packState.relativeStateOfCharge();
    _doc["AbsSoC"] = packState.absoluteStateOfCharge();
    _doc["SoH"] = packState.stateOfHealth();

    _doc["CV1"] = packState.cellVoltage1();
    _doc["CV2"] = packState.cellVoltage2();
    _doc["CV3"] = packState.cellVoltage3();
    _doc["CV4"] = packState.cellVoltage4();
    _doc["CV5"] = packState.cellVoltage5();
    _doc["CV6"] = packState.cellVoltage6();
    _doc["CV7"] = packState.cellVoltage7();
    _doc["CV8"] = packState.cellVoltage8();
    _doc["CV9"] = packState.cellVoltage9();
    _doc["CV10"] = packState.cellVoltage10();
    _doc["CV11"] = packState.cellVoltage11();
    _doc["CV12"] = packState.cellVoltage12();
    _doc["CV13"] = packState.cellVoltage13();
    _doc["CV14"] = packState.cellVoltage14();
    _doc["CV15"] = packState.cellVoltage15();

    _doc["TS1Temp"] = packState.ts1Temp();
    _doc["TS2Temp"] = packState.ts2Temp();
    _doc["TS3Temp"] = packState.ts3Temp();
    _doc["CellTemp"] = packState.cellTemp();
    _doc["FETTemp"] = packState.fetTemp();
    _doc["InTemp"] = packState.internalTemp();

    return _doc;
}
