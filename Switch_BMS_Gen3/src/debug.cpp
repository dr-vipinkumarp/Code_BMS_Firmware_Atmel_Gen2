#include <Arduino.h>
#include <debug.h>

void printDebug(PackState &packState, TI &ti) {
    Serial.println();

    Serial.print(F("FWVer: \t\t"));
    Serial.println(Atmel_FW_Version);

    Serial.print(F("Device Type: \t\t"));
    Serial.println(ti.devType(), HEX);
    Serial.print(F("Device Number: \t\t"));
    Serial.println(ti.devNum(), HEX);
    Serial.print(F("TI FW Version: \t\t"));
    Serial.println(ti.firmwareVersion(), HEX);

    Serial.print(F("SA_AB: \t\t"));
    Serial.println(packState.safetyAlertAB(), HEX);
    Serial.print(F("SA_CD: \t\t"));
    Serial.println(packState.safetyAlertCD(), HEX);
    Serial.print(F("SS_AB: \t\t"));
    Serial.println(packState.safetyStatusAB(), HEX);
    Serial.print(F("SS_CD: \t\t"));
    Serial.println(packState.safetyStatusCD(), HEX);
    Serial.print(F("OS_A: \t\t"));
    Serial.println(packState.opStatusA(), HEX);
    Serial.print(F("OS_B: \t\t"));
    Serial.println(packState.opStatusB(), HEX);
    Serial.print(F("PFA_AB: \t\t"));
    Serial.println(packState.pfAlertAB(), HEX);
    Serial.print(F("PFA_CD: \t\t"));
    Serial.println(packState.pfAlertCD(), HEX);
    Serial.print(F("PFS_AB: \t\t"));
    Serial.println(packState.pfStatusAB(), HEX);
    Serial.print(F("PFS_CD: \t\t"));
    Serial.println(packState.pfStatusCD(), HEX);
    Serial.print(F("BM: \t\t"));
    Serial.println(packState.battMode(), HEX);
    Serial.print(F("BS: \t\t"));
    Serial.println(packState.battStatus(), HEX);
    Serial.print(F("TS: \t\t"));
    Serial.println(packState.tempStatus(), HEX);
    Serial.print(F("CS: \t\t"));
    Serial.println(packState.chgStatus(), HEX);
    Serial.print(F("GS: \t\t"));
    Serial.println(packState.gaugeStatus(), HEX);
    Serial.print(F("MS: \t\t"));
    Serial.println(packState.mfrStatus(), HEX);
    Serial.print(F("AFES: \t\t"));
    Serial.println(packState.afeStatus(), HEX);

    // control variables - influence hardware operations
    Serial.print(F("HFET: \t\t"));
    Serial.println(packState.hostFET(), HEX);
    Serial.print(F("GPIOS: \t\t"));
    Serial.println(packState.gpioStatus(), HEX);
    Serial.print(F("GPIOC: \t\t"));
    Serial.println(packState.gpioControl(), HEX);

    // data variables - for reporting
    //uint16_t Temperature;
    Serial.print(F("PV: \t\t"));
    Serial.println(packState.voltage());
    Serial.print(F("VAV: \t\t"));
    Serial.println(packState.vAuxVoltage());
    Serial.print(F("EAvgV: \t\t"));
    Serial.println(packState.extAvgVoltage());
    Serial.print(F("C: \t\t"));
    Serial.println(packState.current());
    Serial.print(F("AvgC: \t\t"));
    Serial.println(packState.avgCurrent());
    Serial.print(F("Cycles: \t\t"));
    Serial.println(packState.cycleCount());
    Serial.print(F("CapRemain: \t\t"));
    Serial.println(packState.capacityRemaining());
    Serial.print(F("CapFull: \t\t"));
    Serial.println(packState.capacityFull());
    Serial.print(F("RTE: \t\t"));
    Serial.println(packState.runtimeToEmpty());
    Serial.print(F("ATTE: \t\t"));
    Serial.println(packState.avgTimeToEmpty());
    Serial.print(F("ATTF: \t\t"));
    Serial.println(packState.avgTimeToFull());

    Serial.print(F("RelSoC: \t\t"));
    Serial.println(packState.relativeStateOfCharge());
    Serial.print(F("AbsSoC: \t\t"));
    Serial.println(packState.absoluteStateOfCharge());
    Serial.print(F("SoH: \t\t"));
    Serial.println(packState.stateOfHealth());

    Serial.print(F("CV1: \t\t"));
    Serial.println(packState.cellVoltage1());
    Serial.print(F("CV2: \t\t"));
    Serial.println(packState.cellVoltage2());
    Serial.print(F("CV3: \t\t"));
    Serial.println(packState.cellVoltage3());
    Serial.print(F("CV4: \t\t"));
    Serial.println(packState.cellVoltage4());
    Serial.print(F("CV5: \t\t"));
    Serial.println(packState.cellVoltage5());
    Serial.print(F("CV6: \t\t"));
    Serial.println(packState.cellVoltage6());
    Serial.print(F("CV7: \t\t"));
    Serial.println(packState.cellVoltage7());
    Serial.print(F("CV8: \t\t"));
    Serial.println(packState.cellVoltage8());
    Serial.print(F("CV9: \t\t"));
    Serial.println(packState.cellVoltage9());
    Serial.print(F("CV10: \t\t"));
    Serial.println(packState.cellVoltage10());
    Serial.print(F("CV11: \t\t"));
    Serial.println(packState.cellVoltage11());
    Serial.print(F("CV12: \t\t"));
    Serial.println(packState.cellVoltage12());
    Serial.print(F("CV13: \t\t"));
    Serial.println(packState.cellVoltage13());
    Serial.print(F("CV14: \t\t"));
    Serial.println(packState.cellVoltage14());
    Serial.print(F("CV15: \t\t"));
    Serial.println(packState.cellVoltage15());

    Serial.print(F("TS1Temp: \t\t"));
    Serial.println(packState.ts1Temp());
    Serial.print(F("TS2Temp: \t\t"));
    Serial.println(packState.ts2Temp());
    Serial.print(F("TS3Temp: \t\t"));
    Serial.println(packState.ts3Temp());
    Serial.print(F("CellTemp: \t\t"));
    Serial.println(packState.cellTemp());
    Serial.print(F("FETTemp: \t\t"));
    Serial.println(packState.fetTemp());
    Serial.print(F("InTemp: \t\t"));
    Serial.println(packState.internalTemp());
}