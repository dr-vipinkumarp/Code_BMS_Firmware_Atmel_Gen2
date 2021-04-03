#define TI_Addr 0x0B    // Hard coded address of TI BQ78350 on SMBus / I2C
#define sample_rate 1000000
#define Atmel_FW_Version 02

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
#define LEN_IN_FirmwareVersion 13
#define LEN_OUT_FirmwareVersion 8
#define CMD_HardwareVersion 0x0003
#define LEN_IN_HardwareVersion 2
#define LEN_OUT_HardwareVersion 2
#define CMD_CHEM_ID 0x0006
#define LEN_CHEM_ID 2

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

// Control commands
#define READ_MANUFACTURER_STATUS 0x57
#define TGL_FET_ENABLE 0x22
#define CMD_FET_CONTROL 0x2B    // R/W state of FETs
#define CMD_FET_CONTROL_ACCESS 0x1197   // write enable, 0x1 = CHG, 0x2 = DCHG, 0x3 = BOTH, 0x4 = PCHG


// Firmware updates?
// #define // ROM programming mode

#define Cycle_Time 4000   // move this to permanent settings storage in EEPROM and pull it back up on startup
#define Safety_Cycles 500
#define Data_Cycles_Hi 1000
#define Data_Cycles_Lo 5000