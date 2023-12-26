/*
  ESP32_Sofar_KTLX-G3_Modbus_MQTT.h
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// modbus error code indexes
#define MODBUS_ERROR_CODE_0XE0_INDEX 0
#define MODBUS_ERROR_CODE_0XE1_INDEX 1
#define MODBUS_ERROR_CODE_0XE2_INDEX 2
#define MODBUS_ERROR_CODE_0XE3_INDEX 3

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// pv inverter Sofar KTLX-G3 register blocks
#define INVERTER_REGISTER_BLOCK_SYSINFO1_SYSTIME          -
#define INVERTER_REGISTER_BLOCK_SYSINFO1                  0
#define INVERTER_REGISTER_BLOCK_SYSINFO2                  1
#define INVERTER_REGISTER_BLOCK_GRIDOUTPUT1               2
#define INVERTER_REGISTER_BLOCK_EMERGENCYOUTPUT1          3
#define INVERTER_REGISTER_BLOCK_INPUTPV1                  4
#define INVERTER_REGISTER_BLOCK_INPUTPV2                  5
#define INVERTER_REGISTER_BLOCK_INPUTBAT1                 6
#define INVERTER_REGISTER_BLOCK_INPUTBAT2                 7
#define INVERTER_REGISTER_BLOCK_STATISTICS                8
#define INVERTER_REGISTER_BLOCK_UNBALANCEDSUPPORT_CONTROL 9
#define INVERTER_REGISTER_BLOCK_GENERAL1                  10

// pv inverter Sofar KTLX-G3 SysInfo1 [0x042C-0x0431 / 1068-1073] / register block SysTime
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_HEAD     0x0400
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_TAIL     0x043B
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_SIZE     (INVERTER_REGISTER_SYSINFO1_SYSTIME_TAIL - INVERTER_REGISTER_SYSINFO1_SYSTIME_HEAD + 1)
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_ELEMENTS 8
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_ACTIVE   1
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_YEAR     0x042C
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_MONTH    0x042D
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_DAY      0x042E
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_HOUR     0x042F
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_MINUTE   0x0430
#define INVERTER_REGISTER_SYSINFO1_SYSTIME_SECOND   0x0431

// pv inverter Sofar KTLX-G3 SysInfo1 [0x0400-0x043B / 1024-1082] / register block #0
#define INVERTER_REGISTER_SYSINFO1_HEAD                 0x0400
#define INVERTER_REGISTER_SYSINFO1_TAIL                 0x043B
#define INVERTER_REGISTER_SYSINFO1_SIZE                 (INVERTER_REGISTER_SYSINFO1_TAIL - INVERTER_REGISTER_SYSINFO1_HEAD + 1)
#define INVERTER_REGISTER_SYSINFO1_ELEMENTS             27
#define INVERTER_REGISTER_SYSINFO1_ACTIVE               1
#define INVERTER_REGISTER_SYSINFO1_SYSSTATE             0x0404
#define INVERTER_REGISTER_SYSINFO1_FAULT01              0x0405
#define INVERTER_REGISTER_SYSINFO1_FAULT02              0x0406
#define INVERTER_REGISTER_SYSINFO1_FAULT03              0x0407
#define INVERTER_REGISTER_SYSINFO1_FAULT04              0x0408
#define INVERTER_REGISTER_SYSINFO1_FAULT05              0x0409
#define INVERTER_REGISTER_SYSINFO1_FAULT06              0x040A
#define INVERTER_REGISTER_SYSINFO1_FAULT07              0x040B
#define INVERTER_REGISTER_SYSINFO1_FAULT08              0x040C
#define INVERTER_REGISTER_SYSINFO1_FAULT09              0x040D
#define INVERTER_REGISTER_SYSINFO1_FAULT10              0x040E
#define INVERTER_REGISTER_SYSINFO1_FAULT11              0x040F
#define INVERTER_REGISTER_SYSINFO1_FAULT12              0x0410
#define INVERTER_REGISTER_SYSINFO1_FAULT13              0x0411
#define INVERTER_REGISTER_SYSINFO1_FAULT14              0x0412
#define INVERTER_REGISTER_SYSINFO1_FAULT15              0x0413
#define INVERTER_REGISTER_SYSINFO1_FAULT16              0x0414
#define INVERTER_REGISTER_SYSINFO1_FAULT17              0x0415
#define INVERTER_REGISTER_SYSINFO1_FAULT18              0x0416
//#define INVERTER_REGISTER_SYSINFO1_COUNTDOWN            0x0417
#define INVERTER_REGISTER_SYSINFO1_TEMP_ENV1            0x0418
// Temperature_Env2 F16 1.0
#define INVERTER_REGISTER_SYSINFO1_TEMP_HEATSINK1       0x041A
// ...
// Temperature_HeatSink6 F16 1.0
#define INVERTER_REGISTER_SYSINFO1_TEMP_INV1            0x0420
// ...
// Temperature_Inv3 F16 1.0
// Temp_Rsvd1 F16 1.0
// ...
// Temp_Rsvd3 F16 1.0
#define INVERTER_REGISTER_SYSINFO1_GENERATION_TIME_TODAY 0x0426
#define INVERTER_REGISTER_SYSINFO1_GENERATION_TIME_TOTAL 0x0427
#define INVERTER_REGISTER_SYSINFO1_SERVICETIME_TOTAL    0x0429
// ...
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_YEAR         0x042C
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_MONTH        0x042D
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_DAY          0x042E
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_HOUR         0x042F
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_MINUTE       0x0430
//#define INVERTER_REGISTER_SYSINFO1_SYSTIME_SECOND       0x0431
// ...
// Fault19 U16
// ...
// Fault27 U16

// pv inverter Sofar KTLX-G3 SysInfo2 [0x0440-0x047F / 1088-1151] / register block #1
#define INVERTER_REGISTER_SYSINFO2_HEAD                           0x0440
#define INVERTER_REGISTER_SYSINFO2_TAIL                           0x047F
#define INVERTER_REGISTER_SYSINFO2_SIZE                           (INVERTER_REGISTER_SYSINFO2_TAIL - INVERTER_REGISTER_SYSINFO2_HEAD + 1)
#define INVERTER_REGISTER_SYSINFO2_ELEMENTS                       28
#define INVERTER_REGISTER_SYSINFO2_ACTIVE                         0
#define INVERTER_REGISTER_SYSINFO2_PRODUCTION_CODE                0x0444
// ...
#define INVERTER_REGISTER_SYSINFO2_HARDWARE_VERSION_0             0x044D
#define INVERTER_REGISTER_SYSINFO2_HARDWARE_VERSION_1             0x044E
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_COM     0x044F
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_COM     0x0450
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_COM    0x0451
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_COM     0x0452
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_MASTER  0x0453
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_MASTER  0x0454
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_MASTER 0x0455
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_MASTER  0x0456
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_SLAVE   0x0457
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_SLAVE   0x0458
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_SLAVE  0x0459
#define INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_SLAVE   0x045A
#define INVERTER_REGISTER_SYSINFO2_SAFETY_VERSION_MAJOR           0x045B
#define INVERTER_REGISTER_SYSINFO2_SAFETY_VERSION_MINOR           0x045C
#define INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_COM               0x045D
#define INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_MASTER            0x045E
#define INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_SLAVE             0x045F
#define INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_STAGE  0x0460
#define INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_MAJOR  0x0461
#define INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_CUSTOM 0x0462
#define INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_MINOR  0x0463
#define INVERTER_REGISTER_SYSINFO2_SAFETY_HARDWARE_VERSION0       0x0464
#define INVERTER_REGISTER_SYSINFO2_SAFETY_HARDWARE_VERSION1       0x0465
// ...
// State9 U16

// pv inverter Sofar KTLX-G3 GridOutput1 [0x0480-0x04BD / 1152-1213] / register block #2
#define INVERTER_REGISTER_GRIDOUTPUT1_HEAD               0x0480
#define INVERTER_REGISTER_GRIDOUTPUT1_TAIL               0x04BD
#define INVERTER_REGISTER_GRIDOUTPUT1_SIZE               (INVERTER_REGISTER_GRIDOUTPUT1_TAIL - INVERTER_REGISTER_GRIDOUTPUT1_HEAD + 1)
#define INVERTER_REGISTER_GRIDOUTPUT1_ELEMENTS           10
#define INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE             1
#define INVERTER_REGISTER_GRIDOUTPUT1_FREQUENCY_GRID     0x0484
#define INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE_POWER       0x0485
#define INVERTER_REGISTER_GRIDOUTPUT1_REACTIVE_POWER     0x0486
#define INVERTER_REGISTER_GRIDOUTPUT1_APPARENT_POWER     0x0487
#define INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE_POWER_PCC   0x0488
#define INVERTER_REGISTER_GRIDOUTPUT1_REACTIVE_POWER_PCC 0x0489
#define INVERTER_REGISTER_GRIDOUTPUT1_APPARENT_POWER_PCC 0x048A
// ...
#define INVERTER_REGISTER_GRIDOUTPUT1_POWER_FACTOR       0x04BD

// pv inverter Sofar KTLX-G3 EmergenyOutput1 [0x0500-0x0527 / 1280-1319] / register block #3
#define INVERTER_REGISTER_EMERGENCYOUTPUT1_HEAD     0x0500
#define INVERTER_REGISTER_EMERGENCYOUTPUT1_TAIL     0x0527
#define INVERTER_REGISTER_EMERGENCYOUTPUT1_SIZE     (INVERTER_REGISTER_EMERGENCYOUTPUT1_TAIL - INVERTER_REGISTER_EMERGENCYOUTPUT1_HEAD + 1)
#define INVERTER_REGISTER_EMERGENCYOUTPUT1_ELEMENTS 2
#define INVERTER_REGISTER_EMERGENCYOUTPUT1_ACTIVE   0

// pv inverter Sofar KTLX-G3 InputPV1 [0x0580-0x05B3 / 1408-1459] / register block #4
#define INVERTER_REGISTER_INPUTPV1_HEAD        0x0580
#define INVERTER_REGISTER_INPUTPV1_TAIL        0x05B3
#define INVERTER_REGISTER_INPUTPV1_SIZE        (INVERTER_REGISTER_INPUTPV1_TAIL - INVERTER_REGISTER_INPUTPV1_HEAD + 1)
#define INVERTER_REGISTER_INPUTPV1_ELEMENTS    8
#define INVERTER_REGISTER_INPUTPV1_ACTIVE      1
#define INVERTER_REGISTER_INPUTPV1_VOLTAGE_PV1 0x0584
#define INVERTER_REGISTER_INPUTPV1_CURRENT_PV1 0x0585
#define INVERTER_REGISTER_INPUTPV1_POWER_PV1   0x0586
#define INVERTER_REGISTER_INPUTPV1_VOLTAGE_PV2 0x0587
#define INVERTER_REGISTER_INPUTPV1_CURRENT_PV2 0x0588
#define INVERTER_REGISTER_INPUTPV1_POWER_PV2   0x0589
// ...
// Voltage_PV16 U16 0.1
// Current_PV16 U16 0.01
// Power_PV16 U16 0.01

// pv inverter Sofar KTLX-G3 InputPV2 [0x05C0-0x05C4 / 1472-1476] / register block #5
#define INVERTER_REGISTER_INPUTPV2_HEAD           0x05C0
#define INVERTER_REGISTER_INPUTPV2_TAIL           0x05C4
#define INVERTER_REGISTER_INPUTPV2_SIZE           (INVERTER_REGISTER_INPUTPV2_TAIL - INVERTER_REGISTER_INPUTPV2_HEAD + 1)
#define INVERTER_REGISTER_INPUTPV2_ELEMENTS       3
#define INVERTER_REGISTER_INPUTPV2_ACTIVE         0
#define INVERTER_REGISTER_INPUTPV2_POWER_PV_TOTAL 0x05C4

// pv inverter Sofar KTLX-G3 InputBat1 [0x0600-0x063B / 1536-1595] / register block #6
#define INVERTER_REGISTER_INPUTBAT1_HEAD     0x0600
#define INVERTER_REGISTER_INPUTBAT1_TAIL     0x063B
#define INVERTER_REGISTER_INPUTBAT1_SIZE     (INVERTER_REGISTER_INPUTBAT1_TAIL - INVERTER_REGISTER_INPUTBAT1_HEAD + 1)
#define INVERTER_REGISTER_INPUTBAT1_ELEMENTS 2
#define INVERTER_REGISTER_INPUTBAT1_ACTIVE   0

// pv inverter Sofar KTLX-G3 InputBat2 [0x0640-0x0669 / 1600-1641] / register block #7
#define INVERTER_REGISTER_INPUTBAT2_HEAD     0x0640
#define INVERTER_REGISTER_INPUTBAT2_TAIL     0x0669
#define INVERTER_REGISTER_INPUTBAT2_SIZE     (INVERTER_REGISTER_INPUTBAT2_TAIL - INVERTER_REGISTER_INPUTBAT2_HEAD + 1)
#define INVERTER_REGISTER_INPUTBAT2_ELEMENTS 2
#define INVERTER_REGISTER_INPUTBAT2_ACTIVE   0

// pv inverter Sofar KTLX-G3 Realtime ElectricityStatistics1 [0x0680-0x069B / 1664-1691] / register block #8
#define INVERTER_REGISTER_STATISTICS_HEAD                   0x0680
#define INVERTER_REGISTER_STATISTICS_TAIL                   0x069B
#define INVERTER_REGISTER_STATISTICS_SIZE                   (INVERTER_REGISTER_STATISTICS_TAIL - INVERTER_REGISTER_STATISTICS_HEAD + 1)
#define INVERTER_REGISTER_STATISTICS_ELEMENTS               10
#define INVERTER_REGISTER_STATISTICS_ACTIVE                 1
#define INVERTER_REGISTER_STATISTICS_PV_GENERATION_TODAY    0x0684
#define INVERTER_REGISTER_STATISTICS_PV_GENERATION_TOTAL    0x0686
#define INVERTER_REGISTER_STATISTICS_LOAD_CONSUMPTION_TODAY 0x0688
#define INVERTER_REGISTER_STATISTICS_LOAD_CONSUMPTION_TOTAL 0x068A
#define INVERTER_REGISTER_STATISTICS_ENERGY_PURCHASE_TODAY  0x068C
#define INVERTER_REGISTER_STATISTICS_ENERGY_PURCHASE_TOTAL  0x068E
#define INVERTER_REGISTER_STATISTICS_ENERGY_SELLING_TODAY   0x0690
#define INVERTER_REGISTER_STATISTICS_ENERGY_SELLING_TOTAL   0x0692
//#define INVERTER_REGISTER_STATISTICS_BAT_CHARGE_TODAY       0x0694
//#define INVERTER_REGISTER_STATISTICS_BAT_CHARGE_TOTAL       0x0696
//#define INVERTER_REGISTER_STATISTICS_BAT_DISCHARGE_TODAY    0x0698
//#define INVERTER_REGISTER_STATISTICS_BAT_DISCHARGE_TOTAL    0x069A

// pv inverter Sofar KTLX-G3 UnblancedSupportControl [0x1038-0x103D / 4152-4157] / register block #9
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_HEAD                  0x1038
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_TAIL                  0x103D
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_SIZE                  (INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_TAIL - INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_HEAD + 1)
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ELEMENTS              7
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ACTIVE                0
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_PV_GENERATION_RATIO   0x1039
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ENERGY_PURCHASE_RATIO 0x103A
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ENERGY_SELLING_RATIO  0x103B
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_BAT_CHARGE_RATIO      0x103C
#define INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_BAT_DISCHARGE_RATIO   0x103D

#define INVERTER_REGISTER_BLOCK_COUNT 10  // number of inverter register blocks

#define INVERTER_REGISTER_SYSINFO1_SYSSTATE_INDEX 0x02

// pv inverter state code (INVERTER_REGISTER_SYSINFO1_SYSSTATE)
#define INVERTER_STATE_STANDBY           0x00
#define INVERTER_STATE_SELF_CHECK        0x01
#define INVERTER_STATE_NORMAL            0x02
#define INVERTER_STATE_BATTERY_ACTIVATED 0x03
#define INVERTER_STATE_FAULT             0x04
#define INVERTER_STATE_PERMANENT_FAULT   0x05
#define INVERTER_STATE_OFFLINE           0x06

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS 7
#define SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENT_LENGTH 64

const char inverterModbusConverterMQTTSubscriptionsArray[SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS][SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENT_LENGTH] = {
  "Inverter_Modbus_Converter_Modbus_Errors_0xE0",
  "Inverter_Modbus_Converter_Modbus_Errors_0xE1",
  "Inverter_Modbus_Converter_Modbus_Errors_0xE2",
  "Inverter_Modbus_Converter_Modbus_Errors_0xE3",
  "Inverter_Modbus_Converter_MQTT_Keep_Alive",
  "Inverter_Modbus_Converter_MQTT_Keep_Alive_Received",
  "Inverter_Modbus_Converter_MQTT_Keep_Alive_Delayed"
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
struct inverterRegisterBlockStruct {
  uint16_t inverterRegisterBlockHead;
  uint16_t inverterRegisterBlockTail;
  uint16_t inverterRegisterBlockSize;
  uint16_t inverterRegisterBlockElements;
  uint16_t inverterRegisterBlockActive;
};

static struct inverterRegisterBlockStruct inverterRegisterBlockSysTime[] = {
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_HEAD, INVERTER_REGISTER_SYSINFO1_SYSTIME_TAIL, INVERTER_REGISTER_SYSINFO1_SYSTIME_SIZE, INVERTER_REGISTER_SYSINFO1_SYSTIME_ELEMENTS, INVERTER_REGISTER_SYSINFO1_SYSTIME_ACTIVE }  // #0
};

static struct inverterRegisterBlockStruct inverterRegisterBlocks[] = {
  { INVERTER_REGISTER_SYSINFO1_HEAD, INVERTER_REGISTER_SYSINFO1_TAIL, INVERTER_REGISTER_SYSINFO1_SIZE, INVERTER_REGISTER_SYSINFO1_ELEMENTS, INVERTER_REGISTER_SYSINFO1_ACTIVE },                                                                                      // #0
  { INVERTER_REGISTER_SYSINFO2_HEAD, INVERTER_REGISTER_SYSINFO2_TAIL, INVERTER_REGISTER_SYSINFO2_SIZE, INVERTER_REGISTER_SYSINFO2_ELEMENTS, INVERTER_REGISTER_SYSINFO2_ACTIVE },                                                                                      // #1
  { INVERTER_REGISTER_GRIDOUTPUT1_HEAD, INVERTER_REGISTER_GRIDOUTPUT1_TAIL, INVERTER_REGISTER_GRIDOUTPUT1_SIZE, INVERTER_REGISTER_GRIDOUTPUT1_ELEMENTS, INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE },                                                                       // #2
  { INVERTER_REGISTER_EMERGENCYOUTPUT1_HEAD, INVERTER_REGISTER_EMERGENCYOUTPUT1_TAIL, INVERTER_REGISTER_EMERGENCYOUTPUT1_SIZE, INVERTER_REGISTER_EMERGENCYOUTPUT1_ELEMENTS, INVERTER_REGISTER_EMERGENCYOUTPUT1_ACTIVE },                                              // #3
  { INVERTER_REGISTER_INPUTPV1_HEAD, INVERTER_REGISTER_INPUTPV1_TAIL, INVERTER_REGISTER_INPUTPV1_SIZE, INVERTER_REGISTER_INPUTPV1_ELEMENTS, INVERTER_REGISTER_INPUTPV1_ACTIVE },                                                                                      // #4
  { INVERTER_REGISTER_INPUTPV2_HEAD, INVERTER_REGISTER_INPUTPV2_TAIL, INVERTER_REGISTER_INPUTPV2_SIZE, INVERTER_REGISTER_INPUTPV2_ELEMENTS, INVERTER_REGISTER_INPUTPV2_ACTIVE },                                                                                      // #5
  { INVERTER_REGISTER_INPUTBAT1_HEAD, INVERTER_REGISTER_INPUTBAT1_TAIL, INVERTER_REGISTER_INPUTBAT1_SIZE, INVERTER_REGISTER_INPUTBAT1_ELEMENTS, INVERTER_REGISTER_INPUTBAT1_ACTIVE },                                                                                 // #6
  { INVERTER_REGISTER_INPUTBAT2_HEAD, INVERTER_REGISTER_INPUTBAT2_TAIL, INVERTER_REGISTER_INPUTBAT1_SIZE, INVERTER_REGISTER_INPUTBAT2_ELEMENTS, INVERTER_REGISTER_INPUTBAT2_ACTIVE },                                                                                 // #7
  { INVERTER_REGISTER_STATISTICS_HEAD, INVERTER_REGISTER_STATISTICS_TAIL, INVERTER_REGISTER_STATISTICS_SIZE, INVERTER_REGISTER_STATISTICS_ELEMENTS, INVERTER_REGISTER_STATISTICS_ACTIVE },                                                                            // #8
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_HEAD, INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_TAIL, INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_SIZE, INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ELEMENTS, INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ACTIVE }  // #9
};

struct inverterRegisterDataStruct {
	uint16_t inverterRegisterAddress;
  char *inverterRegisterDescriptor;
  char *inverterRegisterDataType;
  float inverterRegisterDataFactor;
  char *inverterRegisterUnit;
  bool inverterRegisterActiveMQTT;
};

static struct inverterRegisterDataStruct inverterRegisterSysInfo1SysTime[] = {         // register block SysTime
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_HEAD, "Block_Head", "", 0, "", 0 },             // #0
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_TAIL, "Block_Tail", "", 0, "", 0 },             // #1
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_YEAR, "SysTime_Year", "U16", 1.0, "", 0 },      // #2
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_MONTH, "SysTime_Month", "U16", 1.0, "", 0 },    // #3
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_DAY, "SysTime_Day", "U16", 1.0, "", 0 },        // #4
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_HOUR, "SysTime_Hour", "U16", 1.0, "", 0 },      // #5
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_MINUTE, "SysTime_Minute", "U16", 1.0, "", 0 },  // #6
  { INVERTER_REGISTER_SYSINFO1_SYSTIME_SECOND, "SysTime_Second", "U16", 1.0, "", 0 }   // #7
};

static struct inverterRegisterDataStruct inverterRegisterSysInfo1[] = {                                       // register block #0
  { INVERTER_REGISTER_SYSINFO1_HEAD, "Block_Head", "", 0, "", 0 },                                            // #0
  { INVERTER_REGISTER_SYSINFO1_TAIL, "Block_Tail", "", 0, "", 0 },                                            // #1
  { INVERTER_REGISTER_SYSINFO1_SYSSTATE, "SysState", "U16", 1.0, "", 1 },                                     // #2
  { INVERTER_REGISTER_SYSINFO1_FAULT01, "Fault1", "U16", 1.0, "", 0 },                                        // #3
  { INVERTER_REGISTER_SYSINFO1_FAULT02, "Fault2", "U16", 1.0, "", 0 },                                        // #4
  { INVERTER_REGISTER_SYSINFO1_FAULT03, "Fault3", "U16", 1.0, "", 0 },                                        // #5
  { INVERTER_REGISTER_SYSINFO1_FAULT04, "Fault4", "U16", 1.0, "", 0 },                                        // #6
  { INVERTER_REGISTER_SYSINFO1_FAULT05, "Fault5", "U16", 1.0, "", 0 },                                        // #7
  { INVERTER_REGISTER_SYSINFO1_FAULT06, "Fault6", "U16", 1.0, "", 0 },                                        // #8
  { INVERTER_REGISTER_SYSINFO1_FAULT07, "Fault7", "U16", 1.0, "", 0 },                                        // #9
  { INVERTER_REGISTER_SYSINFO1_FAULT08, "Fault8", "U16", 1.0, "", 0 },                                        // #10
  { INVERTER_REGISTER_SYSINFO1_FAULT09, "Fault9", "U16", 1.0, "", 0 },                                        // #11
  { INVERTER_REGISTER_SYSINFO1_FAULT10, "Fault10", "U16", 1.0, "", 0 },                                       // #12
  { INVERTER_REGISTER_SYSINFO1_FAULT11, "Fault11", "U16", 1.0, "", 0 },                                       // #13
  { INVERTER_REGISTER_SYSINFO1_FAULT12, "Fault12", "U16", 1.0, "", 0 },                                       // #14
  { INVERTER_REGISTER_SYSINFO1_FAULT13, "Fault13", "U16", 1.0, "", 0 },                                       // #15
  { INVERTER_REGISTER_SYSINFO1_FAULT14, "Fault14", "U16", 1.0, "", 0 },                                       // #16
  { INVERTER_REGISTER_SYSINFO1_FAULT15, "Fault15", "U16", 1.0, "", 0 },                                       // #17
  { INVERTER_REGISTER_SYSINFO1_FAULT16, "Fault16", "U16", 1.0, "", 0 },                                       // #18
  { INVERTER_REGISTER_SYSINFO1_FAULT17, "Fault17", "U16", 1.0, "", 0 },                                       // #19
  { INVERTER_REGISTER_SYSINFO1_FAULT18, "Fault18", "U16", 1.0, "", 0 },                                       // #20
  //{ INVERTER_REGISTER_SYSINFO1_COUNTDOWN, "Countdown", "U16", 1.0, "", 0 },                                   // #21
  { INVERTER_REGISTER_SYSINFO1_TEMP_ENV1, "Temperature_Env1", "U16", 1.0, "°C", 1 },                          // #22
  { INVERTER_REGISTER_SYSINFO1_TEMP_HEATSINK1, "Temperature_Heatsink1", "U16", 1.0, "°C", 1 },                // #23
  { INVERTER_REGISTER_SYSINFO1_TEMP_INV1, "Temperature_Inv1", "U16", 1.0, "°C", 1 },                          // #24
  { INVERTER_REGISTER_SYSINFO1_GENERATION_TIME_TODAY, "Generation_Time_Today", "F16", 0.01666667, "h", 1 },   // #25
  { INVERTER_REGISTER_SYSINFO1_GENERATION_TIME_TOTAL, "Generation_Time_Total", "F32",  0.01666667, "h", 1 },  // #26
  { INVERTER_REGISTER_SYSINFO1_SERVICETIME_TOTAL, "ServiceTime_Total", "F32", 0.01666667, "h", 1 }            // #27
};

static struct inverterRegisterDataStruct inverterRegisterSysInfo2[] = {                                               // register block #1
  { INVERTER_REGISTER_SYSINFO2_HEAD, "Block_Head", "", 0, "", 0 },                                                    // #0
  { INVERTER_REGISTER_SYSINFO2_TAIL, "Block_Tail", "", 0, "", 0 },                                                    // #1
  { INVERTER_REGISTER_SYSINFO2_PRODUCTION_CODE, "Production_Code", "U16", 1.0, "", 0 },                               // #2
  { INVERTER_REGISTER_SYSINFO2_HARDWARE_VERSION_0, "Hardware_Version_0", "CHAR", 0, "", 0 },                          // #3
  { INVERTER_REGISTER_SYSINFO2_HARDWARE_VERSION_1, "Hardware_Version_1", "CHAR", 0, "", 0 },                          // #4
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_COM, "Software_Version_Stage_COM", "CHAR", 0, "", 0 },          // #5
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_COM, "Software_Version_Major_COM", "CHAR", 0, "", 0 },          // #6
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_COM, "Software_Version_Custom_COM", "CHAR", 0, "", 0 },        // #7
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_COM, "Software_Version_Minor_COM", "CHAR", 0, "", 0 },          // #8
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_MASTER, "Software_Version_Stage_Master", "CHAR", 0, "", 0 },    // #9
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_MASTER, "Software_Version_Major_Master", "CHAR", 0, "", 0 },    // #10
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_MASTER, "Software_Version_Custom_Master", "CHAR", 0, "", 0 },  // #11
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_MASTER, "Software_Version_Minor_Master", "CHAR", 0, "", 0 },    // #12
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_STAGE_SLAVE, "Software_Version_Stage_Slave", "CHAR", 0, "", 0 },      // #13
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MAJOR_SLAVE, "Software_Version_Major_Slave", "CHAR", 0, "", 0 },      // #14
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_CUSTOM_SLAVE, "Software_Version_Custom_Slave", "CHAR", 0, "", 0 },    // #15
  { INVERTER_REGISTER_SYSINFO2_SOFTWARE_VERSION_MINOR_SLAVE, "Software_Version_Minor_Slave", "CHAR", 0, "", 0 },      // #16
  { INVERTER_REGISTER_SYSINFO2_SAFETY_VERSION_MAJOR, "Safety_Version_Major", "CHAR", 0, "", 0 },                      // #17
  { INVERTER_REGISTER_SYSINFO2_SAFETY_VERSION_MINOR, "Safety_Version_Minor", "CHAR", 0, "", 0 },                      // #18
  { INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_COM, "Boot_Version_COM", "U16", 1.0, "", 0 },                             // #19
  { INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_MASTER, "Boot_Version_Master", "U16", 1.0, "", 0 },                       // #20
  { INVERTER_REGISTER_SYSINFO2_BOOT_VERSION_SLAVE, "Boot_Version_Slave", "U16", 1.0, "", 0 },                         // #21
  { INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_STAGE, "Safety_Firmware_Version_Stage", "CHAR", 0, "", 0 },    // #22
  { INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_MAJOR, "Safety_Firmware_Version_Major", "CHAR", 0, "", 0 },    // #23
  { INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_CUSTOM, "Safety_Firmware_Version_Custom", "CHAR", 0, "", 0 },  // #24
  { INVERTER_REGISTER_SYSINFO2_SAFETY_FIRMWARE_VERSION_MINOR, "Safety_Firmware_Version_Minor", "CHAR", 0, "", 0 },    // #25
  { INVERTER_REGISTER_SYSINFO2_SAFETY_HARDWARE_VERSION0, "Safety_Hardware_Version_0", "CHAR", 0, "", 0 },             // #26
  { INVERTER_REGISTER_SYSINFO2_SAFETY_HARDWARE_VERSION1, "Safety_Hardware_Version_1", "CHAR", 0, "", 0 }              // #27
};

static struct inverterRegisterDataStruct inverterRegisterSysGridOutput1[] = {                                // register block #2
  { INVERTER_REGISTER_GRIDOUTPUT1_HEAD, "Block_Head", "", 0, "", 0 },                                        // #0
  { INVERTER_REGISTER_GRIDOUTPUT1_TAIL, "Block_Tail", "", 0, "", 0 },                                        // #1
  { INVERTER_REGISTER_GRIDOUTPUT1_FREQUENCY_GRID, "Frequency_Grid", "F16", 0.01, "Hz", 1 },                  // #2
  { INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE_POWER, "Active_Power_Output_Total", "F16", 0.01, "kW", 1 },         // #3
  { INVERTER_REGISTER_GRIDOUTPUT1_REACTIVE_POWER, "Reactive_Power_Output_Total", "F16", 0.01, "kvar", 1 },   // #4
  { INVERTER_REGISTER_GRIDOUTPUT1_APPARENT_POWER, "Apparent_Power_Output_Total", "F16", 0.01, "kVA", 1 },    // #5
  { INVERTER_REGISTER_GRIDOUTPUT1_ACTIVE_POWER_PCC, "Active_Power_PCC_Total", "F16", 0.01, "kW", 1 },        // #6
  { INVERTER_REGISTER_GRIDOUTPUT1_REACTIVE_POWER_PCC, "Reactive_Power_PCC_Total", "F16", 0.01, "kvar", 1 },  // #7
  { INVERTER_REGISTER_GRIDOUTPUT1_APPARENT_POWER_PCC, "Apparent_Power_PCC_Total", "F16", 0.01, "kVA", 1 },   // #8
  { INVERTER_REGISTER_GRIDOUTPUT1_POWER_FACTOR, "Power_Factor", "F16", 0.001, "", 1 }                        // #9
};

static struct inverterRegisterDataStruct inverterRegisterEmergencyOutput1[] = {  // register block #3
  { INVERTER_REGISTER_EMERGENCYOUTPUT1_HEAD, "Block_Head", "", 0, "", 0 },       // #0
  { INVERTER_REGISTER_EMERGENCYOUTPUT1_TAIL, "Block_Tail", "CHAR", 0, "", 0 }    // #1
};

static struct inverterRegisterDataStruct inverterRegisterInputPV1[] = {            // register block #4
  { INVERTER_REGISTER_INPUTPV1_HEAD, "Block_Head", "", 0, "", 0 },                 // #0
  { INVERTER_REGISTER_INPUTPV1_TAIL, "Block_Tail", "", 0, "", 0 },                 // #1
  { INVERTER_REGISTER_INPUTPV1_VOLTAGE_PV1, "Voltage_PV1", "F16", 0.1, "V", 1 },   // #2
  { INVERTER_REGISTER_INPUTPV1_CURRENT_PV1, "Current_PV1", "F16", 0.01, "A", 1 },  // #3
  { INVERTER_REGISTER_INPUTPV1_POWER_PV1, "Power_PV1", "F16", 0.01, "kW", 1 },     // #4
  { INVERTER_REGISTER_INPUTPV1_VOLTAGE_PV2, "Voltage_PV2", "F16", 0.1, "V", 1 },   // #5
  { INVERTER_REGISTER_INPUTPV1_CURRENT_PV2, "Current_PV2", "F16", 0.01, "A", 1 },  // #6
  { INVERTER_REGISTER_INPUTPV1_POWER_PV2, "Power_PV2", "F16", 0.01, "kW", 1 }      // #7
};

static struct inverterRegisterDataStruct inverterRegisterInputPV2[] = {                 // register block #5
  { INVERTER_REGISTER_INPUTPV2_HEAD, "Block_Head", "", 0, "", 0 },                      // #0
  { INVERTER_REGISTER_INPUTPV2_TAIL, "Block_Tail", "", 0, "", 0 },                      // #1
  { INVERTER_REGISTER_INPUTPV2_POWER_PV_TOTAL, "Power_PV_Total", "F16", 0.1, "kW", 0 }  // #2
};

static struct inverterRegisterDataStruct inverterRegisterInputBat1[] = {  // register block #6
  { INVERTER_REGISTER_INPUTBAT1_HEAD, "Block_Head", "", 0, "", 0 },       // #0
  { INVERTER_REGISTER_INPUTBAT1_TAIL, "Block_Tail", "CHAR", 0, "", 0 }    // #1
};

static struct inverterRegisterDataStruct inverterRegisterInputBat2[] = {  // register block #7
  { INVERTER_REGISTER_INPUTBAT2_HEAD, "Block_Head", "", 0, "", 0 },       // #0
  { INVERTER_REGISTER_INPUTBAT2_TAIL, "Block_Tail", "CHAR", 0, "", 0 }    // #1
};

static struct inverterRegisterDataStruct inverterRegisterElectricityStatistics1[] = {                        // register block #8
  { INVERTER_REGISTER_STATISTICS_HEAD, "Block_Head", "", 0, "", 0 },                                         // #0
  { INVERTER_REGISTER_STATISTICS_TAIL, "Block_Tail", "", 0, "", 0 },                                         // #1
  { INVERTER_REGISTER_STATISTICS_PV_GENERATION_TODAY, "PV_Generation_Today", "F32", 0.01, "kWh", 1 },        // #2
  { INVERTER_REGISTER_STATISTICS_PV_GENERATION_TOTAL, "PV_Generation_Total", "F32", 0.1, "kWh", 1 },         // #3
  { INVERTER_REGISTER_STATISTICS_LOAD_CONSUMPTION_TODAY, "Load_Consumption_Today", "F32", 0.01, "kWh", 1 },  // #4
  { INVERTER_REGISTER_STATISTICS_LOAD_CONSUMPTION_TOTAL, "Load_Consumption_Total", "F32", 0.1, "kWh", 1 },   // #5
  { INVERTER_REGISTER_STATISTICS_ENERGY_PURCHASE_TODAY, "Energy_Purchase_Today", "F32", 0.01, "kWh", 1 },    // #6
  { INVERTER_REGISTER_STATISTICS_ENERGY_PURCHASE_TOTAL, "Energy_Purchase_Total", "F32", 0.1, "kWh", 1 },     // #7
  { INVERTER_REGISTER_STATISTICS_ENERGY_SELLING_TODAY, "Energy_Selling_Today", "F32", 0.01, "kWh", 1 },      // #8
  { INVERTER_REGISTER_STATISTICS_ENERGY_SELLING_TOTAL, "Energy_Selling_Total", "F32", 0.1, "kWh", 1 }        // #9
  //{ INVERTER_REGISTER_STATISTICS_BAT_CHARGE_TODAY, "Bat_Charge_Today", "F32", 0.01, "kWh", 0 },              // #10
  //{ INVERTER_REGISTER_STATISTICS_BAT_CHARGE_TOTAL, "Bat_Charge_Total", "F32", 0.1, "kWh", 0 },               // #11
  //{ INVERTER_REGISTER_STATISTICS_BAT_DISCHARGE_TODAY, "Bat_Discharge_Today", "F32", 0.01, "kWh", 0 },        // #12
  //{ INVERTER_REGISTER_STATISTICS_BAT_DISCHARGE_TOTAL, "Bat_Discharge_Total", "F32", 0.1, "kWh", 0 }          // #13
};

static struct inverterRegisterDataStruct inverterRegisterUnbalancedSupportControl[] = {                                 // register block #9
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_HEAD, "Block_Head", "", 0, "", 0 },                                     // #0
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_TAIL, "Block_Tail", "", 0, "", 0 },                                     // #1
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_PV_GENERATION_RATIO, "PV_Generation_Ratio", "F16", 0.001, "", 0 },      // #2
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ENERGY_PURCHASE_RATIO, "Energy_Purchase_Ratio", "F16", 0.001, "", 0 },  // #3
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_ENERGY_SELLING_RATIO, "Energy_Selling_Ratio", "F16", 0.001, "", 0 },    // #4
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_BAT_CHARGE_RATIO, "Battery_Charge_Ratio", "F16", 0.001, "", 0 },        // #5
  { INVERTER_REGISTER_UNBALANCEDSUPPORT_CONTROL_BAT_DISCHARGE_RATIO, "Battery_Discharge_Ratio", "F16", 0.001, "", 0 }   // #6
};

struct inverterRegisterStateStruct {
  uint16_t inverterRegisterStateID;
  char *inverterRegisterStateDescriptor;
};

static struct inverterRegisterStateStruct inverterRegisterSysInfo1State[] = {
  { INVERTER_STATE_STANDBY, "Stand By" },
  { INVERTER_STATE_SELF_CHECK, "Self Check" },
  { INVERTER_STATE_NORMAL, "Normal" },
  { INVERTER_STATE_BATTERY_ACTIVATED, "Battery Activated" },
  { INVERTER_STATE_FAULT, "Fault" },
  { INVERTER_STATE_PERMANENT_FAULT, "Permanent Fault" },
  { INVERTER_STATE_OFFLINE, "Offline" }
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void esp32Restart();

void serialOTAReceiver(uint8_t *data, size_t length);

void mqttOutputConverterState();

void outputInfoline();

void mqttKeepAliveReceiver(String &inputTopic, String &inputPayload);

void outputInverterData(uint16_t inverterEntityResponseBufferOffset, char *inverterEntityDesriptor, char *inverterEntityDataType, float inverterEntityDataFactor, char *inverterEntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS);

bool mqttPublishTopicPayloadCHAR(String mqttTopicID, String mqttTopicEntity, char mqttPayloadCHAR, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt16(String mqttTopicID, String mqttTopicEntity, uint16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt32(String mqttTopicID, String mqttTopicEntity, uint32_t mqttPayloadUInt32, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat16(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat32(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat32, bool mqttRetained, uint16_t mqttQoS);

//EOF