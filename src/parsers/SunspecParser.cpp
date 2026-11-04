#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"

#define SUNSPEC_scale(n) (abs(n) < 10 ? pow(10.0, (double)n) : 1.0)

int32_t SUNSPEC_BASE = 40073;

int32_t SUNSPEC_VOLTAGE_SCALE = 40085;
int32_t SUNSPEC_REAL_POWER_SCALE = 40092;
int32_t SUNSPEC_APPARANT_POWER_SCALE = 40097;
int32_t SUNSPEC_CURRENT_SCALE = 40076;
int32_t SUNSPEC_POWER_FACTOR_SCALE = 40107;
int32_t SUNSPEC_FREQUENCY_SCALE = 40087;
int32_t SUNSPEC_REAL_ENERGY_SCALE = 40124;

int32_t SUNSPEC_FREQUENCY = 40086;

int32_t SUNSPEC_CURRENT0 = 40073;
int32_t SUNSPEC_CURRENT1 = 40074;
int32_t SUNSPEC_CURRENT2 = 40075;

int32_t SUNSPEC_VOLTAGE0 = 40078;
int32_t SUNSPEC_VOLTAGE1 = 40079;
int32_t SUNSPEC_VOLTAGE2 = 40080;

int32_t SUNSPEC_REAL_POWER0 = 40089;
int32_t SUNSPEC_REAL_POWER1 = 40090;
int32_t SUNSPEC_REAL_POWER2 = 40091;

int32_t SUNSPEC_APPARANT_POWER0 = 40094;
int32_t SUNSPEC_APPARANT_POWER1 = 40095;
int32_t SUNSPEC_APPARANT_POWER2 = 40096;

int32_t SUNSPEC_POWER_FACTOR0 = 40104;
int32_t SUNSPEC_POWER_FACTOR1 = 40105;
int32_t SUNSPEC_POWER_FACTOR2 = 40106;

int32_t SUNSPEC_REAL_ENERGY_EXPORTED0 = 40110;
int32_t SUNSPEC_REAL_ENERGY_EXPORTED1 = 40112;
int32_t SUNSPEC_REAL_ENERGY_EXPORTED2 = 40114;

int32_t SUNSPEC_REAL_ENERGY_IMPORTED0 = 40118;
int32_t SUNSPEC_REAL_ENERGY_IMPORTED1 = 40120;
int32_t SUNSPEC_REAL_ENERGY_IMPORTED2 = 40122;

int32_t SUNSPEC_REGISTER_COUNT = 64;

enum ModbusType {
  TYPE_UNKNOWN,
  TYPE_INT16,
  TYPE_UINT16,
  TYPE_INT32,
  TYPE_UINT32,
  TYPE_FLOAT32
};

ModbusType get_modbus_type(const String& type) {
  if (type == "int16")   return TYPE_INT16;
  if (type == "uint16")  return TYPE_UINT16;
  if (type == "int32")   return TYPE_INT32;
  if (type == "uint32")  return TYPE_UINT32;
  if (type == "float32") return TYPE_FLOAT32;
  return TYPE_UNKNOWN;
}


ModbusType SUNSPEC_VOLTAGE_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_REAL_POWER_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_APPARANT_POWER_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_CURRENT_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_POWER_FACTOR_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_FREQUENCY_TYPE=TYPE_UNKNOWN;
ModbusType SUNSPEC_REAL_ENERGY_TYPE=TYPE_UNKNOWN;


    #define SUNSPEC_VOLTAGE_SCALE_INT config_doc["sunspec_registers"]["voltage_scale"].as<int>()
    #define SUNSPEC_VOLTAGE_SCALE_NULL config_doc["sunspec_registers"]["voltage_scale"].isNull()
    #define SUNSPEC_FREQUENCY_SCALE_INT config_doc["sunspec_registers"]["frequency_scale"].as<int>()
    #define SUNSPEC_FREQUENCY_SCALE_NULL config_doc["sunspec_registers"]["frequency_scale"].isNull()
    #define SUNSPEC_REAL_ENERGY_SCALE_INT config_doc["sunspec_registers"]["real_energy_scale"].as<int>()
    #define SUNSPEC_REAL_ENERGY_SCALE_NULL config_doc["sunspec_registers"]["real_energy_scale"].isNull()
    #define SUNSPEC_REAL_POWER_SCALE_INT config_doc["sunspec_registers"]["real_power_scale"].as<int>()
    #define SUNSPEC_REAL_POWER_SCALE_NULL config_doc["sunspec_registers"]["real_power_scale"].isNull()  
    #define SUNSPEC_APPARANT_POWER_SCALE_INT config_doc["sunspec_registers"]["apparent_power_scale"].as<int>()
    #define SUNSPEC_APPARANT_POWER_SCALE_NULL config_doc["sunspec_registers"]["apparent_power_scale"].isNull()
    #define SUNSPEC_REAL_POWER_SCALE_INT config_doc["sunspec_registers"]["real_power_scale"].as<int>()
    #define SUNSPEC_CURRENT_SCALE_INT config_doc["sunspec_registers"]["current_scale"].as<int>()
    #define SUNSPEC_CURRENT_SCALE_NULL config_doc["sunspec_registers"]["current_scale"].isNull()
    #define SUNSPEC_POWER_FACTOR_SCALE_INT config_doc["sunspec_registers"]["power_factor_scale"].as<int>()
    #define SUNSPEC_POWER_FACTOR_SCALE_NULL config_doc["sunspec_registers"]["power_factor_scale"].isNull()
    #define SUNSPEC_REAL_POWER0_INT config_doc["sunspec_registers"]["real_power0"].as<int>()
    #define SUNSPEC_REAL_POWER0_NULL config_doc["sunspec_registers"]["real_power0"].isNull()
    #define SUNSPEC_REAL_POWER1_INT config_doc["sunspec_registers"]["real_power1"].as<int>()
    #define SUNSPEC_REAL_POWER1_NULL config_doc["sunspec_registers"]["real_power1"].isNull()
    #define SUNSPEC_REAL_POWER2_INT config_doc["sunspec_registers"]["real_power2"].as<int>()
    #define SUNSPEC_REAL_POWER2_NULL config_doc["sunspec_registers"]["real_power2"].isNull()
    #define SUNSPEC_APPARANT_POWER0_INT config_doc["sunspec_registers"]["apparent_power0"].as<int>()
    #define SUNSPEC_APPARANT_POWER0_NULL config_doc["sunspec_registers"]["apparent_power0"].isNull()
    #define SUNSPEC_APPARANT_POWER1_INT config_doc["sunspec_registers"]["apparent_power1"].as<int>()
    #define SUNSPEC_APPARANT_POWER1_NULL config_doc["sunspec_registers"]["apparent_power1"].isNull()
    #define SUNSPEC_APPARANT_POWER2_INT config_doc["sunspec_registers"]["apparent_power2"].as<int>()
    #define SUNSPEC_APPARANT_POWER2_NULL config_doc["sunspec_registers"]["apparent_power2"].isNull()
    #define SUNSPEC_CURRENT0_NULL config_doc["sunspec_registers"]["current0"].isNull()
    #define SUNSPEC_CURRENT0_INT config_doc["sunspec_registers"]["current0"].as<int>()
    #define SUNSPEC_CURRENT1_NULL config_doc["sunspec_registers"]["current1"].isNull()
    #define SUNSPEC_CURRENT1_INT config_doc["sunspec_registers"]["current1"].as<int>()
    #define SUNSPEC_CURRENT2_NULL config_doc["sunspec_registers"]["current2"].isNull()
    #define SUNSPEC_CURRENT2_INT config_doc["sunspec_registers"]["current2"].as<int>()
    #define SUNSPEC_VOLTAGE0_NULL config_doc["sunspec_registers"]["voltage0"].isNull()
    #define SUNSPEC_VOLTAGE0_INT config_doc["sunspec_registers"]["voltage0"].as<int>()
    #define SUNSPEC_VOLTAGE1_NULL config_doc["sunspec_registers"]["voltage1"].isNull()
    #define SUNSPEC_VOLTAGE1_INT config_doc["sunspec_registers"]["voltage1"].as<int>()
    #define SUNSPEC_VOLTAGE2_NULL config_doc["sunspec_registers"]["voltage2"].isNull()
    #define SUNSPEC_VOLTAGE2_INT config_doc["sunspec_registers"]["voltage2"].as<int>()
    #define SUNSPEC_POWER_FACTOR0_NULL config_doc["sunspec_registers"]["power_factor0"].isNull()
    #define SUNSPEC_POWER_FACTOR0_INT config_doc["sunspec_registers"]["power_factor0"].as<int>()
    #define SUNSPEC_POWER_FACTOR1_NULL config_doc["sunspec_registers"]["power_factor1"].isNull()
    #define SUNSPEC_POWER_FACTOR1_INT config_doc["sunspec_registers"]["power_factor1"].as<int>()
    #define SUNSPEC_POWER_FACTOR2_NULL config_doc["sunspec_registers"]["power_factor2"].isNull()
    #define SUNSPEC_POWER_FACTOR2_INT config_doc["sunspec_registers"]["power_factor2"].as<int>()
    #define SUNSPEC_FREQUENCY_NULL config_doc["sunspec_registers"]["frequency"].isNull()
    #define SUNSPEC_FREQUENCY_INT config_doc["sunspec_registers"]["frequency"].as<int>()
    #define SUNSPEC_REAL_ENERGY_IMPORTED0_NULL config_doc["sunspec_registers"]["real_energy_imported0"].isNull()
    #define SUNSPEC_REAL_ENERGY_IMPORTED0_INT config_doc["sunspec_registers"]["real_energy_imported0"].as<int>()
    #define SUNSPEC_REAL_ENERGY_IMPORTED1_NULL config_doc["sunspec_registers"]["real_energy_imported1"].isNull()
    #define SUNSPEC_REAL_ENERGY_IMPORTED1_INT config_doc["sunspec_registers"]["real_energy_imported1"].as<int>()
    #define SUNSPEC_REAL_ENERGY_IMPORTED2_NULL config_doc["sunspec_registers"]["real_energy_imported2"].isNull()
    #define SUNSPEC_REAL_ENERGY_IMPORTED2_INT config_doc["sunspec_registers"]["real_energy_imported2"].as<int>() 
    #define SUNSPEC_REAL_ENERGY_EXPORTED0_NULL config_doc["sunspec_registers"]["real_energy_exported0"].isNull()
    #define SUNSPEC_REAL_ENERGY_EXPORTED0_INT config_doc["sunspec_registers"]["real_energy_exported0"].as<int>()
    #define SUNSPEC_REAL_ENERGY_EXPORTED1_NULL config_doc["sunspec_registers"]["real_energy_exported1"].isNull()
    #define SUNSPEC_REAL_ENERGY_EXPORTED1_INT config_doc["sunspec_registers"]["real_energy_exported1"].as<int>()
    #define SUNSPEC_REAL_ENERGY_EXPORTED2_NULL config_doc["sunspec_registers"]["real_energy_exported2"].isNull()
    #define SUNSPEC_REAL_ENERGY_EXPORTED2_INT config_doc["sunspec_registers"]["real_energy_exported2"].as<int>()
    #define SUNSPEC_REGISTER_COUNT_INT config_doc["sunspec_registers"]["register_count"].as<int>()
    #define SUNSPEC_REGISTER_COUNT_NULL config_doc["sunspec_registers"]["register_count"].isNull()
    #define SUNSPEC_BASE_NULL config_doc["sunspec_registers"]["base"].isNull()
    #define SUNSPEC_BASE_INT config_doc["sunspec_registers"]["base"].as<int>()


void convert_json_to_sunspec_registers()
{
  if (!config_doc["sunspec"].isNull()) {
  modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
  sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();
  modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"].as<const char*>());
  }
  if (!config_doc["sunspec_registers"].isNull()) {
  SUNSPEC_BASE = SUNSPEC_BASE_NULL? -1 : SUNSPEC_BASE_INT;
  SUNSPEC_REGISTER_COUNT = SUNSPEC_REGISTER_COUNT_NULL? -1 : SUNSPEC_REGISTER_COUNT_INT;
  SUNSPEC_VOLTAGE_SCALE = SUNSPEC_VOLTAGE_SCALE_NULL? -1 : SUNSPEC_VOLTAGE_SCALE_INT;
  SUNSPEC_REAL_POWER_SCALE = SUNSPEC_REAL_POWER_SCALE_NULL? -1 : SUNSPEC_REAL_POWER_SCALE_INT;
  SUNSPEC_APPARANT_POWER_SCALE = SUNSPEC_APPARANT_POWER_SCALE_NULL? -1 : SUNSPEC_APPARANT_POWER_SCALE_INT;
  SUNSPEC_CURRENT_SCALE = SUNSPEC_CURRENT_SCALE_NULL? -1 : SUNSPEC_CURRENT_SCALE_INT;
  SUNSPEC_POWER_FACTOR_SCALE = SUNSPEC_POWER_FACTOR_SCALE_NULL? -1 : SUNSPEC_POWER_FACTOR_SCALE_INT;
  SUNSPEC_FREQUENCY_SCALE = SUNSPEC_FREQUENCY_SCALE_NULL? -1 : SUNSPEC_FREQUENCY_SCALE_INT;
  SUNSPEC_REAL_ENERGY_SCALE = SUNSPEC_REAL_ENERGY_SCALE_NULL? -1 : SUNSPEC_REAL_ENERGY_SCALE_INT;
  SUNSPEC_FREQUENCY = SUNSPEC_FREQUENCY_NULL? -1 : SUNSPEC_FREQUENCY_INT;
  SUNSPEC_CURRENT0 = SUNSPEC_CURRENT0_NULL? -1 : SUNSPEC_CURRENT0_INT;
  SUNSPEC_CURRENT1 = SUNSPEC_CURRENT1_NULL? -1 : SUNSPEC_CURRENT1_INT;
  SUNSPEC_CURRENT2 = SUNSPEC_CURRENT2_NULL? -1 : SUNSPEC_CURRENT2_INT;
  SUNSPEC_VOLTAGE0 = SUNSPEC_VOLTAGE0_NULL? -1 : SUNSPEC_VOLTAGE0_INT;
  SUNSPEC_VOLTAGE1 = SUNSPEC_VOLTAGE1_NULL? -1 : SUNSPEC_VOLTAGE1_INT;
  SUNSPEC_VOLTAGE2 = SUNSPEC_VOLTAGE2_NULL? -1 : SUNSPEC_VOLTAGE2_INT;
  SUNSPEC_REAL_POWER0 = SUNSPEC_REAL_POWER0_NULL? -1 : SUNSPEC_REAL_POWER0_INT;
  SUNSPEC_REAL_POWER1 = SUNSPEC_REAL_POWER1_NULL? -1 : SUNSPEC_REAL_POWER1_INT;
  SUNSPEC_REAL_POWER2 = SUNSPEC_REAL_POWER2_NULL? -1 : SUNSPEC_REAL_POWER2_INT;
  SUNSPEC_APPARANT_POWER0 = SUNSPEC_APPARANT_POWER0_NULL? -1 : SUNSPEC_APPARANT_POWER0_INT;
  SUNSPEC_APPARANT_POWER1 = SUNSPEC_APPARANT_POWER1_NULL? -1 : SUNSPEC_APPARANT_POWER1_INT;
  SUNSPEC_APPARANT_POWER2 = SUNSPEC_APPARANT_POWER2_NULL? -1 : SUNSPEC_APPARANT_POWER2_INT;
  SUNSPEC_POWER_FACTOR0 = SUNSPEC_POWER_FACTOR0_NULL? -1 : SUNSPEC_POWER_FACTOR0_INT;
  SUNSPEC_POWER_FACTOR1 = SUNSPEC_POWER_FACTOR1_NULL? -1 : SUNSPEC_POWER_FACTOR1_INT;
  SUNSPEC_POWER_FACTOR2 = SUNSPEC_POWER_FACTOR2_NULL? -1 : SUNSPEC_POWER_FACTOR2_INT;
  SUNSPEC_REAL_ENERGY_EXPORTED0 = SUNSPEC_REAL_ENERGY_EXPORTED0_NULL? -1 : SUNSPEC_REAL_ENERGY_EXPORTED0_INT;
  SUNSPEC_REAL_ENERGY_EXPORTED1 = SUNSPEC_REAL_ENERGY_EXPORTED1_NULL? -1 : SUNSPEC_REAL_ENERGY_EXPORTED1_INT;
  SUNSPEC_REAL_ENERGY_EXPORTED2 = SUNSPEC_REAL_ENERGY_EXPORTED2_NULL? -1 : SUNSPEC_REAL_ENERGY_EXPORTED2_INT;
  SUNSPEC_REAL_ENERGY_IMPORTED0 = SUNSPEC_REAL_ENERGY_IMPORTED0_NULL? -1 : SUNSPEC_REAL_ENERGY_IMPORTED0_INT;
  SUNSPEC_REAL_ENERGY_IMPORTED1 = SUNSPEC_REAL_ENERGY_IMPORTED1_NULL? -1 : SUNSPEC_REAL_ENERGY_IMPORTED1_INT;
  SUNSPEC_REAL_ENERGY_IMPORTED2 = SUNSPEC_REAL_ENERGY_IMPORTED2_NULL? -1 : SUNSPEC_REAL_ENERGY_IMPORTED2_INT;
  
  SUNSPEC_VOLTAGE_TYPE = get_modbus_type(config_doc["sunspec_registers"]["voltage_type"].as<String>());
  SUNSPEC_REAL_POWER_TYPE = get_modbus_type(config_doc["sunspec_registers"]["real_power_type"].as<String>());
  SUNSPEC_APPARANT_POWER_TYPE = get_modbus_type(config_doc["sunspec_registers"]["apparent_power_type"].as<String>());
  SUNSPEC_CURRENT_TYPE = get_modbus_type(config_doc["sunspec_registers"]["current_type"].as<String>());
  SUNSPEC_POWER_FACTOR_TYPE = get_modbus_type(config_doc["sunspec_registers"]["power_factor_type"].as<String>());
  SUNSPEC_FREQUENCY_TYPE = get_modbus_type(config_doc["sunspec_registers"]["frequency_type"].as<String>());
  SUNSPEC_REAL_ENERGY_TYPE = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>());
  }


}

double convert_modbus_type(int16_t *data, ModbusType type) {
  uint16_t *u16_data = (uint16_t *)data;

  switch (type) {
    case TYPE_INT16:
      return (int16_t)data[0];

    case TYPE_UINT16:
      return (uint16_t)u16_data[0];

    case TYPE_INT32: {
      int32_t value = ((int32_t)u16_data[0] << 16) | u16_data[1];
      return value;
    }

    case TYPE_UINT32: {
      uint32_t value = ((uint32_t)u16_data[0] << 16) | u16_data[1];
      return value;
    }

    case TYPE_FLOAT32: {
      uint32_t value = ((uint32_t)u16_data[0] << 16) | u16_data[1];
      float fvalue;
      memcpy(&fvalue, &value, sizeof(fvalue));
      return fvalue;
    }

    default:
      return NAN;
  }
}

bool modbusSunSpecCallback(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
 //fill with default values if no data is available
  for (int n=0; n<3;n++) {
    PhasePower[n].power = 0;
    PhasePower[n].apparentPower = 0;
    PhasePower[n].current = 0;
    PhasePower[n].powerFactor = 1;
    PhasePower[n].voltage = 230;
    PhasePower[n].frequency = 50;
    
  }


  if (event == Modbus::EX_SUCCESS)
  {
    // data available, process it

 
    int32_t power = 0;
    double scale_V = (SUNSPEC_VOLTAGE_SCALE>= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_VOLTAGE_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_real_power = (SUNSPEC_REAL_POWER_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_REAL_POWER_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_apparant_power = (SUNSPEC_APPARANT_POWER_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_APPARANT_POWER_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_current = (SUNSPEC_CURRENT_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_CURRENT_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_powerfactor = (SUNSPEC_POWER_FACTOR_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_POWER_FACTOR_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_frequency = (SUNSPEC_FREQUENCY_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_FREQUENCY_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_real_energy = (SUNSPEC_REAL_ENERGY_SCALE >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_REAL_ENERGY_SCALE - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;

    PhasePower[0].power = (SUNSPEC_REAL_POWER0 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[0].apparentPower = (SUNSPEC_APPARANT_POWER0 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[0].current = (SUNSPEC_CURRENT0 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[0].powerFactor = (SUNSPEC_POWER_FACTOR0 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR0 - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[0].voltage = (SUNSPEC_VOLTAGE0 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[0].frequency = (SUNSPEC_FREQUENCY >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

    power += PhasePower[0].power;
    if (SUNSPEC_REAL_ENERGY_IMPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[0].consumption = p * scale_real_energy;
    }
    if (SUNSPEC_REAL_ENERGY_EXPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED0 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[0].gridfeedin = p * scale_real_energy;
    }

    PhasePower[1].power = (SUNSPEC_REAL_POWER1 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[1].apparentPower = (SUNSPEC_APPARANT_POWER1 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[1].current = (SUNSPEC_CURRENT1 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[1].powerFactor = (SUNSPEC_POWER_FACTOR1 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR1 - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[1].voltage = (SUNSPEC_VOLTAGE1 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[1].frequency = (SUNSPEC_FREQUENCY >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

    power += PhasePower[1].power;
    if (SUNSPEC_REAL_ENERGY_IMPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[1].consumption = p * scale_real_energy;
    }
    if (SUNSPEC_REAL_ENERGY_EXPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED1 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[1].gridfeedin = p * scale_real_energy;
    }
  
    PhasePower[2].power = (SUNSPEC_REAL_POWER2 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[2].apparentPower = (SUNSPEC_APPARANT_POWER2 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[2].current = (SUNSPEC_CURRENT2 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[2].powerFactor = (SUNSPEC_POWER_FACTOR2 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR2 - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[2].voltage = (SUNSPEC_VOLTAGE2 >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[2].frequency = (SUNSPEC_FREQUENCY >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

    power += PhasePower[2].power;
    if (SUNSPEC_REAL_ENERGY_IMPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[2].consumption = p * scale_real_energy;
    }
    if (SUNSPEC_REAL_ENERGY_EXPORTED0 >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED2 - SUNSPEC_BASE, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[2].gridfeedin = p * scale_real_energy;
    }
 
   
    DEBUG_SERIAL.print(F("SUNSPEC power: "));
    DEBUG_SERIAL.println(power);
  }
  else
  {
    // error like timeout, connection lost, etc.
    DEBUG_SERIAL.print(F("Modbus-error: "));
    DEBUG_SERIAL.println(event);
    modbus1.disconnect(modbus_ip); // disconnect from the device to reset the connection
  }
  return true;
}

void parseSUNSPEC()
{

  if (!modbus1.isConnected(modbus_ip))
  {
    modbus1.connect(modbus_ip, sunspec_port_int);
  }
  else
  {
    if (SUNSPEC_REGISTER_COUNT> MAX_MODBUS_REGISTERS-1) {
          DEBUG_SERIAL.println(F("SUNSPEC register count exceeds buffer size "));
          DEBUG_SERIAL.println(MAX_MODBUS_REGISTERS - 1);
          DEBUG_SERIAL.println(F(", please check your configuration!"));
          return;
    }
    modbus1.readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t *)&modbus_result[0], SUNSPEC_REGISTER_COUNT, modbusSunSpecCallback, modbusdev_int);
    delay(10);
    modbus1.task();
  }
}