#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"

#define SUNSPEC_scale(n) (abs(n) < 10 ? pow(10.0, (double)n) : 1.0)

uint16_t SUNSPEC_BASE = 40073;

uint16_t SUNSPEC_VOLTAGE_SCALE = 40085;
uint16_t SUNSPEC_REAL_POWER_SCALE = 40092;
uint16_t SUNSPEC_APPARANT_POWER_SCALE = 40097;
uint16_t SUNSPEC_CURRENT_SCALE = 40076;
uint16_t SUNSPEC_POWER_FACTOR_SCALE = 40107;
uint16_t SUNSPEC_FREQUENCY_SCALE = 40087;
uint16_t SUNSPEC_REAL_ENERGY_SCALE = 40124;

uint16_t SUNSPEC_FREQUENCY = 40086;

uint16_t SUNSPEC_CURRENT0 = 40073;
uint16_t SUNSPEC_CURRENT1 = 40074;
uint16_t SUNSPEC_CURRENT2 = 40075;

uint16_t SUNSPEC_VOLTAGE0 = 40078;
uint16_t SUNSPEC_VOLTAGE1 = 40079;
uint16_t SUNSPEC_VOLTAGE2 = 40080;

uint16_t SUNSPEC_REAL_POWER0 = 40089;
uint16_t SUNSPEC_REAL_POWER1 = 40090;
uint16_t SUNSPEC_REAL_POWER2 = 40091;

uint16_t SUNSPEC_APPARANT_POWER0 = 40094;
uint16_t SUNSPEC_APPARANT_POWER1 = 40095;
uint16_t SUNSPEC_APPARANT_POWER2 = 40096;

uint16_t SUNSPEC_POWER_FACTOR0 = 40104;
uint16_t SUNSPEC_POWER_FACTOR1 = 40105;
uint16_t SUNSPEC_POWER_FACTOR2 = 40106;

uint16_t SUNSPEC_REAL_ENERGY_EXPORTED0 = 40110;
uint16_t SUNSPEC_REAL_ENERGY_EXPORTED1 = 40112;
uint16_t SUNSPEC_REAL_ENERGY_EXPORTED2 = 40114;

uint16_t SUNSPEC_REAL_ENERGY_IMPORTED0 = 40118;
uint16_t SUNSPEC_REAL_ENERGY_IMPORTED1 = 40120;
uint16_t SUNSPEC_REAL_ENERGY_IMPORTED2 = 40122;

uint16_t SUNSPEC_REGISTER_COUNT = 64;






    #define SUNSPEC_VOLTAGE_SCALE config_doc["sunspec_registers"]["voltage_scale"]
    #define SUNSPEC_VOLTAGE_SCALE_INT config_doc["sunspec_registers"]["voltage_scale"].as<int>()
    #define SUNSPEC_VOLTAGE_SCALE_NULL config_doc["sunspec_registers"]["voltage_scale"].isNull()
    #define SUNSPEC_VOLTAGE_TYPE config_doc["sunspec_registers"]["voltage_type"].as<String>()
    #define SUNSPEC_POWER_FACTOR_SCALE config_doc["sunspec_registers"]["power_factor_scale"]
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
    #define SUNSPEC_REAL_POWER_TYPE config_doc["sunspec_registers"]["real_power_type"].as<String>()
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
    #define SUNSPEC_APPARANT_POWER_TYPE config_doc["sunspec_registers"]["apparent_power_type"].as<String>()
    #define SUNSPEC_CURRENT_TYPE config_doc["sunspec_registers"]["current_type"].as<String>()
    #define SUNSPEC_POWER_FACTOR_TYPE config_doc["sunspec_registers"]["power_factor_type"].as<String>()
    #define SUNSPEC_FREQUENCY_TYPE config_doc["sunspec_registers"]["frequency_type"].as<String>()
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
    #define SUNSPEC_REAL_ENERGY_TYPE config_doc["sunspec_registers"]["real_energy_type"].as<String>()
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


double convert_modbus_type(int16_t *data, String type)
{
  uint16_t *u16_data = (uint16_t *)data;
  if (type == "int16")
  {
    return (int16_t)data[0];
  }
  else if (type == "uint16")
  {
    return (uint16_t)u16_data[0];
  }
  else if (type == "int32")
  {
    int32_t value = ((int32_t)u16_data[0] << 16) | u16_data[1];
    return value;
  }
  else if (type == "uint32")
  {
    uint32_t value = ((uint32_t)u16_data[0] << 16) | u16_data[1];
    return value;
  }
  else if (type == "float32")
  {
    uint32_t value = ((uint32_t)u16_data[0] << 16) | u16_data[1];
    float fvalue;
    memcpy(&fvalue, &value, sizeof(fvalue));
    return fvalue;
  }
  else
  {
    // Unsupported type
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


  if (config_doc["sunspec_registers"].isNull()) return true;
  if (event == Modbus::EX_SUCCESS && 
     !SUNSPEC_REGISTER_COUNT_NULL && 
     SUNSPEC_REGISTER_COUNT_INT > 0 &&
     !SUNSPEC_BASE_NULL && 
     SUNSPEC_BASE_INT >= 0)
  {
    // data available, process it

 
    int32_t power = 0;
    double scale_V = (!SUNSPEC_VOLTAGE_SCALE_NULL && SUNSPEC_VOLTAGE_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_VOLTAGE_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_real_power = (!SUNSPEC_REAL_POWER_SCALE_NULL && SUNSPEC_REAL_POWER_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_REAL_POWER_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_apparant_power = (!SUNSPEC_APPARANT_POWER_SCALE_NULL && SUNSPEC_APPARANT_POWER_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_APPARANT_POWER_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_current = (!SUNSPEC_CURRENT_SCALE_NULL && SUNSPEC_CURRENT_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_CURRENT_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_powerfactor = (!SUNSPEC_POWER_FACTOR_SCALE_NULL && SUNSPEC_POWER_FACTOR_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_POWER_FACTOR_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_frequency = (!SUNSPEC_FREQUENCY_SCALE_NULL && SUNSPEC_FREQUENCY_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_FREQUENCY_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;
    double scale_real_energy = (!SUNSPEC_REAL_ENERGY_SCALE_NULL && SUNSPEC_REAL_ENERGY_SCALE_INT >= 0) ? SUNSPEC_scale(modbus_result[constrain(SUNSPEC_REAL_ENERGY_SCALE_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)]) : 1.0;

    PhasePower[0].power = (!SUNSPEC_REAL_POWER0_NULL && SUNSPEC_REAL_POWER0_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[0].apparentPower = (!SUNSPEC_APPARANT_POWER0_NULL && SUNSPEC_APPARANT_POWER0_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[0].current = (!SUNSPEC_CURRENT0_NULL && SUNSPEC_CURRENT0_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[0].powerFactor = (!SUNSPEC_POWER_FACTOR0_NULL && SUNSPEC_POWER_FACTOR0_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[0].voltage = (!SUNSPEC_VOLTAGE0_NULL && SUNSPEC_VOLTAGE0_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[0].frequency = (!SUNSPEC_FREQUENCY_NULL && SUNSPEC_FREQUENCY_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

    power += PhasePower[0].power;
    if (!SUNSPEC_REAL_ENERGY_IMPORTED0_NULL && SUNSPEC_REAL_ENERGY_IMPORTED0_INT >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[0].consumption = p * scale_real_energy;
    }
    if (!SUNSPEC_REAL_ENERGY_EXPORTED0_NULL && SUNSPEC_REAL_ENERGY_EXPORTED0_INT >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED0_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[0].gridfeedin = p * scale_real_energy;
    }

    PhasePower[1].power = (!SUNSPEC_REAL_POWER1_NULL && SUNSPEC_REAL_POWER1_INT >= 0) ?convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[1].apparentPower = (!SUNSPEC_APPARANT_POWER1_NULL && SUNSPEC_APPARANT_POWER1_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[1].current = (!SUNSPEC_CURRENT1_NULL && SUNSPEC_CURRENT1_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[1].powerFactor = (!SUNSPEC_POWER_FACTOR1_NULL && SUNSPEC_POWER_FACTOR1_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[1].voltage = (!SUNSPEC_VOLTAGE1_NULL && SUNSPEC_VOLTAGE1_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[1].frequency = (!SUNSPEC_FREQUENCY_NULL && SUNSPEC_FREQUENCY_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

      power += PhasePower[1].power;
    
    if (!SUNSPEC_REAL_ENERGY_IMPORTED1_NULL && SUNSPEC_REAL_ENERGY_IMPORTED1_INT >= 0)
    {
        double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
        PhaseEnergy[1].consumption = p * scale_real_energy;
    }
    if (!SUNSPEC_REAL_ENERGY_EXPORTED1_NULL && SUNSPEC_REAL_ENERGY_EXPORTED1_INT >= 0)
    {
        double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED1_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
        PhaseEnergy[1].gridfeedin = p * scale_real_energy;
    }
  
    PhasePower[2].power = (!SUNSPEC_REAL_POWER2_NULL && SUNSPEC_REAL_POWER2_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_POWER2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_POWER_TYPE) * scale_real_power + offsetPerPhase : 0;
    PhasePower[2].apparentPower = (!SUNSPEC_APPARANT_POWER2_NULL && SUNSPEC_APPARANT_POWER2_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_APPARANT_POWER2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_APPARANT_POWER_TYPE) * scale_apparant_power + offsetPerPhase : 0;
    PhasePower[2].current = (!SUNSPEC_CURRENT2_NULL && SUNSPEC_CURRENT2_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_CURRENT2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_CURRENT_TYPE) * scale_current : 0;
    PhasePower[2].powerFactor = (!SUNSPEC_POWER_FACTOR2_NULL && SUNSPEC_POWER_FACTOR2_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_POWER_FACTOR2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_POWER_FACTOR_TYPE) * scale_powerfactor : 0;
    PhasePower[2].voltage = (!SUNSPEC_VOLTAGE2_NULL && SUNSPEC_VOLTAGE2_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_VOLTAGE2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_VOLTAGE_TYPE) * scale_V : 0;
    PhasePower[2].frequency = (!SUNSPEC_FREQUENCY_NULL && SUNSPEC_FREQUENCY_INT >= 0) ? convert_modbus_type(&modbus_result[constrain(SUNSPEC_FREQUENCY_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_FREQUENCY_TYPE) * scale_frequency : 0;

    power += PhasePower[2].power;
    
    if (!SUNSPEC_REAL_ENERGY_IMPORTED2_NULL && SUNSPEC_REAL_ENERGY_IMPORTED2_INT >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_IMPORTED2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
      PhaseEnergy[2].consumption = p * scale_real_energy;
    }
    if (!SUNSPEC_REAL_ENERGY_EXPORTED2_NULL && SUNSPEC_REAL_ENERGY_EXPORTED2_INT >= 0)
    {
      double p = convert_modbus_type(&modbus_result[constrain(SUNSPEC_REAL_ENERGY_EXPORTED2_INT - SUNSPEC_BASE_INT, 0, MAX_MODBUS_REGISTERS - 2)], SUNSPEC_REAL_ENERGY_TYPE);
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
    modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
    sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();
    modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"].as<const char*>()); // Todo, replace by DNS resolver for hostnames, currently only IP addresses are supported
    modbus1.connect(modbus_ip, sunspec_port_int);
  }
  else
  {
    if (!config_doc["sunspec_registers"].isNull()) {
      
      if (!SUNSPEC_REGISTER_COUNT_NULL && SUNSPEC_REGISTER_COUNT_INT > 0 && !SUNSPEC_BASE_NULL && SUNSPEC_BASE_INT >= 0)
      {
        if (SUNSPEC_REGISTER_COUNT_INT> MAX_MODBUS_REGISTERS-1) {
          DEBUG_SERIAL.println(F("SUNSPEC register count exceeds buffer size "));
          DEBUG_SERIAL.println(MAX_MODBUS_REGISTERS - 1);
          DEBUG_SERIAL.println(F(", please check your configuration!"));
          return;
        }
        // issue an holding register read to read the SUNSPEC registers, the callback will be called when the data is available
        modbus1.readHreg(modbus_ip, SUNSPEC_BASE_INT, (uint16_t *)&modbus_result[0], SUNSPEC_REGISTER_COUNT_INT, modbusSunSpecCallback, modbusdev_int);
      }
    }
    
    delay(10);
    modbus1.task();
  }
}