#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"

#define SUNSPEC_scale(n) (abs(n) < 10 ? pow(10.0, (double)n) : 1.0)

enum ModbusType {
  TYPE_UNKNOWN,
  TYPE_INT16,
  TYPE_UINT16,
  TYPE_INT32,
  TYPE_UINT32,
  TYPE_FLOAT32
};

struct SunspecMap {
    double* dst;              
    int32_t reg;            
    ModbusType modbusType;  
    int32_t scale_register;    
    double offset;          
    double defaultVal;      
};


#define N_SUNSPEC_TABLE 24
SunspecMap sunspecTable[N_SUNSPEC_TABLE] = {0};


ModbusType get_modbus_type(const String& type) {
  if (type == "int16")   return TYPE_INT16;
  if (type == "uint16")  return TYPE_UINT16;
  if (type == "int32")   return TYPE_INT32;
  if (type == "uint32")  return TYPE_UINT32;
  if (type == "float32") return TYPE_FLOAT32;
  return TYPE_UNKNOWN;
}



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


#define CHECK(b,a) (b) ? -1 : ( (a)-SUNSPEC_BASE<0) ? -1 : ( (a)-SUNSPEC_BASE>SUNSPEC_REGISTER_COUNT-2) ? -1 : (a)-SUNSPEC_BASE 

int32_t SUNSPEC_BASE = 40073;
int32_t SUNSPEC_REGISTER_COUNT = 64;


void convert_json_to_sunspec_registers()
{

  if (!config_doc["sunspec"].isNull()) {
  modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
  sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();
  modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"].as<const char*>());
  }
  if (!config_doc["sunspec_registers"].isNull()) {

    SUNSPEC_BASE = SUNSPEC_BASE_NULL? 0xFFFF : min(0xFFFF,SUNSPEC_BASE_INT);
    SUNSPEC_REGISTER_COUNT = SUNSPEC_REGISTER_COUNT_NULL? 0 : min(256,SUNSPEC_REGISTER_COUNT_INT);


    int n=0;
    /* frequency */
    sunspecTable[n]= {.dst=&PhasePower[0].frequency,
                       .reg=CHECK(SUNSPEC_FREQUENCY_NULL,SUNSPEC_FREQUENCY_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["frequency_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_FREQUENCY_SCALE_NULL,SUNSPEC_FREQUENCY_SCALE_INT),
                       .offset =0,
                       .defaultVal=50 };n++;
    sunspecTable[n]=  sunspecTable[n-1];sunspecTable[n].dst=&PhasePower[1].frequency; n++;
    sunspecTable[n]=  sunspecTable[n-1];sunspecTable[n].dst=&PhasePower[2].frequency; n++; 
    // n=3
  
    /* voltage */
    sunspecTable[n]= {.dst=&PhasePower[0].voltage,
                       .reg=CHECK(SUNSPEC_VOLTAGE0_NULL,SUNSPEC_VOLTAGE0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["voltage_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_VOLTAGE_SCALE_NULL,SUNSPEC_VOLTAGE_SCALE_INT),
                       .offset =0,
                       .defaultVal=230 };n++;
    sunspecTable[n]= {.dst=&PhasePower[1].voltage,
                       .reg=CHECK(SUNSPEC_VOLTAGE1_NULL,SUNSPEC_VOLTAGE1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["voltage_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_VOLTAGE_SCALE_NULL,SUNSPEC_VOLTAGE_SCALE_INT),
                       .offset =0,
                       .defaultVal=230 };n++;
    sunspecTable[n]= {.dst=&PhasePower[2].voltage,
                       .reg=CHECK(SUNSPEC_VOLTAGE2_NULL,SUNSPEC_VOLTAGE2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["voltage_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_VOLTAGE_SCALE_NULL,SUNSPEC_VOLTAGE_SCALE_INT),
                       .offset =0,
                       .defaultVal=230 };n++;
    // n= 6
 
    /* current */
    sunspecTable[n]= {.dst=&PhasePower[0].current,
                       .reg=CHECK(SUNSPEC_CURRENT0_NULL,SUNSPEC_CURRENT0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["current_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_CURRENT_SCALE_NULL,SUNSPEC_CURRENT_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    sunspecTable[n]= {.dst=&PhasePower[1].current,
                       .reg=CHECK(SUNSPEC_CURRENT1_NULL,SUNSPEC_CURRENT1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["current_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_CURRENT_SCALE_NULL,SUNSPEC_CURRENT_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;                       
    sunspecTable[n]= {.dst=&PhasePower[2].current,
                       .reg=CHECK(SUNSPEC_CURRENT2_NULL,SUNSPEC_CURRENT2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["current_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_CURRENT_SCALE_NULL,SUNSPEC_CURRENT_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    // n= 9

    /* real power */
    sunspecTable[n]= {.dst=&PhasePower[0].power,
                       .reg=CHECK(SUNSPEC_REAL_POWER0_NULL,SUNSPEC_REAL_POWER0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_POWER_SCALE_NULL,SUNSPEC_REAL_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    sunspecTable[n]= {.dst=&PhasePower[1].power,
                       .reg=CHECK(SUNSPEC_REAL_POWER1_NULL,SUNSPEC_REAL_POWER1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_POWER_SCALE_NULL,SUNSPEC_REAL_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;                       
    sunspecTable[n]= {.dst=&PhasePower[2].power,
                       .reg=CHECK(SUNSPEC_REAL_POWER2_NULL,SUNSPEC_REAL_POWER2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_POWER_SCALE_NULL,SUNSPEC_REAL_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    // n = 12

    /* apprant power */
    sunspecTable[n]= {.dst=&PhasePower[0].apparentPower,
                       .reg=CHECK(SUNSPEC_APPARANT_POWER0_NULL,SUNSPEC_APPARANT_POWER0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["apparent_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_APPARANT_POWER_SCALE_NULL,SUNSPEC_APPARANT_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    sunspecTable[n]= {.dst=&PhasePower[1].apparentPower,
                       .reg=CHECK(SUNSPEC_APPARANT_POWER1_NULL,SUNSPEC_APPARANT_POWER1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["apparent_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_APPARANT_POWER_SCALE_NULL,SUNSPEC_APPARANT_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;                       
    sunspecTable[n]= {.dst=&PhasePower[2].apparentPower,
                       .reg=CHECK(SUNSPEC_APPARANT_POWER2_NULL,SUNSPEC_APPARANT_POWER2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["apparent_power_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_APPARANT_POWER_SCALE_NULL,SUNSPEC_APPARANT_POWER_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    // n = 15

    /* power factor */
    sunspecTable[n]= {.dst=&PhasePower[0].powerFactor,
                       .reg=CHECK(SUNSPEC_POWER_FACTOR0_NULL,SUNSPEC_POWER_FACTOR0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["power_factor_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_POWER_FACTOR_SCALE_NULL,SUNSPEC_POWER_FACTOR_SCALE_INT),
                       .offset =0,
                       .defaultVal=1 };n++;
    sunspecTable[n]= {.dst=&PhasePower[1].powerFactor,
                       .reg=CHECK(SUNSPEC_POWER_FACTOR1_NULL,SUNSPEC_POWER_FACTOR1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["power_factor_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_POWER_FACTOR_SCALE_NULL,SUNSPEC_POWER_FACTOR_SCALE_INT),
                       .offset =0,
                       .defaultVal=1 };n++;                       
    sunspecTable[n]= {.dst=&PhasePower[2].powerFactor,
                       .reg=CHECK(SUNSPEC_POWER_FACTOR2_NULL,SUNSPEC_POWER_FACTOR2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["power_factor_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_POWER_FACTOR_SCALE_NULL,SUNSPEC_POWER_FACTOR_SCALE_INT),
                       .offset =0,
                       .defaultVal=1 };n++;
    // n = 18

    /* real energy exported */
    sunspecTable[n]= {.dst=&PhaseEnergy[0].gridfeedin,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_EXPORTED0_NULL,SUNSPEC_REAL_ENERGY_EXPORTED0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    sunspecTable[n]= {.dst=&PhaseEnergy[1].gridfeedin,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_EXPORTED1_NULL,SUNSPEC_REAL_ENERGY_EXPORTED1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;                       
    sunspecTable[n]= {.dst=&PhaseEnergy[2].gridfeedin,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_EXPORTED2_NULL,SUNSPEC_REAL_ENERGY_EXPORTED2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    // n = 21

    /* real energy imported */
    sunspecTable[n]= {.dst=&PhaseEnergy[0].consumption,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_IMPORTED0_NULL,SUNSPEC_REAL_ENERGY_IMPORTED0_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    sunspecTable[n]= {.dst=&PhaseEnergy[1].consumption,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_IMPORTED1_NULL,SUNSPEC_REAL_ENERGY_IMPORTED1_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;                       
    sunspecTable[n]= {.dst=&PhaseEnergy[2].consumption,
                       .reg=CHECK(SUNSPEC_REAL_ENERGY_IMPORTED2_NULL,SUNSPEC_REAL_ENERGY_IMPORTED2_INT),
                       .modbusType = get_modbus_type(config_doc["sunspec_registers"]["real_energy_type"].as<String>()),
                       .scale_register=CHECK(SUNSPEC_REAL_ENERGY_SCALE_NULL,SUNSPEC_REAL_ENERGY_SCALE_INT),
                       .offset =0,
                       .defaultVal=0 };n++;
    // n = 24

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
 
  if (event == Modbus::EX_SUCCESS)
  {
    // data available, process it

 
    int32_t power = 0;
    double scale=1.0;
    for (int n=0;n<N_SUNSPEC_TABLE;n++) {
      if (sunspecTable[n].dst!=NULL) {
        if (sunspecTable[n].reg<0) {
          sunspecTable[n].dst[0]=sunspecTable[n].defaultVal;
          continue;
        }
        scale =  (sunspecTable[n].scale_register>=0) ? SUNSPEC_scale(modbus_result[sunspecTable[n].scale_register])  : 1.0;
        sunspecTable[n].dst[0]=convert_modbus_type(&modbus_result[sunspecTable[n].reg],sunspecTable[n].modbusType)*scale;

      }
      
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