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
SunspecMap *sunspecTable=NULL;


ModbusType get_modbus_type(const String& type) {
  if (type == "int16")   return TYPE_INT16;
  if (type == "uint16")  return TYPE_UINT16;
  if (type == "int32")   return TYPE_INT32;
  if (type == "uint32")  return TYPE_UINT32;
  if (type == "float32") return TYPE_FLOAT32;
  return TYPE_UNKNOWN;
}

int32_t SUNSPEC_BASE = 0;
int32_t SUNSPEC_REGISTER_COUNT=0;


template <typename TKey>
int32_t get_checked_reg(JsonObject regs, TKey key, bool is_32bit = false) {
  if (regs.isNull() || regs[key].isNull()) return -1;
  
  int32_t reg = regs[key].template as<int32_t>();
  int32_t offset = reg - SUNSPEC_BASE;
  int32_t size = is_32bit ? 2 : 1;
  
  if (offset < 0 || (offset + size) > SUNSPEC_REGISTER_COUNT) {
    return -1;
  }
  return offset;
}


template <typename TReg, typename TType, typename TScale>
void add_sunspec_entry(int& index, JsonObject regs, double* dst, 
                       TReg regKey, TType typeKey, 
                       TScale scaleKey, double defaultVal) {
  if (index >= N_SUNSPEC_TABLE) return;

  ModbusType mType = get_modbus_type(regs[typeKey] | "");
  bool is32Bit = (mType == TYPE_INT32 || mType == TYPE_UINT32 || mType == TYPE_FLOAT32);

  sunspecTable[index] = {
    .dst = dst,
    .reg = get_checked_reg(regs, regKey, is32Bit),
    .modbusType = mType,
    .scale_register = get_checked_reg(regs, scaleKey, false),
    .offset = 0,
    .defaultVal = defaultVal
  };
  index++;
}


void add_phase_entry(int& index, JsonObject regs, double* dst, 
                     const __FlashStringHelper* baseKey, int idx, 
                     const __FlashStringHelper* typeKey, 
                     const __FlashStringHelper* scaleKey, double defaultVal) {
  char keyBuf[32];
#if defined(ESP32)
  snprintf(keyBuf, sizeof(keyBuf), "%s%d", reinterpret_cast<const char*>(baseKey), idx);
#else
  snprintf_P(keyBuf, sizeof(keyBuf), PSTR("%S%d"), baseKey, idx);
#endif
  add_sunspec_entry(index, regs, dst, keyBuf, typeKey, scaleKey, defaultVal);
}



void add_phase_entry(int& index, JsonObject regs, double* dst, 
                     const char* baseKey, int idx, 
                     const char* typeKey, 
                     const char* scaleKey, double defaultVal) {
  char keyBuf[32];
  snprintf_P(keyBuf, sizeof(keyBuf), PSTR("%s%d"), baseKey, idx);
  add_sunspec_entry(index, regs, dst, keyBuf, typeKey, scaleKey, defaultVal);
}

void convert_json_to_sunspec_registers() {
  if (!config_doc["sunspec"].isNull()) {
    modbusdev_int = config_doc["sunspec"]["modbus_dev"].as<uint8_t>();
    sunspec_port_int = config_doc["sunspec"]["modbus_port"].as<uint16_t>();
    modbus_ip.fromString(config_doc["sunspec"]["modbus_ip"] | "");
  }

  JsonObject regs = config_doc["sunspec_registers"];
  if (regs.isNull()) return;

  SUNSPEC_BASE = regs["base"].isNull() ? 0xFFFF : min(0xFFFF, regs["base"].as<int>());
  SUNSPEC_REGISTER_COUNT = regs["register_count"].isNull() ? 0 : min(MAX_MODBUS_REGISTERS, regs["register_count"].as<int>());


  if (sunspecTable==NULL) { sunspecTable = new SunspecMap[N_SUNSPEC_TABLE]; }
  memset(sunspecTable, 0, N_SUNSPEC_TABLE * sizeof(SunspecMap));

  int n = 0;

  for (int i = 0; i < 3; i++) {
    add_sunspec_entry(n, regs, &PhasePower[i].frequency, 
                      F("frequency"), F("frequency_type"), F("frequency_scale"), 50.0);
  }

  for (int i = 0; i < 3; i++) {
    add_phase_entry(n, regs, &PhasePower[i].voltage,       F("voltage"),              i, F("voltage_type"),        F("voltage_scale"),        230.0);
    add_phase_entry(n, regs, &PhasePower[i].current,       F("current"),              i, F("current_type"),        F("current_scale"),        0.0);
    add_phase_entry(n, regs, &PhasePower[i].power,         F("real_power"),           i, F("real_power_type"),     F("real_power_scale"),     0.0);
    add_phase_entry(n, regs, &PhasePower[i].apparentPower, F("apparent_power"),       i, F("apparent_power_type"), F("apparent_power_scale"), 0.0);
    add_phase_entry(n, regs, &PhasePower[i].powerFactor,   F("power_factor"),         i, F("power_factor_type"),   F("power_factor_scale"),   1.0);
    add_phase_entry(n, regs, &PhaseEnergy[i].gridfeedin,   F("real_energy_exported"), i, F("real_energy_type"),    F("real_energy_scale"),    0.0);
    add_phase_entry(n, regs, &PhaseEnergy[i].consumption,  F("real_energy_imported"), i, F("real_energy_type"),    F("real_energy_scale"),    0.0);
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
    power=PhasePower[0].power+PhasePower[1].power+PhasePower[2].power;

    DEBUG_SERIAL.print(F("SUNSPEC power: "));
    DEBUG_SERIAL.println(power);
  }
  else
  {
    // error like timeout, connection lost, etc.
    DEBUG_SERIAL.print(F("Modbus-error: "));
    DEBUG_SERIAL.println(event);
    modbus1->disconnect(modbus_ip); // disconnect from the device to reset the connection
  }
  return true;
}

void parseSUNSPEC()
{

  if (!modbus1->isConnected(modbus_ip))
  {
    modbus1->connect(modbus_ip, sunspec_port_int);
  }
  else
  {
    if (SUNSPEC_REGISTER_COUNT> MAX_MODBUS_REGISTERS-1) {
          DEBUG_SERIAL.println(F("SUNSPEC register count exceeds buffer size "));
          DEBUG_SERIAL.println(MAX_MODBUS_REGISTERS - 1);
          DEBUG_SERIAL.println(F(", please check your configuration!"));
          return;
    }
    modbus1->readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t *)&modbus_result[0], SUNSPEC_REGISTER_COUNT, modbusSunSpecCallback, modbusdev_int);
    delay(10);
    modbus1->task();
  }
}