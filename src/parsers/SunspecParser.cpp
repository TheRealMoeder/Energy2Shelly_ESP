#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"





const char STR_INT16[]   PROGMEM = "int16";
const char STR_UINT16[]  PROGMEM = "uint16";
const char STR_INT32[]   PROGMEM = "int32";
const char STR_UINT32[]  PROGMEM = "uint32";
const char STR_FLOAT32[] PROGMEM = "float32";

template <typename... Args>
constexpr size_t max_sizeof(const Args&... args) {
  size_t result = 0;
  ((result = sizeof(args) > result ? sizeof(args) : result), ...);
  return result;
}

constexpr size_t MAX_TYPE_LEN = max_sizeof(
  STR_INT16, STR_UINT16, STR_INT32, STR_UINT32, STR_FLOAT32
);

#define MODBUSTYPE_32BIT_DOMAIN 128

enum ModbusType {
  TYPE_UNKNOWN=0,
  TYPE_INT16,
  TYPE_UINT16,
  TYPE_INT32=MODBUSTYPE_32BIT_DOMAIN,
  TYPE_UINT32,
  TYPE_FLOAT32
};

struct ModbusTypeEntry {
  const char* name;
  ModbusType type;
};

const ModbusTypeEntry MODBUS_TYPES[] PROGMEM = {
  { STR_INT16,   TYPE_INT16   },
  { STR_UINT16,  TYPE_UINT16  },
  { STR_INT32,   TYPE_INT32   },
  { STR_UINT32,  TYPE_UINT32  },
  { STR_FLOAT32, TYPE_FLOAT32 }
};

constexpr size_t MODBUS_TYPES_COUNT = sizeof(MODBUS_TYPES) / sizeof(MODBUS_TYPES[0]);

ModbusType get_modbus_type(const char* type) {
  if (!type) return TYPE_UNKNOWN;

  for (size_t i = 0; i < MODBUS_TYPES_COUNT; ++i) {
    const char* flash_str = reinterpret_cast<const char*>(pgm_read_ptr(&MODBUS_TYPES[i].name));
    
    if (strncmp_P(type, flash_str, MAX_TYPE_LEN) == 0) {
      return static_cast<ModbusType>(pgm_read_byte(&MODBUS_TYPES[i].type));
    }
  }
  return TYPE_UNKNOWN;
}

ModbusType get_modbus_type(const __FlashStringHelper* type) {
  if (!type) return TYPE_UNKNOWN;

  char buf[MAX_TYPE_LEN];
  strncpy_P(buf, reinterpret_cast<const char*>(type), MAX_TYPE_LEN - 1);
  buf[MAX_TYPE_LEN - 1] = '\0';

  return get_modbus_type(buf);
}

ModbusType get_modbus_type(const String& type) {
  return get_modbus_type(type.c_str());
}


const float SCALE_FACTORS_F[] PROGMEM = {
  1e-10f, 1e-9f, 1e-8f, 1e-7f, 1e-6f, 1e-5f, 1e-4f, 1e-3f, 1e-2f, 1e-1f,
  1.0f,
  1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f, 1e8f, 1e9f, 1e10f
};

inline double get_sunspec_scale_f(int16_t n) {
  if (n >= -10 && n <= 10) {
    return pgm_read_float(&SCALE_FACTORS_F[n + 10]);
  }
  return 1.0f;
}

struct SunspecMap {
    double* dst;              
    int32_t reg;    
    int32_t scale_register;    
    ModbusType modbusType;       
};


#define N_SUNSPEC_TABLE 24
SunspecMap *sunspecTable=NULL;

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
                       TScale scaleKey) {
  if (index >= N_SUNSPEC_TABLE) return;

  ModbusType mType = get_modbus_type(regs[typeKey] | "");
  bool is32Bit = (mType >= MODBUSTYPE_32BIT_DOMAIN);

  sunspecTable[index] = {
    .dst = dst,
    .reg = get_checked_reg(regs, regKey, is32Bit),
    .scale_register = get_checked_reg(regs, scaleKey, false),
    .modbusType = mType
  };
  index++;
}


void add_phase_entry(int& index, JsonObject regs, double* dst, 
                     const __FlashStringHelper* baseKey, int idx, 
                     const __FlashStringHelper* typeKey, 
                     const __FlashStringHelper* scaleKey) {
  char keyBuf[32];
#if defined(ESP32)
  snprintf(keyBuf, sizeof(keyBuf), "%s%d", reinterpret_cast<const char*>(baseKey), idx);
#else
  snprintf_P(keyBuf, sizeof(keyBuf), PSTR("%S%d"), baseKey, idx);
#endif
  add_sunspec_entry(index, regs, dst, keyBuf, typeKey, scaleKey);
}



void add_phase_entry(int& index, JsonObject regs, double* dst, 
                     const char* baseKey, int idx, 
                     const char* typeKey, 
                     const char* scaleKey) {
  char keyBuf[32];
  snprintf_P(keyBuf, sizeof(keyBuf), PSTR("%s%d"), baseKey, idx);
  add_sunspec_entry(index, regs, dst, keyBuf, typeKey, scaleKey);
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
                      F("frequency"), F("frequency_type"), F("frequency_scale"));
  }

  for (int i = 0; i < 3; i++) {
    add_phase_entry(n, regs, &PhasePower[i].voltage,       F("voltage"),              i, F("voltage_type"),        F("voltage_scale"));
    add_phase_entry(n, regs, &PhasePower[i].current,       F("current"),              i, F("current_type"),        F("current_scale"));
    add_phase_entry(n, regs, &PhasePower[i].power,         F("real_power"),           i, F("real_power_type"),     F("real_power_scale"));
    add_phase_entry(n, regs, &PhasePower[i].apparentPower, F("apparent_power"),       i, F("apparent_power_type"), F("apparent_power_scale"));
    add_phase_entry(n, regs, &PhasePower[i].powerFactor,   F("power_factor"),         i, F("power_factor_type"),   F("power_factor_scale"));
    add_phase_entry(n, regs, &PhaseEnergy[i].gridfeedin,   F("real_energy_exported"), i, F("real_energy_type"),    F("real_energy_scale"));
    add_phase_entry(n, regs, &PhaseEnergy[i].consumption,  F("real_energy_imported"), i, F("real_energy_type"),    F("real_energy_scale"));
 }

}

float convert_modbus_type(int16_t *data, ModbusType type) {
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

double check_and_deliver_default_value(double *target)
{
  for (int n=0;n<3;n++) {
    if (target==&PhasePower[n].voltage) {
      ONA_validVoltage = false;
      return 230.0;
    }
    if (target==&PhasePower[n].frequency) {
      ONA_validfrequency = false;  
      return 50.0;
    }
    if (target==&PhasePower[n].powerFactor) return 1.0;
  }
  return 0;
}


bool modbusSunSpecCallback(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
 
  if (event == Modbus::EX_SUCCESS)
  {
#ifdef ONA_ACTIVE
    ONA_validVoltage = true;
    ONA_validfrequency = true;
#endif

    // data available, process it
 
    int32_t power = 0;
    float scale=1.0f;
    for (int n=0;n<N_SUNSPEC_TABLE;n++) {
      if (sunspecTable[n].dst!=NULL) {
        if (sunspecTable[n].reg<0) {
          sunspecTable[n].dst[0]=check_and_deliver_default_value(sunspecTable[n].dst);
          continue;
        }
        scale =  (sunspecTable[n].scale_register>=0) ? get_sunspec_scale_f(modbus_result[sunspecTable[n].scale_register])  : 1.0f;
        sunspecTable[n].dst[0]= (double) (convert_modbus_type(&modbus_result[sunspecTable[n].reg],sunspecTable[n].modbusType)*scale);

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