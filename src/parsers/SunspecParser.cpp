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

bool modbusSunSpecCallback(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
 
 
  if (event == Modbus::EX_SUCCESS)
  {
    // data available, process it
    int32_t power = 0;
    double scale_V = SUNSPEC_scale(modbus_result[SUNSPEC_VOLTAGE_SCALE - SUNSPEC_BASE]);
    double scale_real_power = SUNSPEC_scale(modbus_result[SUNSPEC_REAL_POWER_SCALE - SUNSPEC_BASE]);
    double scale_apparant_power = SUNSPEC_scale(modbus_result[SUNSPEC_APPARANT_POWER_SCALE - SUNSPEC_BASE]);
    double scale_current = SUNSPEC_scale(modbus_result[SUNSPEC_CURRENT_SCALE - SUNSPEC_BASE]);
    double scale_powerfactor = SUNSPEC_scale(modbus_result[SUNSPEC_POWER_FACTOR_SCALE - SUNSPEC_BASE]);
    double scale_frequency = SUNSPEC_scale(modbus_result[SUNSPEC_FREQUENCY_SCALE - SUNSPEC_BASE]);
    double scale_real_energy = SUNSPEC_scale(modbus_result[SUNSPEC_REAL_ENERGY_SCALE - SUNSPEC_BASE]);

    PhasePower[0].power = modbus_result[SUNSPEC_REAL_POWER0 - SUNSPEC_BASE] * scale_real_power + offsetPerPhase;
    PhasePower[0].apparentPower = modbus_result[SUNSPEC_APPARANT_POWER0 - SUNSPEC_BASE] * scale_apparant_power + offsetPerPhase;
    PhasePower[0].current = modbus_result[SUNSPEC_CURRENT0 - SUNSPEC_BASE] * scale_current;
    PhasePower[0].powerFactor = modbus_result[SUNSPEC_POWER_FACTOR0 - SUNSPEC_BASE] * scale_powerfactor;
    PhasePower[0].voltage = modbus_result[SUNSPEC_VOLTAGE0 - SUNSPEC_BASE] * scale_V;
    PhasePower[0].frequency = modbus_result[SUNSPEC_FREQUENCY - SUNSPEC_BASE] * scale_frequency;

    power += PhasePower[0].power;

    {
      uint32_t p = 0;
      uint8_t *p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_IMPORTED0 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[0].consumption = p / 1000.0 * scale_real_energy;
      p = 0;
      p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_EXPORTED0 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[0].gridfeedin = p / 1000.0 * scale_real_energy;
    }

    PhasePower[1].power = modbus_result[SUNSPEC_REAL_POWER1 - SUNSPEC_BASE] * scale_real_power + offsetPerPhase;
    PhasePower[1].apparentPower = modbus_result[SUNSPEC_APPARANT_POWER1 - SUNSPEC_BASE] * scale_apparant_power + offsetPerPhase;
    PhasePower[1].current = modbus_result[SUNSPEC_CURRENT1 - SUNSPEC_BASE] * scale_current;
    PhasePower[1].powerFactor = modbus_result[SUNSPEC_POWER_FACTOR1 - SUNSPEC_BASE] * scale_powerfactor;
    PhasePower[1].voltage = modbus_result[SUNSPEC_VOLTAGE1 - SUNSPEC_BASE] * scale_V;
    PhasePower[1].frequency = modbus_result[SUNSPEC_FREQUENCY - SUNSPEC_BASE] * scale_frequency;

    power += PhasePower[1].power;
    {
      uint32_t p = 0;
      uint8_t *p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_IMPORTED1 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[1].consumption = p / 1000.0 * scale_real_energy;
      p = 0;
      p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_EXPORTED1 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[1].gridfeedin = p / 1000.0 * scale_real_energy;
    }

    PhasePower[2].power = modbus_result[SUNSPEC_REAL_POWER2 - SUNSPEC_BASE] * scale_real_power + offsetPerPhase;
    PhasePower[2].apparentPower = modbus_result[SUNSPEC_APPARANT_POWER2 - SUNSPEC_BASE] * scale_apparant_power + offsetPerPhase;
    PhasePower[2].current = modbus_result[SUNSPEC_CURRENT2 - SUNSPEC_BASE] * scale_current;
    PhasePower[2].powerFactor = modbus_result[SUNSPEC_POWER_FACTOR2 - SUNSPEC_BASE] * scale_powerfactor;
    PhasePower[2].voltage = modbus_result[SUNSPEC_VOLTAGE2 - SUNSPEC_BASE] * scale_V;
    PhasePower[2].frequency = modbus_result[SUNSPEC_FREQUENCY - SUNSPEC_BASE] * scale_frequency;

    power += PhasePower[2].power;
    {
      uint32_t p = 0;
      uint8_t *p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_IMPORTED2 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[2].consumption = p / 1000.0 * scale_real_energy;
      p = 0;
      p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_EXPORTED2 - SUNSPEC_BASE];
      p |= ((uint32_t)p_u8[2]) << 0;
      p |= ((uint32_t)p_u8[3]) << 8;
      p |= ((uint32_t)p_u8[0]) << 16;
      p |= ((uint32_t)p_u8[1]) << 24;
      PhaseEnergy[2].gridfeedin = p / 1000.0 * scale_real_energy;
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
    modbus_ip.fromString(mqtt_server);
    sunspec_port_int = atol(mqtt_port);
    modbusdev_int = atol(modbus_dev);
    modbus1.connect(modbus_ip, sunspec_port_int);
  }
  else
  {
    // issue an holding register read to read the SUNSPEC registers, the callback will be called when the data is available
    modbus1.readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t *)&modbus_result[0], SUNSPEC_REGISTER_COUNT, modbusSunSpecCallback, modbusdev_int);
    delay(10);
    modbus1.task();
  }
}
