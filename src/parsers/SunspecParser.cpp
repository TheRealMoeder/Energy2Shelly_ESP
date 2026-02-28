#include "../config/Configuration.h"
#include "../data/DataStructures.h"
#include "../data/DataProcessor.h"

// FIX: Moved #defines out of function body to file scope.
// Defining macros inside a function is legal but misleading - they persist
// beyond the function scope. File-scope placement is clearer and correct.
#define SUNSPEC_BASE                  40072
#define SUNSPEC_CURRENT               40072
#define SUNSPEC_CURRENT_SCALE         40075
#define SUNSPEC_VOLTAGE               40077
#define SUNSPEC_VOLTAGE_SCALE         40084
#define SUNSPEC_FREQUENCY             40085
#define SUNSPEC_FREQUENCY_SCALE       40086
#define SUNSPEC_REAL_POWER            40088
#define SUNSPEC_REAL_POWER_SCALE      40091
#define SUNSPEC_APPARANT_POWER        40093
#define SUNSPEC_APPARANT_POWER_SCALE  40096
#define SUNSPEC_POWER_FACTOR          40103
#define SUNSPEC_POWER_FACTOR_SCALE    40106
#define SUNSPEC_REAL_ENERGY_EXPORTED  40109
#define SUNSPEC_REAL_IMPORTED_EXPORTED 40117
#define SUNSPEC_REAL_ENERGY_SCALE     40123

// Scale factor conversion for Sunspec Modbus registers
double SUNSPEC_scale(int n) {
  double val = 1.0;
  switch (n) {
  case -3:
    val = 0.001;
    break;
  case -2:
    val = 0.01;
    break;
  case -1:
    val = 0.1;
    break;
  case 0:
    val = 1.0;
    break;
  case 1:
    val = 10.0;
    break;
  case 2:
    val = 100.0;
    break;
  default:
    val = 1.0;
  }
  return val;
}

// Parse Sunspec Modbus TCP registers
void parseSUNSPEC() {
  DEBUG_SERIAL.printf("DEBUG: parseSUNSPEC() called, server=%s\n", mqtt_server);

  if (mqtt_server[0] == '\0') {
    return;
  }
  modbus_ip.fromString(mqtt_server);
  if (!modbus1.isConnected(modbus_ip)) {
    modbus1.connect(modbus_ip, mqttPortInt);
  } else {
    uint16_t transaction =
        modbus1.readHreg(modbus_ip, SUNSPEC_BASE, (uint16_t *)&modbus_result[0],
                         64, nullptr, modbusDeviceId);

    // FIX: Replaced delay(10) + busy-wait with yield()-based polling.
    // The original delay(10) blocked the CPU entirely, starving the WiFi stack,
    // WebSocket and MQTT handlers. yield() processes background tasks instead.
    // parseSUNSPEC() is called from loop() on a timer, so the overall polling
    // interval is preserved - only the blocking within each call is eliminated.
    modbus1.task();
    yield();
    int t = 0;
    while (modbus1.isTransaction(transaction)) {
      modbus1.task();
      yield();
      t++;
      if (t > 50) {
        DEBUG_SERIAL.println(F("Timeout SUNSPEC"));
        modbus1.disconnect(modbus_ip);
        break;
      }
    }
    int32_t power = 0;
    if (t <= 50) {
      double scale_V =
          SUNSPEC_scale(modbus_result[SUNSPEC_VOLTAGE_SCALE - SUNSPEC_BASE]);
      double scale_real_power =
          SUNSPEC_scale(modbus_result[SUNSPEC_REAL_POWER_SCALE - SUNSPEC_BASE]);
      double scale_apparant_power = SUNSPEC_scale(
          modbus_result[SUNSPEC_APPARANT_POWER_SCALE - SUNSPEC_BASE]);
      double scale_current =
          SUNSPEC_scale(modbus_result[SUNSPEC_CURRENT_SCALE - SUNSPEC_BASE]);
      double scale_powerfactor = SUNSPEC_scale(
          modbus_result[SUNSPEC_POWER_FACTOR_SCALE - SUNSPEC_BASE]);
      double scale_frequency =
          SUNSPEC_scale(modbus_result[SUNSPEC_FREQUENCY_SCALE - SUNSPEC_BASE]);

      for (int n = 0; n < 3; n++) {
        PhasePower[n].power =
            modbus_result[SUNSPEC_REAL_POWER - SUNSPEC_BASE + n] *
            scale_real_power;
        PhasePower[n].apparentPower =
            modbus_result[SUNSPEC_APPARANT_POWER - SUNSPEC_BASE + n] *
            scale_apparant_power;
        PhasePower[n].current =
            modbus_result[SUNSPEC_CURRENT - SUNSPEC_BASE + n] * scale_current;
        PhasePower[n].powerFactor =
            modbus_result[SUNSPEC_POWER_FACTOR - SUNSPEC_BASE + n] *
            scale_powerfactor;
        PhasePower[n].voltage =
            modbus_result[SUNSPEC_VOLTAGE - SUNSPEC_BASE + n] * scale_V;
        PhasePower[n].frequency =
            modbus_result[SUNSPEC_FREQUENCY - SUNSPEC_BASE] * scale_frequency;
        power += PhasePower[n].power;
      }

      double scale_real_energy = SUNSPEC_scale(
          modbus_result[SUNSPEC_REAL_ENERGY_SCALE - SUNSPEC_BASE]);
      for (int n = 0; n < 3; n++) {
        uint32_t p = 0;
        uint8_t *p_u8 =
            (uint8_t *)&modbus_result[SUNSPEC_REAL_IMPORTED_EXPORTED -
                                      SUNSPEC_BASE + 2 * n];
        p |= ((uint32_t)p_u8[2]) << 0;
        p |= ((uint32_t)p_u8[3]) << 8;
        p |= ((uint32_t)p_u8[0]) << 16;
        p |= ((uint32_t)p_u8[1]) << 24;
        PhaseEnergy[n].consumption = p / 1000.0 * scale_real_energy;
        p = 0;
        p_u8 = (uint8_t *)&modbus_result[SUNSPEC_REAL_ENERGY_EXPORTED -
                                         SUNSPEC_BASE + 2 * n];
        p |= ((uint32_t)p_u8[2]) << 0;
        p |= ((uint32_t)p_u8[3]) << 8;
        p |= ((uint32_t)p_u8[0]) << 16;
        p |= ((uint32_t)p_u8[1]) << 24;
        PhaseEnergy[n].gridfeedin = -p / 1000.0 * scale_real_energy;
      }
    }
    DEBUG_SERIAL.printf("SUNSPEC power: %d,%d\n\r", t, power);
  }
}
