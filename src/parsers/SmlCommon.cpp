#include "SmlCommon.h"
#include "../data/DataProcessing.h"
#include <limits.h>

// bits for the OBIS values found in the current telegram
#define SML_SEEN_CONSUMPTION 0x01
#define SML_SEEN_PRODUCTION  0x02
#define SML_SEEN_POWER       0x04
#define SML_SEEN_POWER_L1    0x08
#define SML_SEEN_POWER_L2    0x10
#define SML_SEEN_POWER_L3    0x20
#define SML_SEEN_ENERGY      (SML_SEEN_CONSUMPTION | SML_SEEN_PRODUCTION)
#define SML_SEEN_PHASES      (SML_SEEN_POWER_L1 | SML_SEEN_POWER_L2 | SML_SEEN_POWER_L3)

static double sml_consumption = 0, sml_production = 0, sml_power = 0;
static double sml_power_l1 = 0, sml_power_l2 = 0, sml_power_l3 = 0;
static uint8_t sml_seen = 0;
static bool sml_had_power = false;

typedef struct
{
  const unsigned char OBIS[6];
  void (*Handler)();
} OBISHandler;

// read the current OBIS value and remember that the meter sent it
static void smlReadValue(double &target, sml_units_t unit, uint8_t seen_bit)
{
  long long int val;
  signed char scaler = SCHAR_MIN; // stays untouched if the unit does not match or the list is incomplete
  smlOBISByUnit(val, scaler, unit);
  if (scaler == SCHAR_MIN)
    return;
  target = val * pow(10, scaler);
  sml_seen |= seen_bit;
}

// supports currently only:
// - consumption (OBIS 1-0:1.8.0)
// - production  (OBIS 1-0:2.8.0)
// - power       (OBIS 1-0:16.7.0)
// - power L1    (OBIS 1-0:36.7.0)
// - power L2    (OBIS 1-0:56.7.0)
// - power L3    (OBIS 1-0:76.7.0)
static void Consumption() { smlReadValue(sml_consumption, SML_WATT_HOUR, SML_SEEN_CONSUMPTION); }
static void Production() { smlReadValue(sml_production, SML_WATT_HOUR, SML_SEEN_PRODUCTION); }
static void Power() { smlReadValue(sml_power, SML_WATT, SML_SEEN_POWER); }
static void PowerL1() { smlReadValue(sml_power_l1, SML_WATT, SML_SEEN_POWER_L1); }
static void PowerL2() { smlReadValue(sml_power_l2, SML_WATT, SML_SEEN_POWER_L2); }
static void PowerL3() { smlReadValue(sml_power_l3, SML_WATT, SML_SEEN_POWER_L3); }

static OBISHandler OBISHandlers[] = {
    {{0x01, 0x00, 0x01, 0x08, 0x00, 0xff}, &Consumption}, /* 1-0: 1. 8.0*255 (Consumption Total) */
    {{0x01, 0x00, 0x02, 0x08, 0x00, 0xff}, &Production},  /* 1-0: 2. 8.0*255 (Production Total) */
    {{0x01, 0x00, 0x10, 0x07, 0x00, 0xff}, &Power},       /* 1-0:16. 7.0*255 (power) */
    {{0x01, 0x00, 0x24, 0x07, 0x00, 0xff}, &PowerL1},     /* 1-0:36. 7.0*255 (power L1) */
    {{0x01, 0x00, 0x38, 0x07, 0x00, 0xff}, &PowerL2},     /* 1-0:56. 7.0*255 (power L2) */
    {{0x01, 0x00, 0x4c, 0x07, 0x00, 0xff}, &PowerL3},     /* 1-0:76. 7.0*255 (power L3) */
    {{0, 0}}};

sml_states_t smlParseByte(unsigned char val)
{
  int iHandler = 0;
  sml_states_t s = smlState(val);
  switch (s)
  {
  case SML_START:
    /* reset local vars */
    sml_consumption = 0;
    sml_production = 0;
    sml_power = 0;
    sml_power_l1 = 0;
    sml_power_l2 = 0;
    sml_power_l3 = 0;
    sml_seen = 0;
    break;
  case SML_LISTEND:
    for (
        iHandler = 0;
        OBISHandlers[iHandler].Handler != 0 && !(smlOBISCheck(OBISHandlers[iHandler].OBIS));
        iHandler++)
      ;
    if (OBISHandlers[iHandler].Handler != 0)
    {
      OBISHandlers[iHandler].Handler();
    }
    break;
  case SML_FINAL:
    // energy counters are only published if the meter sent them, a missing counter must not be reported as 0
    if (sml_seen & SML_SEEN_ENERGY)
    {
      setEnergyData(sml_consumption, sml_production);
    }
    sml_had_power = true;
    if ((sml_seen & SML_SEEN_PHASES) == SML_SEEN_PHASES)
    {
      setPowerData(sml_power_l1, sml_power_l2, sml_power_l3);
    }
    else if (sml_seen & SML_SEEN_POWER)
    {
      setPowerData(sml_power);
    }
    else
    {
      // no power value in this telegram, publish 0 W instead of keeping the last (stale) value
      setPowerData(0.0);
      sml_had_power = false;
    }
    break;
  default:
    break;
  }
  return s;
}

bool smlLastTelegramHadPower()
{
  return sml_had_power;
}
