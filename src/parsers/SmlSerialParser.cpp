#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include <sml.h>

// functions for SML via UART (IR reading head wired to the ESP)
#ifdef ESP32
#include "driver/gpio.h"
static HardwareSerial &smlSerial = Serial1;
#else
#include <SoftwareSerial.h>
static SoftwareSerial smlSerial;
#endif

#define SML_MAX_BYTES_PER_CALL 512   // limit time spent in one worker_loop() pass
#define SML_STATUS_INTERVAL 30000    // print statistics every 30s
#define SML_PREVIEW_BYTES 64         // hex dump of the first bytes after boot, helps with baud/invert issues

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

static bool sml_serial_ready = false;
static uint32_t sml_bytes = 0, sml_frames_ok = 0, sml_frames_crc_error = 0, sml_unexpected = 0, sml_frames_no_power = 0;
static unsigned long sml_last_frame = 0;
static unsigned long sml_last_status = 0;

typedef struct
{
  const unsigned char OBIS[6];
  void (*Handler)();
} SmlSerialOBISHandler;

// read the current OBIS value and remember that the meter sent it
static void smlReadValue(double &target, sml_units_t unit, uint8_t seen_bit)
{
  long long int val;
  signed char scaler;
  smlOBISByUnit(val, scaler, unit);
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
static void smlConsumption() { smlReadValue(sml_consumption, SML_WATT_HOUR, SML_SEEN_CONSUMPTION); }
static void smlProduction() { smlReadValue(sml_production, SML_WATT_HOUR, SML_SEEN_PRODUCTION); }
static void smlPower() { smlReadValue(sml_power, SML_WATT, SML_SEEN_POWER); }
static void smlPowerL1() { smlReadValue(sml_power_l1, SML_WATT, SML_SEEN_POWER_L1); }
static void smlPowerL2() { smlReadValue(sml_power_l2, SML_WATT, SML_SEEN_POWER_L2); }
static void smlPowerL3() { smlReadValue(sml_power_l3, SML_WATT, SML_SEEN_POWER_L3); }

static SmlSerialOBISHandler smlSerialOBISHandlers[] = {
    {{0x01, 0x00, 0x01, 0x08, 0x00, 0xff}, &smlConsumption}, /* 1-0: 1. 8.0*255 (Consumption Total) */
    {{0x01, 0x00, 0x02, 0x08, 0x00, 0xff}, &smlProduction},  /* 1-0: 2. 8.0*255 (Production Total) */
    {{0x01, 0x00, 0x10, 0x07, 0x00, 0xff}, &smlPower},       /* 1-0:16. 7.0*255 (power) */
    {{0x01, 0x00, 0x24, 0x07, 0x00, 0xff}, &smlPowerL1},     /* 1-0:36. 7.0*255 (power L1) */
    {{0x01, 0x00, 0x38, 0x07, 0x00, 0xff}, &smlPowerL2},     /* 1-0:56. 7.0*255 (power L2) */
    {{0x01, 0x00, 0x4c, 0x07, 0x00, 0xff}, &smlPowerL3},     /* 1-0:76. 7.0*255 (power L3) */
    {{0, 0}}};

// reject pins that would detach SPI flash/PSRAM or the native USB port
static bool smlValidRxPin(int pin)
{
#ifdef ESP32
  if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT || !GPIO_IS_VALID_GPIO(pin))
    return false;
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  if (pin >= 12 && pin <= 19) // 12-17 SPI flash, 18/19 USB D-/D+
    return false;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  if (pin == 19 || pin == 20 || (pin >= 26 && pin <= 32)) // USB D-/D+, SPI flash/PSRAM
    return false;
#elif defined(CONFIG_IDF_TARGET_ESP32)
  if (pin >= 6 && pin <= 11) // SPI flash
    return false;
#endif
  return true;
#else
  return pin >= 0 && pin <= 15 && !(pin >= 6 && pin <= 11); // 6-11 SPI flash, 16 has no interrupt
#endif
}

void setupSMLSerial()
{
  int rx_pin = sml_rx_pin;
  uint32_t baud = sml_baud;
  bool invert = (sml_invert != NULL && strcmp(sml_invert, "true") == 0);

  if (!smlValidRxPin(rx_pin))
  {
    DEBUG_SERIAL.printf("SML serial: GPIO%d can not be used as RX pin, please change the configuration!\r\n", rx_pin);
    return;
  }
#ifdef ESP32
  smlSerial.setRxBufferSize(1024); // must be called before begin(), one SML telegram is typically 250-600 bytes
  smlSerial.begin(baud, SERIAL_8N1, rx_pin, -1, invert);
#else
  // explicit ISR buffer size, the default (10 x bufCapacity edges) does not fit into the ESP8266 heap
  smlSerial.begin(baud, SWSERIAL_8N1, rx_pin, -1, invert, 256, 1024);
#endif
  if (!smlSerial)
  {
    DEBUG_SERIAL.printf("SML serial: failed to open UART on GPIO%d!\r\n", rx_pin);
    return;
  }
  sml_serial_ready = true;
  sml_last_frame = millis();
  sml_last_status = millis();
  DEBUG_SERIAL.printf("SML serial: listening on GPIO%d, %u baud, 8N1, invert=%s\r\n", rx_pin, (unsigned int)baud, invert ? "true" : "false");
}

void parseSMLSerial()
{
  if (!sml_serial_ready)
    return;

  int iHandler = 0;
  sml_states_t s;
  uint16_t counter = 0;
  while (smlSerial.available() && counter < SML_MAX_BYTES_PER_CALL)
  {
    counter++;
    unsigned char val = smlSerial.read();
    sml_bytes++;
    if (sml_bytes <= SML_PREVIEW_BYTES)
    {
      DEBUG_SERIAL.printf("%02x ", val);
      if (sml_bytes == SML_PREVIEW_BYTES)
        DEBUG_SERIAL.println();
    }
    s = smlState(val);
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
          smlSerialOBISHandlers[iHandler].Handler != 0 && !(smlOBISCheck(smlSerialOBISHandlers[iHandler].OBIS));
          iHandler++)
        ;
      if (smlSerialOBISHandlers[iHandler].Handler != 0)
      {
        smlSerialOBISHandlers[iHandler].Handler();
      }
      break;
    case SML_UNEXPECTED:
      sml_unexpected++; // normal while syncing to the first telegram, do not print every byte
      break;
    case SML_CHECKSUM_ERROR:
      sml_frames_crc_error++;
      DEBUG_SERIAL.println(F("SML serial: checksum error, telegram discarded"));
      break;
    case SML_FINAL:
      // only publish values the meter actually sent, a missing OBIS must not be reported as 0
      if (sml_seen & SML_SEEN_ENERGY)
      {
        setEnergyData(sml_consumption, sml_production);
      }
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
        sml_frames_no_power++;
      }
      sml_frames_ok++;
      sml_last_frame = millis();
      break;
    default:
      break;
    }
  }

  if (millis() - sml_last_status >= SML_STATUS_INTERVAL)
  {
    sml_last_status = millis();
    DEBUG_SERIAL.printf("SML serial: %u bytes, %u telegrams ok, %u checksum errors, %u unexpected bytes\r\n",
                        (unsigned int)sml_bytes, (unsigned int)sml_frames_ok, (unsigned int)sml_frames_crc_error, (unsigned int)sml_unexpected);
    if (sml_bytes == 0)
    {
      DEBUG_SERIAL.println(F("SML serial: no data received, check wiring / RX pin / reading head position."));
    }
    else if (millis() - sml_last_frame >= SML_STATUS_INTERVAL)
    {
      DEBUG_SERIAL.println(F("SML serial: no valid telegram, check baud rate and invert setting."));
    }
    if (sml_frames_no_power > 0)
    {
      DEBUG_SERIAL.printf("SML serial: %u telegrams without power value (16.7.0), enter the meter PIN and enable extended data (InF on).\r\n", (unsigned int)sml_frames_no_power);
      sml_frames_no_power = 0;
    }
  }
}
