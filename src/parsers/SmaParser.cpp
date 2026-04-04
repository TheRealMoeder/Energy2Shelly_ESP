#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"

void parseSMA() {
  uint8_t buffer[1024];
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int rSize = Udp.read(buffer, 1024);
    if (buffer[0] != 'S' || buffer[1] != 'M' || buffer[2] != 'A') {
      DEBUG_SERIAL.println("Not an SMA packet?");
      return;
    }
    uint16_t grouplen;
    uint16_t grouptag;
    uint8_t *offset = buffer + 4;
    do {
      grouplen = (offset[0] << 8) + offset[1];
      grouptag = (offset[2] << 8) + offset[3];
      offset += 4;
      if (grouplen == 0xffff) return;
      if (grouptag == 0x02A0 && grouplen == 4) {
        offset += 4;
      } else if (grouptag == 0x0010) {
        uint8_t *endOfGroup = offset + grouplen;
        // uint16_t protocolID = (offset[0] << 8) + offset[1];
        offset += 2;
        // uint16_t susyID = (offset[0] << 8) + offset[1];
        offset += 2;
        uint32_t serial = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
        DEBUG_SERIAL.print("Received SMA multicast from ");
        DEBUG_SERIAL.println(serial);
        if (sma_id[0] != '\0' && strtoul(sma_id, nullptr, 10) != serial) {
          DEBUG_SERIAL.println("SMA serial not matching - ignoring packet");
          break;
        }
        offset += 4;
        // uint32_t timestamp = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
        offset += 4;
        while (offset < endOfGroup) {
          uint8_t channel = offset[0];
          uint8_t index = offset[1];
          uint8_t type = offset[2];
          // uint8_t tarif = offset[3];
          offset += 4;
          if (type == 8) {
            uint64_t data = ((uint64_t)offset[0] << 56) + ((uint64_t)offset[1] << 48) + ((uint64_t)offset[2] << 40) + ((uint64_t)offset[3] << 32) + ((uint64_t)offset[4] << 24) + ((uint64_t)offset[5] << 16) + ((uint64_t)offset[6] << 8) + offset[7];
            offset += 8;
            switch (index) {
              case 21:
                PhaseEnergy[0].consumption = data / 3600000;
                break;
              case 22:
                PhaseEnergy[0].gridfeedin = data / 3600000;
                break;
              case 41:
                PhaseEnergy[1].consumption = data / 3600000;
                break;
              case 42:
                PhaseEnergy[1].gridfeedin = data / 3600000;
                break;
              case 61:
                PhaseEnergy[2].consumption = data / 3600000;
                break;
              case 62:
                PhaseEnergy[2].gridfeedin = data / 3600000;
                break;
            }
          } else if (type == 4) {
            uint32_t data = (offset[0] << 24) + (offset[1] << 16) + (offset[2] << 8) + offset[3];
            offset += 4;
            switch (index) {
              case 1:
                // 1.4.0 Total grid power in dW - unused
                break;
              case 2:
                // 2.4.0 Total feed-in power in dW - unused
                break;
              case 21:
                PhasePower[0].power = round2(data * 0.1);
                PhasePower[0].frequency = defaultFrequency;
                break;
              case 22:
                PhasePower[0].power -= round2(data * 0.1);
                break;
              case 29:
                PhasePower[0].apparentPower = round2(data * 0.1);
                break;
              case 30:
                PhasePower[0].apparentPower -= round2(data * 0.1);
                break;
              case 31:
                PhasePower[0].current = round2(data * 0.001);
                break;
              case 32:
                PhasePower[0].voltage = round2(data * 0.001);
                break;
              case 33:
                PhasePower[0].powerFactor = round2(data * 0.001);
                break;
              case 41:
                PhasePower[1].power = round2(data * 0.1);
                PhasePower[1].frequency = defaultFrequency;
                break;
              case 42:
                PhasePower[1].power -= round2(data * 0.1);
                break;
              case 49:
                PhasePower[1].apparentPower = round2(data * 0.1);
                break;
              case 50:
                PhasePower[1].apparentPower -= round2(data * 0.1);
                break;
              case 51:
                PhasePower[1].current = round2(data * 0.001);
                break;
              case 52:
                PhasePower[1].voltage = round2(data * 0.001);
                break;
              case 53:
                PhasePower[1].powerFactor = round2(data * 0.001);
                break;
              case 61:
                PhasePower[2].power = round2(data * 0.1);
                PhasePower[2].frequency = defaultFrequency;
                break;
              case 62:
                PhasePower[2].power -= round2(data * 0.1);
                break;
              case 69:
                PhasePower[2].apparentPower = round2(data * 0.1);
                break;
              case 70:
                PhasePower[2].apparentPower -= round2(data * 0.1);
                break;
              case 71:
                PhasePower[2].current = round2(data * 0.001);
                break;
              case 72:
                PhasePower[2].voltage = round2(data * 0.001);
                break;
              case 73:
                PhasePower[2].powerFactor = round2(data * 0.001);
                break;
              default:
                break;
            }
          } else if (channel == 144) {
            // optional handling of version number
            offset += 4;
          } else {
            offset += type;
            DEBUG_SERIAL.println("Unknown measurement");
          }
        }
      } else if (grouptag == 0) {
        // end marker
        offset += grouplen;
      } else {
        DEBUG_SERIAL.print("unhandled group ");
        DEBUG_SERIAL.print(grouptag);
        DEBUG_SERIAL.print(" with len=");
        DEBUG_SERIAL.println(grouplen);
        offset += grouplen;
      }
    } while (grouplen > 0 && offset + 4 < buffer + rSize);
  }
}
