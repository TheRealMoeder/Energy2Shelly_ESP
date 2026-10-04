#include "../config/Configuration.h"
#include "../data/DataProcessing.h"
#include "../data/DataStructures.h"

// ============================================================================
// RCT Power "Serial Communication Protocol" parser (TCP)
//
// Protocol information see (see https://rctclient.readthedocs.io/en/latest/):
// The device behaves like a shared bus: every connected client also sees the
// frames of every other client (the official app works locally and polls the 
// OIDs), including WRITEs and string responses for OIDs nobody asked us about. 
// Frame order is therefore arbitrary - the response for the OID we just requested
// is rarely the next frame - and a lone reader can be ignored altogether
// while the app session carries the data. We treat the connection as a
// stream:
//  - send a READ for every value we track each poll,
//  - consume the stream, accepting RESPONSE frames for any tracked OID and
//    ignoring everything else (CRC-damaged frames, WRITEs, other clients'
//    strings and responses),
//  - never drop the connection because a frame does not match,
//  - drop it only when the socket died, and reconnect on the next poll,
//  - per-slot: keep the last good value, so a partially responsive device
//    still feeds the emulator instead of dropping it to zero.
// ============================================================================
// Receive timeout per read request. The reference implementations (rctclient
// and the Tinkerforge meter) use 2 s; real devices can answer slowly when
// busy. Worst case per poll cycle is a single silent wait on a quiet
// connection (all tracked OIDs asked at once), well within both the poll
// period and the watchdog budget.

#define RCT_RX_TIMEOUT_MS 2000    // per-frame receive window
#define RCT_CYCLE_TIMEOUT_MS 4000 // total per-poll collection budget

static WiFiClient rctClient;

// ---------------------------------------------------------------------------
// Protocol helpers
// ---------------------------------------------------------------------------

// CRC16 as implemented by the rctclient reference (see rctclient.utils.CRC16).
static uint16_t rctCrc16(const uint8_t *data, size_t len)
{
  uint32_t crcsum = 0xFFFF;
  const uint32_t polynom = 0x1021;
  size_t paddedLen = len + (len & 0x01); // append 0x00 if length is odd

  for (size_t i = 0; i < paddedLen; i++)
  {
    uint8_t byte = (i < len) ? data[i] : 0x00;
    crcsum ^= ((uint32_t)byte) << 8;
    for (int j = 0; j < 8; j++)
    {
      crcsum <<= 1;
      if (crcsum & 0x7FFF0000)
      {
        crcsum = (crcsum & 0x0000FFFF) ^ polynom;
      }
    }
  }
  return (uint16_t)(crcsum & 0xFFFF);
}

// Build and send a READ frame for an OID.
static bool rctSendRead(uint32_t oid)
{
  uint8_t frame[8];
  frame[0] = 0x01; // READ
  frame[1] = 0x04; // length: 4 OID bytes
  frame[2] = (uint8_t)(oid >> 24);
  frame[3] = (uint8_t)(oid >> 16);
  frame[4] = (uint8_t)(oid >> 8);
  frame[5] = (uint8_t)(oid);

  uint16_t crc = rctCrc16(frame, 6);
  frame[6] = (uint8_t)(crc >> 8);
  frame[7] = (uint8_t)(crc & 0xFF);

  uint8_t out[18]; // 2b + 8 bytes, each possibly escaped by 0x2d
  size_t oi = 0;
  out[oi++] = 0x2b;
  for (size_t i = 0; i < sizeof(frame); i++)
  {
    if (frame[i] == 0x2b || frame[i] == 0x2d)
    {
      out[oi++] = 0x2d;
    }
    out[oi++] = frame[i];
  }
  return rctClient.write(out, oi) == oi;
}

// Incremental receive state machine. De-escaped bytes are accumulated in
// rctRxBuf starting with the 0x2b start token. 64 bytes fit the largest
// frames the device can send on a shared connection (other clients' WRITEs
// and string payloads can exceed the 5 + 4 + 4 = 13 bytes of a grid value).
#define RCT_RX_BUF_SIZE 64
static uint8_t rctRxBuf[RCT_RX_BUF_SIZE];
static size_t rctRxLen = 0;
static bool rctRxEscaping = false;
static bool rctRxComplete = false;
static size_t rctRxTotal = 0;

static void rctProcessByte(uint8_t c)
{
  if (rctRxLen == 0)
  {
    if (c == 0x2b)
    {
      rctRxBuf[0] = c;
      rctRxLen = 1;
      rctRxEscaping = false;
      rctRxComplete = false;
      rctRxTotal = 0;
    }
    return;
  }

  if (rctRxEscaping)
  {
    rctRxEscaping = false;
  }
  else if (c == 0x2d)
  {
    rctRxEscaping = true;
    return;
  }

  rctRxBuf[rctRxLen++] = c;

  if (rctRxLen == 3)
  {
    // header complete: 2b <command> <length>; length counts OID + payload
    rctRxTotal = 5 + rctRxBuf[2]; // 2b + cmd + len + oid + payload + crc
  }

  if (rctRxTotal > 0 && rctRxLen >= rctRxTotal)
  {
    rctRxComplete = true;
    return;
  }

  // Unexpectedly large frame (e.g. a string payload we never request):
  // drop it and resync on the next start token.
  if (rctRxLen >= RCT_RX_BUF_SIZE)
  {
    rctRxLen = 0;
    rctRxEscaping = false;
  }
}

// Receive result codes
enum RCT_RX : int { RCT_RX_OK = 0, RCT_RX_TIMEOUT, RCT_RX_CRC };

// Wait for and validate one response frame. Returns RCT_RX_OK on success,
// RCT_RX_TIMEOUT when no frame arrived within the receive window (the caller
// can check rctClient.connected() to see whether the peer closed the
// connection) and RCT_RX_CRC when a frame arrived but its checksum does not
// match. Resets the receive state in all cases.
static int rctReceiveFrame(uint8_t &command, uint32_t &oid,
                           uint8_t *payload, size_t payloadCapacity, size_t &payloadLen)
{
  unsigned long startMillisHere = millis();
  while (millis() - startMillisHere < RCT_RX_TIMEOUT_MS)
  {
    while (rctClient.available())
    {
      rctProcessByte(rctClient.read());
      if (rctRxComplete)
      {
        break;
      }
    }
    if (rctRxComplete)
    {
      break;
    }
    if (!rctClient.connected())
    {
      // Peer closed the connection while we were waiting for a response.
      rctRxLen = 0;
      rctRxEscaping = false;
      rctRxComplete = false;
      rctRxTotal = 0;
      return RCT_RX_TIMEOUT;
    }
    delay(1);
  }

  if (!rctRxComplete)
  {
    rctRxLen = 0;
    rctRxEscaping = false;
    rctRxComplete = false;
    rctRxTotal = 0;
    return RCT_RX_TIMEOUT;
  }

  // CRC covers everything after the start token, up to the checksum bytes
  uint16_t calc = rctCrc16(&rctRxBuf[1], rctRxTotal - 3);
  uint16_t recv = ((uint16_t)rctRxBuf[rctRxTotal - 2] << 8) | rctRxBuf[rctRxTotal - 1];
  if (calc != recv)
  {
#if DEBUG
    DEBUG_SERIAL.printf("RCT:   CRC mismatch: calculated 0x%04X, received 0x%04X\n", calc, recv);
#endif
    rctRxLen = 0;
    rctRxEscaping = false;
    rctRxComplete = false;
    rctRxTotal = 0;
    return RCT_RX_CRC;
  }

  command = rctRxBuf[1];
  oid = ((uint32_t)rctRxBuf[3] << 24) | ((uint32_t)rctRxBuf[4] << 16) |
        ((uint32_t)rctRxBuf[5] << 8) | rctRxBuf[6];
  payloadLen = rctRxBuf[2] - 4;

  // Copy only what fits the caller's buffer. Oversized frames (device
  // strings, big WRITE payloads from other clients) stay untouched; the
  // caller filters them out via payloadLen anyway. This keeps an oversized
  // length byte from overflowing the stack buffer.
  if (payload != nullptr && payloadLen > 0 && payloadLen <= payloadCapacity)
  {
    memcpy(payload, &rctRxBuf[7], payloadLen);
  }

  rctRxLen = 0;
  rctRxEscaping = false;
  rctRxComplete = false;
  rctRxTotal = 0;

  return RCT_RX_OK;
}

// Decode a big-endian 4-byte IEEE-754 float (reference: rctclient.decode_value).
static float rctDecodeFloat(const uint8_t *p)
{
  uint32_t i = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
  float f;
  memcpy(&f, &i, sizeof(f));
  return f;
}

// ---------------------------------------------------------------------------
// Values we track (grid meter). Slot order groups contiguous slices so the
// apply helper can pass &rctCur[RCT_SLOT_P0] etc. directly.
// ---------------------------------------------------------------------------
enum RCT_SLOT
{
  RCT_SLOT_P0 = 0, // g_sync.p_ac_sc[0]       grid power L1 [W]
  RCT_SLOT_P1,     // g_sync.p_ac_sc[1]       grid power L2 [W]
  RCT_SLOT_P2,     // g_sync.p_ac_sc[2]       grid power L3 [W]
  RCT_SLOT_EFEED,  // energy.e_grid_feed_total raw, already Wh (rctclient registry)
  RCT_SLOT_ELOAD,  // energy.e_grid_load_total raw, already Wh (rctclient registry)
  RCT_SLOT_V0,     // rb485.u_l_grid[0]       grid voltage L1 [V]
  RCT_SLOT_V1,     // rb485.u_l_grid[1]       grid voltage L2 [V]
  RCT_SLOT_V2,     // rb485.u_l_grid[2]       grid voltage L3 [V]
  RCT_SLOT_F0,     // rb485.f_grid[0]         grid frequency L1 [Hz]
  RCT_SLOT_F1,     // rb485.f_grid[1]         grid frequency L2 [Hz]
  RCT_SLOT_F2,     // rb485.f_grid[2]         grid frequency L3 [Hz]
  RCT_SLOT_I0,     // g_sync.i_dr_eff[0]      grid current L1 [A]
  RCT_SLOT_I1,     // g_sync.i_dr_eff[1]      grid current L2 [A]
  RCT_SLOT_I2,     // g_sync.i_dr_eff[2]      grid current L3 [A]
  RCT_NUM_SLOTS
};

// Object IDs (see link above)
static const uint32_t rctOids[RCT_NUM_SLOTS] = {
  0x27BE51D9, // grid power L1 (W)
  0xF5584F90, // grid power L2 (W)
  0xB221BCFA, // grid power L3 (W)
  0x44D4C533, // feed-in energy (Wh)
  0x62FBE7DC, // load energy (Wh)
  0x93F976AB, // grid voltage L1 (V)
  0x7A9091EA, // grid voltage L2 (V)
  0x21EE7CBB, // grid voltage L3 (V)
  0x9558AD8A, // grid frequency L1 (Hz)
  0xFAE429C5, // grid frequency L2 (Hz)
  0x0104EB6A, // grid frequency L3 (Hz)
  0x89EE3EB5, // grid current L1 (A)
  0x650C1ED7, // grid current L2 (A)
  0x92BC682B, // grid current L3 (A)
};

// Slot index for a RESPONSE frame's OID, or -1 if we do not track it.
static int rctSlotForOid(uint32_t oid)
{
  for (int i = 0; i < RCT_NUM_SLOTS; i++)
  {
    if (rctOids[i] == oid)
    {
      return i;
    }
  }
  return -1;
}

// The official RCT Power app sends this frame once after connecting to switch
// the device into request/response ("COM") mode:
//   0x2b 0x3c 0xe1 = start token, EXTENSION command, single payload byte.
// Most devices (and the rctclient library) work without it, but some firmware
// versions stay silent for plain READs until they received it. No response is
// expected; any bytes the device sends back (a greeting/ack) are drained so
// the following READ frames line up.
static void rctSendExtension()
{
  static const uint8_t ext[] = {0x2b, 0x3c, 0xe1};
  rctClient.write(ext, sizeof(ext));
  unsigned long startMillisHere = millis();
  while (millis() - startMillisHere < 300)
  {
    while (rctClient.available())
    {
      rctClient.read();
    }
    delay(1);
  }
}

// ---------------------------------------------------------------------------
// Current best-known values. Every poll refreshes a subset of these slots
// from the stream; the rest keep their last good value so a partially
// responsive device still feeds the emulator instead of dropping to zero.
// ---------------------------------------------------------------------------
static float rctCur[RCT_NUM_SLOTS];
static bool rctHaveAny = false;
// Set once a slot has ever been answered, so we can tell "measured 0 A" apart
// from "never received" and fall back to the derived current in the latter case.
static bool rctSlotSeen[RCT_NUM_SLOTS];

static void rctApplyValues()
{
  const float *powers = &rctCur[RCT_SLOT_P0];
  const float *energies = &rctCur[RCT_SLOT_EFEED];
  const float *voltages = &rctCur[RCT_SLOT_V0];
  const float *frequencies = &rctCur[RCT_SLOT_F0];
  const float *currents = &rctCur[RCT_SLOT_I0];

  // Grid-meter sign conventions (matches the Shelly 3EM): positive power =
  // consuming from the grid. The RCT energy counters
  // (energy.e_grid_feed_total / e_grid_load_total) are absolute counters in Wh
  // already (rctclient registry, no scale factor); the Shelly 3EM API and this
  // emulator's web UI also work in Wh, so no scaling is applied - only the
  // feed-in direction is negated. setPowerData handles phase_number and
  // power_offset internally.
  setPowerData(powers[0], powers[1], powers[2]);
  double feedInWh = -(double)energies[0];
  double loadWh = (double)energies[1];
  setEnergyData(loadWh, feedInWh);

  // Overwrite the defaulted electrical values with the metered ones. Current is
  // taken from the grid meter's own current sensors (g_sync.i_dr_eff) rather
  // than derived from power/voltage, which is only correct for PF == 1. If a
  // device never answers the current OIDs we keep the derived value so the
  // emulator still reports a plausible current instead of 0 A.
  for (int i = 0; i < 3; i++)
  {
    PhasePower[i].voltage = voltages[i];
    if (rctSlotSeen[RCT_SLOT_I0 + i] && voltages[i] > 0.0)
    {
      // Metered current: keep the Shelly triple self-consistent. setPowerData
      // assumes PF == 1 (S = |P| and I = P/V), which stops holding once we use
      // a real current reading, so derive apparent power and PF from the
      // metered V, I and active power instead of leaving them at the defaults.
      PhasePower[i].current = currents[i];
      double apparent = (double)voltages[i] * (double)currents[i];
      if (apparent < 0.0)
      {
        apparent = -apparent;
      }
      PhasePower[i].apparentPower = round2(apparent);
      // Shelly reports pf as an unsigned 0..1 factor; the direction of the flow
      // is carried by the sign of act_power, so use the magnitude here.
      double active = (PhasePower[i].power < 0.0) ? -PhasePower[i].power : PhasePower[i].power;
      double pf = (apparent > 0.0) ? (active / apparent) : 1.0;
      if (pf > 1.0)
      {
        pf = 1.0;
      }
      PhasePower[i].powerFactor = round2(pf);
    }
    else if (voltages[i] > 0.0)
    {
      PhasePower[i].current = PhasePower[i].power / voltages[i];
    }
    PhasePower[i].frequency = frequencies[i];
  }
}

// ---------------------------------------------------------------------------
// Data source entry point (called from main.cpp worker_loop)
// ---------------------------------------------------------------------------

void parseRCT()
{
  if (!rctClient.connected())
  {
    int port = atol(rct_port);
    if (port <= 0)
    {
      port = 8899;
    }
    DEBUG_SERIAL.print(F("RCT: connecting to "));
    DEBUG_SERIAL.print(rct_host);
    DEBUG_SERIAL.print(F(":"));
    DEBUG_SERIAL.println(port);
    if (!rctClient.connect(rct_host, port))
    {
      DEBUG_SERIAL.println(F("RCT: connect failed"));
      return;
    }
#if DEBUG
    // Confirm the address actually reached the device (IP or hostname).
    DEBUG_SERIAL.print(F("RCT:   connected, peer IP "));
    DEBUG_SERIAL.println(rctClient.remoteIP());
#endif
    // Give the freshly established TCP connection a moment to settle before
    // the first request (avoids races on some stacks).
    delay(20);
    // Switch the device into request/response mode the way the official app
    // does, and discard any greeting/ack bytes it may send back.
    rctSendExtension();
  }
  // On a reused connection we deliberately do NOT drain: the device serves a
  // shared stream, so bytes left over from the previous poll may already be
  // the responses we want.

  // Ask for every value we track. The device answers all, some, or none of
  // them (e.g. while another client holds it); whatever it answers is served
  // onto every open connection, in any order.
  for (int i = 0; i < RCT_NUM_SLOTS; i++)
  {
    rctSendRead(rctOids[i]);
  }

  // Consume the stream for this poll: accept RESPONSE frames for tracked
  // OIDs, skip everything else (other clients' WRITEs/strings, CRC-damaged
  // frames, responses for OIDs we do not track) without tearing the
  // connection down.
  uint32_t freshMask = 0;
  const uint32_t allSlots = (1u << RCT_NUM_SLOTS) - 1;
  unsigned long deadline = millis() + RCT_CYCLE_TIMEOUT_MS;
  bool streamQuiet = false;
  while (freshMask != allSlots && !streamQuiet && (int32_t)(millis() - deadline) < 0)
  {
    uint8_t command = 0;
    uint8_t payload[64];
    size_t payloadLen = 0;
    uint32_t respOid = 0;
    int rc = rctReceiveFrame(command, respOid, payload, sizeof(payload), payloadLen);
    if (rc == RCT_RX_TIMEOUT)
    {
      if (!rctClient.connected())
      {
        // Peer closed the socket (the device tolerates few clients and may
        // evict us); reconnect on the next poll.
        rctClient.stop();
      }
      streamQuiet = true;
      break;
    }
    if (rc == RCT_RX_CRC)
    {
      // Damaged frame; the state machine already resynced on the next start
      // token. Skip it - nothing worth dropping the connection over.
      continue;
    }
    if (command == 0x05 && payloadLen == 4)
    {
      int slot = rctSlotForOid(respOid);
      if (slot >= 0)
      {
        rctCur[slot] = rctDecodeFloat(payload);
        freshMask |= (1u << slot);
        rctSlotSeen[slot] = true;
        rctHaveAny = true;
      }
    }
  }

  if (rctHaveAny)
  {
    // Apply the current best-known values. Slots not refreshed this poll
    // keep their last good value, so a partially responsive device still
    // feeds the emulator. Back-to-back identical setPowerData/setEnergyData
    // calls are idempotent.
    rctApplyValues();

    // Count how many values were refreshed this poll.
    int freshCount = 0;
    for (uint32_t m = freshMask; m; m &= m - 1)
    {
      freshCount++;
    }
    DEBUG_SERIAL.printf("RCT: grid L1/L2/L3: %.1f/%.1f/%.1f W, %.1f/%.1f/%.1f V, %.2f/%.2f/%.2f A (%d/%d fresh)\n",
                        rctCur[RCT_SLOT_P0], rctCur[RCT_SLOT_P1], rctCur[RCT_SLOT_P2],
                        rctCur[RCT_SLOT_V0], rctCur[RCT_SLOT_V1], rctCur[RCT_SLOT_V2],
                        rctCur[RCT_SLOT_I0], rctCur[RCT_SLOT_I1], rctCur[RCT_SLOT_I2],
                        freshCount, RCT_NUM_SLOTS);
    if (freshCount < RCT_NUM_SLOTS)
    {
      DEBUG_SERIAL.print(F("RCT:   missing this poll:"));
      for (int i = 0; i < RCT_NUM_SLOTS; i++)
      {
        if (!(freshMask & (1u << i)))
        {
          DEBUG_SERIAL.printf(" %08X", rctOids[i]);
        }
      }
      DEBUG_SERIAL.println();
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("RCT: no data yet (device unresponsive or only answers while the app is connected)"));
  }
}

