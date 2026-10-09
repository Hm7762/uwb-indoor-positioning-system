/*
 * ================================================================
 *  TAG — Real DS-TWR / SDS-TWR  (SmartHat)
 *  Makerfabs ESP32-UWB-DW3000
 *
 *  Genuine 3-message double-sided two-way ranging: this tag no
 *  longer computes range itself. For each anchor it sends POLL,
 *  receives that anchor's RESPONSE, then sends a unicast FINAL back
 *  to that SAME anchor delayed by the SAME interval the anchor used
 *  for RESPONSE (matched reply delays -> SDS-TWR-grade accuracy).
 *  The anchor computes the range from all 6 timestamps and publishes
 *  it directly over MQTT. This tag is now purely a timestamp relay —
 *  no clock-offset register reads, no REPORT aggregation/broadcast.
 *
 *  FLASH INSTRUCTIONS:
 *  Board 4: TAG_ID = 1  (equipment tag — stationary)
 *  Board 5: TAG_ID = 0  (helmet tag — worker wears this)
 *  Change ONLY the TAG_ID line before each flash.
 * ================================================================
 */

#include "dw3000.h"
#include "dw3000_config_options.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "../secrets.h"   // copy firmware/secrets.example.h -> firmware/secrets.h and fill in your network

// ── CHANGE BEFORE EACH FLASH ──────────────────────────────────
#define TAG_ID  0    // 0 = helmet tag,  1 = equipment tag

// ── Debug verbosity — keep OFF once timing is tight; per-cycle    ──
// Serial.printf at 115200 baud is blocking and can itself eat      ──
// multiple ms, which matters once TDMA_FRAME_MS is this short.     ──
#define DEBUG_VERBOSE  0

// ── DIAGNOSTIC MODE — OFF now. It did its job (found the attempt-1  ──
// forcetrxoff collision and confirmed A1's RESPONSE is now heard),
// but its own per-attempt Serial.printf() calls (hundreds of us each
// at 115200 baud) were eating directly into the 2500us window the tag
// has to notice A1's RESPONSE, process it, and schedule the matched-
// delay FINAL — causing "FINAL delayed TX FAIL (A1)" and starving A2
// (4000us, third in line) entirely. Turn back to 1 only if a new
// no-anchors-heard-at-all regression needs diagnosing again.
#define DIAG_MODE  0

// ── WiFi / MQTT ───────────────────────────────────────────────
// Defined in secrets.h (see the #include above) — not hardcoded here
// so real credentials never land in version control.

// ── Network ───────────────────────────────────────────────────
#define NET_PAN_ID        0xDECA
#define MASTER_ANCHOR_ID  0xA0
#define NUM_ANCHORS       3

// ── Function codes ────────────────────────────────────────────
#define FUNC_POLL      0xE0
#define FUNC_RESPONSE  0xE1
#define FUNC_FINAL     0xE4
#define FUNC_BEACON    0xE3

// ── TDMA timing — must match anchor.ino exactly ────────────────
// Shrunk from 1400ms/150ms — the old values meant each tag only
// updated ~0.7x/sec. These give each tag a slot roughly every
// 100ms (~10Hz). If FINAL sends start failing after flashing, widen
// SLOT_DURATION_MS / TDMA_FRAME_MS (and the delays below) — these
// are conservative starting points, not bench-verified on this
// hardware.
#define TDMA_FRAME_MS     100
#define SLOT_DURATION_MS   25
#define BEACON_GUARD_MS     20
#define BEACON_TIMEOUT_MS  (TDMA_FRAME_MS * 3)

// ── Anchor reply delay — must match RESP_DLY in anchor.ino ─────
// A0=1000us, A1=2500us, A2=6500us (NOT 4000 — see below). Reused for
// this tag's own FINAL reply delay to each anchor (matched Da<->Db is
// the SDS-TWR part).
//
// A2's delay was 4000us and that was the real bug behind "A2 never
// heard" (confirmed via A2's own Serial log: it reliably received
// every POLL and sent every RESPONSE — the tag just never caught it).
// Root cause: the DW3000 is half-duplex. Once the tag schedules its
// matched-delay FINAL back to an anchor, it's committed to that
// pending TX and can't receive anything until it actually fires. For
// A1: RESPONSE processed at ~2500us, FINAL fires at 2500+2500=5000us
// -> the tag is radio-deaf from ~2500us to ~5000us EVERY cycle. A2's
// own RESPONSE arrived at 4000us -- squarely inside that window.
// Deterministic collision, not noise (A0/A1 don't collide with each
// other: A0's busy window 1000->2000us closes with 500us to spare
// before A1's 2500us arrival). Fix: push A2 out past 5000us so its
// RESPONSE lands after the tag is listening again. 6500us gives
// 1500us of margin.
const uint32_t ANCHOR_DELAYS[NUM_ANCHORS] = { 1000, 2500, 6500 };

// ── Bounded wait for TX-complete (SYS_STATUS_TXFRS) ─────────────
// NOT unguarded — dwt_starttx(DWT_START_TX_DELAYED) can silently
// never set TXFRS (bad delayed time already passed, radio state
// glitch, etc). An unbounded `while(!TXFRS);` then hangs loop()
// forever, starving mqtt.loop()/keepalive — exactly the
// "disconnected: exceeded timeout" symptom seen on both tags.
// 10ms is generous (TX takes well under 1ms); if it's ever hit,
// something upstream (delayed time already elapsed) is wrong and
// we should bail, not hang.
#define TX_COMPLETE_TIMEOUT_MS 10

// ── Antenna delay ─────────────────────────────────────────────
// Left at the shared default — see anchor.ino header comment for why
// (software-side RANGE_OFFSETS correction in server.js instead).
#define TX_ANT_DLY  16385
#define RX_ANT_DLY  16385

// ── UWB pins ──────────────────────────────────────────────────
const uint8_t PIN_RST = DW3000_PIN_RST;
const uint8_t PIN_IRQ = DW3000_PIN_IRQ;
const uint8_t PIN_SS  = DW3000_PIN_CS;

// ── UWB config ────────────────────────────────────────────────
static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9,
  0,              /* sfdType=0 — required for ESP32 boards */
  DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
  (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// ── Message buffers ───────────────────────────────────────────
static uint8_t tx_poll_msg[] = {
  0x41, 0x88, 0,
  0, 0,       /* PAN ID (3-4) */
  0xFF, 0xFF, /* broadcast dst (5-6) */
  0, 0,       /* src tag ID (7-8) */
  FUNC_POLL,
  0, 0
};

// Unicast FINAL — addressed to one specific anchor per send.
static uint8_t tx_final_msg[] = {
  0x41, 0x88, 0,
  0, 0,       /* PAN ID (3-4) */
  0, 0,       /* dst anchor ID (5-6) */
  0, 0,       /* src tag ID (7-8) */
  FUNC_FINAL,
  0, 0, 0, 0, /* poll_tx_ts   (10-13) */
  0, 0, 0, 0, /* resp_rx_ts   (14-17) */
  0, 0, 0, 0, /* final_tx_ts  (18-21) */
  0, 0
};

#define BEACON_FRAME_IDX      11
#define BEACON_SLOT_IDX       13
#define RESP_POLL_RX_TS_IDX   10
#define RESP_TX_TS_IDX        14
#define FINAL_POLL_TX_TS_IDX  10
#define FINAL_RESP_RX_TS_IDX  14
#define FINAL_TX_TS_IDX       18
#define RX_BUF_LEN            48

static uint8_t  rxBuf[RX_BUF_LEN];
static uint32_t statusReg     = 0;
static uint8_t  seqNum        = 0;
static uint32_t syncedEpochMs = 0;
static uint32_t lastBeaconMs  = 0;
static bool     timeSynced    = false;
static bool     slotDone      = false;
static uint8_t  finalSentMask = 0;
static uint32_t cycleCount    = 0;
static uint32_t finalTxFail   = 0;

// ── Diagnostic counters (DIAG_MODE only) ────────────────────────
static uint32_t diagFramesSeen     = 0;  // any valid radio frame received
static uint32_t diagBeaconIdMatch  = 0;  // PAN+master+FUNC_BEACON matched
static uint32_t diagBeaconMismatch = 0;  // matched above but frame/slot dur wrong
static uint32_t diagLastPrintMs    = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

extern dwt_txconfig_t txconfig_options;

static inline uint16_t readU16(const uint8_t* d) {
  return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}
static inline void writeU16(uint8_t* d, uint16_t v) {
  d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8);
}

// ── Connectivity ──────────────────────────────────────────────
void wifiConnect() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[T%d] WiFi connecting", TAG_ID);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf(" OK — %s\n", WiFi.localIP().toString().c_str());
}

void mqttConnect() {
  char id[16]; snprintf(id, sizeof(id), "tag_%d", TAG_ID);
  while (!mqtt.connected()) {
    Serial.printf("[T%d] MQTT connecting...", TAG_ID);
    if (mqtt.connect(id)) Serial.println("OK");
    else { Serial.println("failed, retry 1s"); delay(1000); }
  }
}

// ── Listen for beacon ─────────────────────────────────────────
static void listenForBeacon(uint16_t timeoutMs) {
  dwt_forcetrxoff();
  dwt_setrxtimeout(timeoutMs * 1000UL);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Software-bounded (see waitForRxEvent comment) — a few ms of margin
  // over the requested hardware timeoutMs, not an unguarded spin.
  if (!waitForRxEvent((uint32_t)timeoutMs + 20)) {
    dwt_write32bitreg(SYS_STATUS_ID,
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK);
    return;
  }

  if (!(statusReg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return;
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t flen    = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  uint16_t dataLen = (uint16_t)(flen - FCS_LEN);
  if (dataLen == 0 || dataLen > RX_BUF_LEN) return;

  dwt_readrxdata(rxBuf, dataLen, 0);
  diagFramesSeen++;

  uint16_t panId = readU16(&rxBuf[3]);
  uint16_t srcId = readU16(&rxBuf[7]);
  uint8_t  func  = rxBuf[9];

#if DIAG_MODE
  Serial.printf("[T%d] DIAG rx#%lu pan=0x%04X src=0x%04X func=0x%02X len=%u\n",
    TAG_ID, diagFramesSeen, panId, srcId, func, dataLen);
#endif

  if (panId != NET_PAN_ID)       return;
  if (srcId != MASTER_ANCHOR_ID) return;
  if (func  != FUNC_BEACON)      return;

  uint16_t frameDur = readU16(&rxBuf[BEACON_FRAME_IDX]);
  uint16_t slotDur  = readU16(&rxBuf[BEACON_SLOT_IDX]);

  if (frameDur == TDMA_FRAME_MS && slotDur == SLOT_DURATION_MS) {
    diagBeaconIdMatch++;
    syncedEpochMs = millis();
    lastBeaconMs  = syncedEpochMs;
    timeSynced    = true;
    slotDone      = false;
    Serial.printf("[T%d] Beacon sync OK (frame=%u slot=%u)\n", TAG_ID, frameDur, slotDur);
  } else {
    diagBeaconMismatch++;
    Serial.printf("[T%d] DIAG beacon ID matched but TIMING MISMATCH: got frame=%ums slot=%ums, this tag expects frame=%dms slot=%dms — the anchor is very likely running OLDER/DIFFERENT firmware than this tag.\n",
      TAG_ID, frameDur, slotDur, TDMA_FRAME_MS, SLOT_DURATION_MS);
  }
}

// ── Bounded wait for an RX event (RXFCG / RX_TO / RX_ERR) ────────
// The DW3000 hardware rx-timeout register (dwt_setrxtimeout) is
// trusted to eventually set SYS_STATUS_ALL_RX_TO on its own, so the
// original code here was `while (!(...));` with no software bound.
// If that hardware timeout ever fails to fire (register race,
// driver/hardware quirk) — same failure class as the TX-complete
// bug already fixed — this hangs loop() forever, which matches the
// symptom of ranging cycles never completing/printing after a
// successful beacon sync. This adds a millis()-based backstop.
static bool waitForRxEvent(uint32_t swTimeoutMs) {
  uint32_t start = millis();
  while (millis() - start < swTimeoutMs) {
    statusReg = dwt_read32bitreg(SYS_STATUS_ID);
    if (statusReg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))
      return true;
  }
  return false;
}

// ── Handle one anchor RESPONSE: reply with a matched-delay FINAL ──
// Returns true iff this frame identified a NEW anchor (by payload
// content, not by which listen attempt it arrived on — see
// performRanging() rewrite: the old code assumed listen-attempt #i
// == anchor i, which broke down whenever one anchor was slow/late
// and a still-open window swallowed the next anchor's transmission).
static bool handleResponse(uint64_t pollTx64) {
  uint64_t respRx64 = get_rx_timestamp_u64();

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  uint32_t flen    = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  uint16_t dataLen = (uint16_t)(flen - FCS_LEN);
  if (dataLen == 0 || dataLen > RX_BUF_LEN) {
#if DIAG_MODE
    Serial.printf("[T%d] DIAG rejected RESPONSE: bad dataLen=%u\n", TAG_ID, dataLen);
#endif
    return false;
  }

  dwt_readrxdata(rxBuf, dataLen, 0);

#if DIAG_MODE
  uint8_t  gotFunc = rxBuf[9];
  uint16_t gotPan  = readU16(&rxBuf[3]);
  uint16_t gotDst  = readU16(&rxBuf[5]);
  uint16_t gotSrc  = readU16(&rxBuf[7]);
  if (gotFunc != FUNC_RESPONSE || gotPan != NET_PAN_ID || gotDst != (uint16_t)TAG_ID) {
    Serial.printf("[T%d] DIAG rejected RESPONSE: func=0x%02X pan=0x%04X dst=0x%04X src=0x%04X len=%u (expected func=0x%02X pan=0x%04X dst=0x%04X)\n",
      TAG_ID, gotFunc, gotPan, gotDst, gotSrc, dataLen, FUNC_RESPONSE, NET_PAN_ID, (uint16_t)TAG_ID);
  }
#endif

  if (rxBuf[9]           != FUNC_RESPONSE)    return false;
  if (readU16(&rxBuf[3]) != NET_PAN_ID)       return false;
  if (readU16(&rxBuf[5]) != (uint16_t)TAG_ID) return false;

  uint16_t anchorId  = readU16(&rxBuf[7]);
  int      anchorIdx = (int)(anchorId & 0x0F);
  if (anchorIdx < 0 || anchorIdx >= NUM_ANCHORS) {
#if DIAG_MODE
    Serial.printf("[T%d] DIAG rejected RESPONSE: anchorId=0x%04X out of range\n", TAG_ID, anchorId);
#endif
    return false;
  }

  // Already handled this anchor this cycle (e.g. a stray duplicate,
  // or the collision pattern above) — don't re-schedule a FINAL.
  if (finalSentMask & (1 << anchorIdx)) {
#if DIAG_MODE
    Serial.printf("[T%d] DIAG A%d duplicate RESPONSE ignored this cycle\n", TAG_ID, anchorIdx);
#endif
    return false;
  }

  // Schedule FINAL back to this same anchor, delayed by the same
  // interval it used for RESPONSE (matched delay -> SDS-TWR).
  uint32_t finalTxTime =
    (uint32_t)((respRx64 + ((uint64_t)ANCHOR_DELAYS[anchorIdx] * UUS_TO_DWT_TIME)) >> 8);
  dwt_setdelayedtrxtime(finalTxTime);
  uint64_t finalTxTs = (((uint64_t)(finalTxTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  tx_final_msg[2] = rxBuf[2];
  tx_final_msg[5] = (uint8_t)(anchorId);
  tx_final_msg[6] = (uint8_t)(anchorId >> 8);

  resp_msg_set_ts(&tx_final_msg[FINAL_POLL_TX_TS_IDX], pollTx64);
  resp_msg_set_ts(&tx_final_msg[FINAL_RESP_RX_TS_IDX], respRx64);
  resp_msg_set_ts(&tx_final_msg[FINAL_TX_TS_IDX],      finalTxTs);

  dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
  dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

  int ret = dwt_starttx(DWT_START_TX_DELAYED);
  if (ret == DWT_SUCCESS) {
    // REVERTED the earlier "don't wait for TXFRS" experiment. It was
    // tested (identical A0/A1/A2 outcomes with and without it) and
    // gave no timing benefit — but it left a real hazard: tx_final_msg
    // is ONE shared buffer. Without waiting here, a second anchor's
    // RESPONSE arriving soon after (now common with the continuous-
    // listen loop) calls handleResponse() again and immediately
    // overwrites this same buffer + reschedules a new delayed TX,
    // potentially while THIS FINAL is still pending/mid-transmission —
    // which can corrupt or cancel it. That's the likely cause of
    // anchors publishing out-of-range garbage distances (server-side
    // "FAIL" entries) after that experiment. Bounded wait restored.
    uint32_t txWaitStart = millis();
    bool txOk = false;
    while (millis() - txWaitStart < TX_COMPLETE_TIMEOUT_MS) {
      if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) { txOk = true; break; }
    }
    if (txOk) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      finalSentMask |= (1 << anchorIdx);
#if DIAG_MODE
      Serial.printf("[T%d] DIAG FINAL -> A%d sent (confirmed on-air)\n", TAG_ID, anchorIdx);
#endif
      return true;
    } else {
      finalTxFail++;
      dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
      Serial.printf("[T%d] FINAL TX timed out waiting for TXFRS #%lu (A%d)\n", TAG_ID, finalTxFail, anchorIdx);
      return false;
    }
  } else {
    finalTxFail++;
    dwt_write32bitreg(SYS_STATUS_ID,
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
    Serial.printf("[T%d] FINAL delayed TX FAIL #%lu (A%d)\n", TAG_ID, finalTxFail, anchorIdx);
    return false;
  }
}

// ── Full ranging cycle ────────────────────────────────────────
static void performRanging() {
  finalSentMask = 0;
#if DIAG_MODE
  Serial.printf("[T%d] DIAG performRanging enter\n", TAG_ID);
#endif

  // Send POLL
  tx_poll_msg[2] = seqNum;
  dwt_setrxaftertxdelay(100);
  dwt_setrxtimeout(ANCHOR_DELAYS[0] + 2000);
  dwt_write32bitreg(SYS_STATUS_ID,
    SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  seqNum++;

  // BUG FOUND: get_tx_timestamp_u64() was being read immediately after
  // dwt_starttx(), with no wait for the transmission to actually finish.
  // The DW3000's TX timestamp register isn't guaranteed valid until
  // TXFRS is set — reading it early can return a stale value from a
  // previous TX, or an unsettled one. pollTx64 feeds Ra/Da for EVERY
  // anchor's range calculation (it's the pollTxWire field sent in every
  // FINAL) — a wrong value here corrupts all three anchors' distances
  // identically, which is exactly what "0/3 fresh" showed (not an
  // A2-specific problem). Same bounded-wait pattern already used
  // elsewhere in this file, just never applied to the POLL's own TX.
  uint32_t pollTxWaitStart = millis();
  while (millis() - pollTxWaitStart < TX_COMPLETE_TIMEOUT_MS) {
    if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) break;
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  uint64_t pollTx64 = get_tx_timestamp_u64();

  // ── Continuous listen, not three fixed "slot #i == anchor i" windows ──
  // The old code assumed listen-attempt #1 == A0, #2 == A1, #3 == A2,
  // each with a timeout sized from the GAP between that anchor's and
  // the previous anchor's RESP_DLY. That breaks down the moment one
  // anchor is late/missed: the still-open window for it can swallow
  // the NEXT anchor's real transmission instead (we saw this directly:
  // a "slot 1" reception decoding as anchor 1's payload). Instead,
  // keep re-arming RX in a single stretch covering the full expected
  // response period (up to the last anchor's RESP_DLY + margin) and
  // identify each response by its own payload, however many attempts
  // it takes, until all NUM_ANCHORS are seen or the deadline passes.
  uint32_t rxAttemptTimeoutUs = ANCHOR_DELAYS[NUM_ANCHORS - 1] + 3000;
  // A2's own FINAL now fires at 2*6500=13000us (~13ms) post-POLL — bump
  // the overall bound from 15 to 20ms so that isn't cutting it close.
  uint32_t cycleDeadlineMs    = millis() + 20;   // overall software bound
  int      gotCount           = 0;
  int      attempt            = 0;

  while (gotCount < NUM_ANCHORS && millis() < cycleDeadlineMs) {
    attempt++;
    // Force the transceiver off before each re-arm. Evidence: attempt #1
    // (the very first rxenable after the POLL TX) reliably sets a status
    // bit, but a SECOND rxenable issued later in the same performRanging()
    // call — right after handleResponse() did its own RX-status write and
    // (on a matched anchor) a further TX for the FINAL — 100% reliably
    // leaves the DW3000 never setting RXFCG/RX_TO/RX_ERR at all, forcing
    // the 15ms software backstop every time. That is a stuck-receiver
    // state, not a real absence of anchors: a rapid rxenable-after-recent-
    // tx/rx without an intervening clean transceiver-off is a known rough
    // edge in DW1000/DW3000 driver usage. forcetrxoff() resets TRX state
    // to idle before every re-arm, which is the standard fix for exactly
    // this "receiver wedged after back-to-back operations" symptom.
    // Only force-idle before attempt #2+. Attempt #1 follows a POLL sent
    // with DWT_RESPONSE_EXPECTED + dwt_setrxaftertxdelay(100), which
    // auto-arms RX in hardware right after that TX completes — calling
    // forcetrxoff() here steps on that handoff (confirmed by testing:
    // attempt #1, which was 100% reliable before forcetrxoff was added
    // to every iteration, started failing every cycle once it was added
    // here too). Attempt #2+ follows handleResponse()'s own TX, which is
    // explicitly waited-for and confirmed complete (TXFRS) before this
    // loop continues — no pending hardware auto-arm there, so
    // forcetrxoff() is safe and still needed to fix the original
    // stuck-receiver symptom on the second+ re-arm.
    if (attempt > 1) {
      dwt_forcetrxoff();
    }
    dwt_setrxtimeout(rxAttemptTimeoutUs);
    // rxenable's return code was previously ignored. If it's non-zero the
    // DW3000 REJECTED the enable call outright (chip not settled/ready) —
    // that alone explains "hardware never sets any status bit": the
    // receiver was never actually turned on, so nothing could ever fire.
    // If it's 0 here but we still hit the software timeout below, the
    // enable was accepted and the stall is happening somewhere else
    // (different root cause than forcetrxoff can fix).
    int rxEnableRc = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (rxEnableRc != 0) {
      Serial.printf("[T%d] DIAG listen attempt #%d: dwt_rxenable() REJECTED, rc=%d\n", TAG_ID, attempt, rxEnableRc);
    }

    if (!waitForRxEvent(15)) {
      Serial.printf("[T%d] DIAG listen attempt #%d: SOFTWARE TIMEOUT (hardware never set RXFCG/RX_TO/RX_ERR) rxEnableRc=%d\n", TAG_ID, attempt, rxEnableRc);
      dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK);
      break;  // hardware truly stuck — no point retrying within this cycle
    }
#if DIAG_MODE
    Serial.printf("[T%d] DIAG listen attempt #%d status=0x%08lX (have=0x%02X)\n",
      TAG_ID, attempt, (unsigned long)statusReg, finalSentMask);
#endif

    if (statusReg & SYS_STATUS_RXFCG_BIT_MASK) {
      if (handleResponse(pollTx64)) gotCount++;
    } else {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      // A hardware RX timeout/error here just means nothing arrived on
      // THIS attempt — keep looping until the overall deadline, since
      // a later anchor may still be about to transmit.
    }
  }

  cycleCount++;
  Serial.printf("[T%d] cycle=%lu finalSentMask=0x%02X\n",
    TAG_ID, cycleCount, finalSentMask);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== TAG %d (DS/SDS-TWR) ===\n", TAG_ID);
  Serial.printf("ANT_DELAY=%d  FRAME=%dms SLOT=%dms\n", TX_ANT_DLY, TDMA_FRAME_MS, SLOT_DURATION_MS);

  tx_poll_msg[3]  = (uint8_t)(NET_PAN_ID);
  tx_poll_msg[4]  = (uint8_t)(NET_PAN_ID >> 8);
  tx_poll_msg[7]  = (uint8_t)(TAG_ID);
  tx_poll_msg[8]  = (uint8_t)(TAG_ID >> 8);
  tx_final_msg[3] = (uint8_t)(NET_PAN_ID);
  tx_final_msg[4] = (uint8_t)(NET_PAN_ID >> 8);
  tx_final_msg[7] = (uint8_t)(TAG_ID);
  tx_final_msg[8] = (uint8_t)(TAG_ID >> 8);

  UART_init();
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(2);

  dwt_softreset();
  delay(2);

  while (!dwt_checkidlerc()) { Serial.println("DW3000 idle wait..."); delay(200); }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAILED"); while(1) delay(100);
  }
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  if (dwt_configure(&config)) {
    Serial.println("CONFIG FAILED"); while(1) delay(100);
  }
  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  wifiConnect();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqttConnect();

  // Tag 1 starts 500ms after Tag 0 to avoid TDMA collision
  if (TAG_ID == 1) delay(500);

  Serial.printf("[T%d] Ready — waiting for beacon\n", TAG_ID);
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  if (!timeSynced) {
    listenForBeacon(200);
#if DIAG_MODE
    uint32_t nowDiag = millis();
    if (nowDiag - diagLastPrintMs > 3000) {
      diagLastPrintMs = nowDiag;
      Serial.printf("[T%d] DIAG heartbeat: framesSeen=%lu beaconIdMatch=%lu beaconMismatch=%lu wifi=%d mqtt=%d\n",
        TAG_ID, diagFramesSeen, diagBeaconIdMatch, diagBeaconMismatch,
        WiFi.status() == WL_CONNECTED, mqtt.connected());
    }
#endif
    return;
  }

  uint32_t nowMs = millis();

  // Re-sync if beacon lost
  if (nowMs - lastBeaconMs > BEACON_TIMEOUT_MS) {
    timeSynced = false;
    slotDone   = false;
    Serial.printf("[T%d] Beacon lost — resyncing\n", TAG_ID);
    return;
  }

  uint32_t elapsed   = nowMs - syncedEpochMs;
  uint32_t frameOff  = elapsed % TDMA_FRAME_MS;
  uint32_t slotStart = BEACON_GUARD_MS + TAG_ID * SLOT_DURATION_MS;
  uint32_t slotEnd   = slotStart + SLOT_DURATION_MS;

  if (frameOff >= slotStart && frameOff < slotEnd && !slotDone) {
    performRanging();
    slotDone = true;
  }

  if (frameOff < slotStart || frameOff >= slotEnd) {
    slotDone = false;
    listenForBeacon(5);
  }

  delay(1);
}
