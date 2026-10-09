/*
 * ================================================================
 *  ANCHOR — Real DS-TWR / SDS-TWR  (SmartHat)
 *  Makerfabs ESP32-UWB-DW3000
 *
 *  Genuine 3-message double-sided two-way ranging:
 *    POLL (tag->all anchors) -> RESPONSE (this anchor->tag, delayed)
 *    -> FINAL (tag->this anchor, delayed by the SAME interval the
 *    anchor used for RESPONSE, which is what makes this SDS- rather
 *    than plain asymmetric DS-TWR: matched reply delays cancel the
 *    clock-drift error term to a much smaller residual than the old
 *    single-exchange + hardware-clock-offset-register approach).
 *  Every anchor now computes its own range and publishes it directly
 *  over MQTT — there is no more "master hears tag's broadcast REPORT"
 *  single point of failure.
 *
 *  FLASH INSTRUCTIONS:
 *  Board 1: ANCHOR_ID = 0xA0  (master — sends beacons only)
 *  Board 2: ANCHOR_ID = 0xA1
 *  Board 3: ANCHOR_ID = 0xA2
 *  Change ONLY the ANCHOR_ID line before each flash.
 *  All other settings identical across all 3 anchor boards.
 * ================================================================
 */

#include "dw3000.h"
#include "dw3000_config_options.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "../secrets.h"   // copy firmware/secrets.example.h -> firmware/secrets.h and fill in your network

// ── CHANGE BEFORE EACH FLASH ──────────────────────────────────
// Set to the ID of the specific board you're flashing before each
// upload. 0xA0 is the master (sends beacons); 0xA1 and 0xA2 are the
// other two anchors. Nothing else in this file changes per board.
#define ANCHOR_ID    0xA0   // 0xA0, 0xA1, or 0xA2

// ── Debug verbosity — OFF now. Same reasoning as tag.ino's DIAG_MODE: ──
// the tag-side log proved these per-cycle prints were themselves
// eating into the sub-millisecond RESP_DLY/FINAL_RX_MARGIN_US budget.
// Turn back to 1 only if a new problem needs diagnosing on this board.
#define DEBUG_VERBOSE  0

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

// ── TDMA timing ───────────────────────────────────────────────
// Shrunk from 1400ms/150ms — the old values meant each tag only
// updated ~0.7x/sec with ~1.08s of dead air per frame. These values
// give each tag a ranging slot roughly every 100ms (~10Hz). If any
// anchor's `delayedFail` counter climbs after flashing, widen
// SLOT_DURATION_MS / TDMA_FRAME_MS (and RESP_DLY below) — these are
// conservative starting points, not bench-verified on this hardware.
#define TDMA_FRAME_MS           100
#define SLOT_DURATION_MS         25
#define BEACON_START_DELAY_MS  5000
#define BEACON_TX_TIMEOUT_MS     20

// ── Bounded wait for TX-complete (SYS_STATUS_TXFRS) on the RESPONSE ──
// NOT unguarded — dwt_starttx(DWT_START_TX_DELAYED) can silently
// never set TXFRS (bad delayed time already passed, radio state
// glitch, etc). An unbounded `while(!TXFRS);` then hangs loop()
// forever, starving mqtt.loop()/keepalive — exactly the
// "disconnected: exceeded timeout" symptom seen on both tags (same
// bug, same fix, as tag.ino's TX_COMPLETE_TIMEOUT_MS).
#define TX_COMPLETE_TIMEOUT_MS   10

// ── Per-anchor reply delay ─────────────────────────────────────
// A0 = 1000us, A1 = 2500us, A2 = 6500us. AIDX = lower nibble of
// ANCHOR_ID (0, 1, 2). MUST match ANCHOR_DELAYS in tag.ino exactly —
// matching Da (tag's reply delay) to Db (this anchor's reply delay)
// is what gives the SDS-TWR accuracy benefit.
//
// A2 is NOT on the uniform +1500us step anymore (that gave 4000, which
// was the real "A2 never heard" bug — see tag.ino's ANCHOR_DELAYS
// comment for the full mechanism: the tag is half-duplex and committed
// to its delayed FINAL-to-A1 TX from ~2500us to ~5000us post-POLL, and
// A2's old 4000us RESPONSE landed inside that blackout window every
// single cycle). A2 now replies at 6500us, after that window closes.
#define AIDX      ((ANCHOR_ID) & 0x0F)
#define RESP_DLY  ((AIDX) == 2 ? 6500 : (1000 + (AIDX) * 1500))

// Extra margin (us) added on top of RESP_DLY while waiting for the
// tag's FINAL after this anchor's RESPONSE goes out.
#define FINAL_RX_MARGIN_US  3000

// ── Antenna delay ─────────────────────────────────────────────
// Left at the shared default on all boards — see plan notes: the
// per-link range bias in this system does not decompose cleanly into
// additive per-device terms (network least-squares fit RMSE 18.8cm),
// so it is corrected in software (server.js RANGE_OFFSETS) rather
// than by chasing per-board ANT_DLY register values against noisy
// data. Revisit with a proper fixed-distance calibration rig.
#define TX_ANT_DLY  16385
#define RX_ANT_DLY  16385

// ── 40-bit timestamp mask (for local-vs-local full timestamp diffs) ─
#define TS_MASK  0x000000FFFFFFFFFFULL

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
static uint8_t tx_resp_msg[] = {
  0x41, 0x88, 0,
  0, 0,           /* PAN ID (3-4) */
  0, 0,           /* dst tag ID (5-6) */
  0, 0,           /* src anchor ID (7-8) */
  FUNC_RESPONSE,
  0, 0, 0, 0,     /* poll_rx_ts (10-13) */
  0, 0, 0, 0,     /* resp_tx_ts (14-17) */
  0, 0
};

static uint8_t tx_beacon_msg[] = {
  0x41, 0x88, 0,
  0, 0,           /* PAN ID (3-4) */
  0xFF, 0xFF,     /* broadcast dst (5-6) */
  0, 0,           /* src anchor ID (7-8) */
  FUNC_BEACON,
  0,              /* beacon seq (10) */
  0, 0,           /* frame duration ms (11-12) */
  0, 0,           /* slot duration ms (13-14) */
  0, 0
};

#define RESP_POLL_RX_TS_IDX  10
#define RESP_TX_TS_IDX       14
#define BEACON_SEQ_IDX       10
#define BEACON_FRAME_IDX     11
#define BEACON_SLOT_IDX      13

// FINAL message field layout (received from tag, not transmitted here)
#define FINAL_POLL_TX_TS_IDX  10
#define FINAL_RESP_RX_TS_IDX  14
#define FINAL_TX_TS_IDX       18

#define RX_BUF_LEN           48

static uint8_t  rxBuf[RX_BUF_LEN];
static uint32_t statusReg    = 0;
static uint64_t lastRxTs     = 0;
static uint64_t pollRxTs     = 0;
static uint64_t respTxTs     = 0;
static uint32_t delayedFail  = 0;
static uint32_t finalMissed  = 0;
static uint32_t lastBeaconMs = 0;
static uint8_t  beaconSeq    = 0;
static uint32_t respCount    = 0;
static uint32_t rangeCount   = 0;
static uint32_t pollCount    = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

extern dwt_txconfig_t txconfig_options;

// ── Bounded wait for an RX event (RXFCG / RX_TO / RX_ERR) ────────
// Same rationale as tag.ino's waitForRxEvent(): the DW3000 hardware
// rx-timeout register is trusted to eventually set SYS_STATUS_ALL_RX_TO,
// but if it doesn't (register race / driver quirk), an unguarded
// `while(!(...));` hangs loop() forever — which here would also stall
// mqtt.loop(), eventually dropping this anchor's MQTT connection.
static bool waitForRxEvent(uint32_t swTimeoutMs) {
  uint32_t start = millis();
  while (millis() - start < swTimeoutMs) {
    statusReg = dwt_read32bitreg(SYS_STATUS_ID);
    if (statusReg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))
      return true;
  }
  return false;
}

static inline uint16_t readU16(const uint8_t* d) {
  return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}
static inline void writeU16(uint8_t* d, uint16_t v) {
  d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8);
}

// ── Connectivity ──────────────────────────────────────────────
void wifiConnect() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[A%X] WiFi connecting", ANCHOR_ID);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf(" OK — %s\n", WiFi.localIP().toString().c_str());
}

void mqttConnect() {
  char id[16]; snprintf(id, sizeof(id), "anchor_%X", ANCHOR_ID);
  while (!mqtt.connected()) {
    Serial.printf("[A%X] MQTT connecting...", ANCHOR_ID);
    if (mqtt.connect(id)) Serial.println("OK");
    else { Serial.println("failed, retry 1s"); delay(1000); }
  }
}

// ── Beacon (A0 only) ─────────────────────────────────────────
static void sendBeacon() {
  tx_beacon_msg[BEACON_SEQ_IDX] = beaconSeq++;
  dwt_write32bitreg(SYS_STATUS_ID,
    SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  dwt_writetxdata(sizeof(tx_beacon_msg), tx_beacon_msg, 0);
  dwt_writetxfctrl(sizeof(tx_beacon_msg), 0, 0);
  if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
    uint32_t t = millis();
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
      if (millis() - t > BEACON_TX_TIMEOUT_MS) break;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  }
  lastBeaconMs = millis();
#if DEBUG_VERBOSE
  Serial.printf("[A%X] Beacon #%d\n", ANCHOR_ID, beaconSeq - 1);
#endif
}

// ── Publish a single range over MQTT ──────────────────────────
static void publishRange(uint16_t tagId, float distM, bool valid) {
  if (!mqtt.connected()) mqttConnect();
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"tag\":%d,\"anchor\":%d,\"dist\":%.4f,\"ts\":%lu}",
    tagId, AIDX, valid ? distM : -1.0f, millis());
  mqtt.publish("uwb/range", buf);
  mqtt.loop();
  rangeCount++;
}

// ── Handle the FINAL from the tag, compute range, publish ─────
// Continuous listen, not a single RX attempt. UWB is a shared medium:
// every anchor's RESPONSE and every tag's FINAL is physically heard
// by every nearby radio, not just its intended recipient. The
// original code here did ONE blocking receive and gave up
// (finalMissed++) on whatever arrived first — even if that frame was
// a SIBLING anchor's own RESPONSE broadcast, not this tag's FINAL.
// Confirmed via diagnostics: A1 (RESP_DLY=2500us) opens its FINAL
// listen window right after its own RESPONSE (~2500us post-POLL);
// A2's RESPONSE fires at ~4000us, landing inside that window BEFORE
// the tag's real FINAL-to-A1 arrives (~5000us, i.e. 2500us resp-delay
// + 2500us matched final-delay) — so A1 always consumed A2's
// broadcast first and never got a second chance. Fix: keep re-arming
// RX, silently discard anything that isn't a validly-addressed FINAL
// for OUR anchor ID and THIS tag, until a match arrives or the
// overall deadline passes.
static void handleFinalAndRange(uint16_t tagId, uint64_t pollRxTsLocal, uint64_t respTxTsLocal) {
  // A2's own matched-delay FINAL now fires at ~2*6500=13000us (~13ms)
  // post-POLL — bumped from 15 to 20ms so that isn't cutting it close
  // (matches the same bump in tag.ino's performRanging()).
  uint32_t deadlineMs = millis() + 20;
  int      attempt    = 0;

  while (millis() < deadlineMs) {
    attempt++;
    uint32_t timeoutUs = RESP_DLY + FINAL_RX_MARGIN_US;
    // Force transceiver to idle before every re-arm. Same root cause as
    // tag.ino's "listen attempt #2 always hardware-stuck" finding: a
    // dwt_rxenable() issued shortly after a recent TX (this anchor's own
    // delayed RESPONSE) or a recent RX-status-register write (a prior
    // overheard/discarded attempt in this very loop) can leave the DW3000
    // in a state where it never sets RXFCG/RX_TO/RX_ERR again. This loop
    // is exactly that pattern — rxenable, discard, rxenable again — so it
    // gets the same fix.
    dwt_forcetrxoff();
    dwt_setrxtimeout(timeoutUs);
    // Same check as tag.ino: rxenable's return code was previously
    // ignored. Non-zero here means the DW3000 rejected the enable call
    // outright (still busy from the RESPONSE TX that just went out) —
    // which alone would explain "hardware never sets any status bit."
    int rxEnableRc = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (rxEnableRc != 0) {
      Serial.printf("[A%X] DIAG FINAL wait attempt #%d: dwt_rxenable() REJECTED, rc=%d\n", ANCHOR_ID, attempt, rxEnableRc);
    }

    if (!waitForRxEvent(15)) {
      Serial.printf("[A%X] DIAG FINAL wait attempt #%d: SOFTWARE TIMEOUT T%d rxEnableRc=%d\n", ANCHOR_ID, attempt, tagId, rxEnableRc);
      dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK);
      finalMissed++;
      return;
    }
    uint32_t st = statusReg;

    if (!(st & SYS_STATUS_RXFCG_BIT_MASK)) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      continue;  // nothing arrived this attempt — keep listening
    }

    uint64_t finalRxTs = get_rx_timestamp_u64();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t flen    = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    uint16_t dataLen = (uint16_t)(flen - FCS_LEN);
    if (dataLen == 0 || dataLen > RX_BUF_LEN) continue;

    dwt_readrxdata(rxBuf, dataLen, 0);

    uint16_t gotPan = readU16(&rxBuf[3]);
    uint8_t  gotFn  = rxBuf[9];
    uint16_t gotDst = readU16(&rxBuf[5]);
    uint16_t gotTag = readU16(&rxBuf[7]);
    bool isOurFinal = (gotPan == NET_PAN_ID) && (gotFn == FUNC_FINAL) &&
                       (gotDst == (uint16_t)ANCHOR_ID) && (gotTag == tagId);

#if DEBUG_VERBOSE
    if (!isOurFinal) {
      Serial.printf("[A%X] DIAG overheard (not our FINAL) attempt #%d: pan=0x%04X fn=0x%02X dst=0x%04X tag=0x%04X\n",
        ANCHOR_ID, attempt, gotPan, gotFn, gotDst, gotTag);
    }
#endif

    if (!isOurFinal) continue;  // sibling anchor's RESPONSE, stale FINAL, etc — keep listening

    uint32_t pollTxWire, respRxWire, finalTxWire;
    resp_msg_get_ts(&rxBuf[FINAL_POLL_TX_TS_IDX], &pollTxWire);
    resp_msg_get_ts(&rxBuf[FINAL_RESP_RX_TS_IDX], &respRxWire);
    resp_msg_get_ts(&rxBuf[FINAL_TX_TS_IDX],      &finalTxWire);

    // Ra, Da come from the tag's own (wire, 32-bit) timestamps.
    // Rb, Db come from this anchor's own local (full-resolution) timestamps.
    int64_t Ra = (int64_t)((uint32_t)(respRxWire - pollTxWire));
    int64_t Da = (int64_t)((uint32_t)(finalTxWire - respRxWire));
    int64_t Rb = (int64_t)((finalRxTs      - respTxTsLocal) & TS_MASK);
    int64_t Db = (int64_t)((respTxTsLocal  - pollRxTsLocal) & TS_MASK);

    double tof = (((double)Ra * (double)Rb) - ((double)Da * (double)Db))
                 / (double)(Ra + Rb + Da + Db) * DWT_TIME_UNITS;
    double distM = tof * SPEED_OF_LIGHT;
    if (distM < 0) distM = 0;

    publishRange(tagId, distM, true);
#if DEBUG_VERBOSE
    Serial.printf("[A%X] RANGE T%d: %.3fm (n=%lu)\n", ANCHOR_ID, tagId, distM, rangeCount);
#endif
    return;
  }

  finalMissed++;
#if DEBUG_VERBOSE
  Serial.printf("[A%X] DIAG FINAL never matched within deadline T%d n=%lu\n", ANCHOR_ID, tagId, finalMissed);
#endif
}

// ── Handle POLL ───────────────────────────────────────────────
static void handlePoll() {
  pollRxTs = lastRxTs;
  uint16_t tagId = readU16(&rxBuf[7]);

  uint32_t respTxTime =
    (pollRxTs + ((uint64_t)RESP_DLY * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(respTxTime);

  respTxTs = (((uint64_t)(respTxTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  tx_resp_msg[2] = rxBuf[2];
  tx_resp_msg[3] = rxBuf[3];
  tx_resp_msg[4] = rxBuf[4];
  tx_resp_msg[5] = rxBuf[7];
  tx_resp_msg[6] = rxBuf[8];

  resp_msg_set_ts(&tx_resp_msg[RESP_POLL_RX_TS_IDX], pollRxTs);
  resp_msg_set_ts(&tx_resp_msg[RESP_TX_TS_IDX],      respTxTs);

  dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
  dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

  int ret = dwt_starttx(DWT_START_TX_DELAYED);
  if (ret == DWT_SUCCESS) {
    uint32_t txWaitStart = millis();
    bool txOk = false;
    while (millis() - txWaitStart < TX_COMPLETE_TIMEOUT_MS) {
      if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) { txOk = true; break; }
    }
    if (txOk) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      respCount++;
#if DEBUG_VERBOSE
      Serial.printf("[A%X] RESP #%lu T%d\n", ANCHOR_ID, respCount, tagId);
#endif
      // Immediately listen for this tag's FINAL and, on success, range+publish.
      handleFinalAndRange(tagId, pollRxTs, respTxTs);
    } else {
      delayedFail++;
      dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
      Serial.printf("[A%X] RESP TX timed out waiting for TXFRS #%lu T%d\n", ANCHOR_ID, delayedFail, tagId);
    }
  } else {
    delayedFail++;
    dwt_write32bitreg(SYS_STATUS_ID,
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
    Serial.printf("[A%X] Delayed TX FAIL #%lu\n", ANCHOR_ID, delayedFail);
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== ANCHOR 0x%02X (DS/SDS-TWR) ===\n", ANCHOR_ID);
  Serial.printf("RESP_DLY=%dus  ANT_DELAY=%d  FRAME=%dms SLOT=%dms\n",
    RESP_DLY, TX_ANT_DLY, TDMA_FRAME_MS, SLOT_DURATION_MS);

  tx_resp_msg[7]   = (uint8_t)(ANCHOR_ID);
  tx_resp_msg[8]   = (uint8_t)(ANCHOR_ID >> 8);
  tx_beacon_msg[3] = (uint8_t)(NET_PAN_ID);
  tx_beacon_msg[4] = (uint8_t)(NET_PAN_ID >> 8);
  tx_beacon_msg[7] = (uint8_t)(ANCHOR_ID);
  tx_beacon_msg[8] = (uint8_t)(ANCHOR_ID >> 8);
  writeU16(&tx_beacon_msg[BEACON_FRAME_IDX], TDMA_FRAME_MS);
  writeU16(&tx_beacon_msg[BEACON_SLOT_IDX],  SLOT_DURATION_MS);

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

  Serial.printf("[A%X] Ready\n", ANCHOR_ID);
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  if (ANCHOR_ID == MASTER_ANCHOR_ID
      && millis() > BEACON_START_DELAY_MS
      && millis() - lastBeaconMs >= TDMA_FRAME_MS) {
    sendBeacon();
  }

  dwt_forcetrxoff();
  dwt_setrxtimeout(20 * 1000UL);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  if (!waitForRxEvent(30)) {
    Serial.printf("[A%X] DIAG main-loop RX wait: SOFTWARE TIMEOUT (hardware never set RXFCG/RX_TO/RX_ERR)\n", ANCHOR_ID);
    dwt_write32bitreg(SYS_STATUS_ID,
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK);
    return;
  }

  if (!(statusReg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return;
  }

  lastRxTs = get_rx_timestamp_u64();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  uint32_t flen    = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  uint16_t dataLen = (uint16_t)(flen - FCS_LEN);
  if (dataLen == 0 || dataLen > RX_BUF_LEN) return;

  dwt_readrxdata(rxBuf, dataLen, 0);
  if (readU16(&rxBuf[3]) != NET_PAN_ID) return;

  uint8_t fn = rxBuf[9];
  if (fn == FUNC_POLL) {
    pollCount++;
    uint16_t tagIdSeen = readU16(&rxBuf[7]);
#if DEBUG_VERBOSE
    Serial.printf("[A%X] DIAG POLL #%lu from T%d\n", ANCHOR_ID, pollCount, tagIdSeen);
#endif
    handlePoll();
  }
  // FUNC_FINAL is consumed inline by handleFinalAndRange() right after
  // this anchor's own RESPONSE, not dispatched here.
}
