/*
 * SmartHat — Complete Backend Server (debug version)
 */
'use strict'

const mqtt    = require('mqtt')
const express = require('express')
const { WebSocketServer } = require('ws')
const http    = require('http')
const fs      = require('fs')
const path    = require('path')

// ── Anchor positions — UPDATE THESE to match your physical setup ──
// z is each anchor's real, measured height off the ground (metres).
// There is no separate "tag height" assumption anymore — see the 3D
// trilaterate() below, which solves x, y AND z together from the raw
// slant ranges instead of assuming a fixed tag height and correcting
// to a 2D horizontal distance.
const ANCHORS = [
  { id: 0, x: 0.000, y: 0.000, z: 2.23 },
  { id: 1, x: 2.390, y: 0.000, z: 2.21 },
  { id: 2, x: 1.253, y: 2.504, z: 2.21 }
]

// Only used to disambiguate the 3D solve on a tag's very first fix
// (see trilaterate() below) — after that, each tag's own last-known
// z takes over. Not a hard assumption about tag height.
const NOMINAL_TAG_HEIGHT_M = 1.2

// Software calibration offsets — measured mean minus true distance,
// applied directly to the raw SLANT range (straight-line anchor-to-tag
// distance) now that height is solved rather than assumed. To
// re-derive: measure the real straight-line distance from an anchor to
// a tag with a tape measure (not the horizontal/floor-projected
// distance) and use it as ?true_dist= in GET /api/calibrate.
// Positive = system reads HIGH (subtract to correct)
// Negative = system reads LOW  (add back to correct)
//
// Kept software-side rather than reflashing per-board TX_ANT_DLY/
// RX_ANT_DLY registers: a least-squares fit of these six links against
// per-device (tag+anchor) additive terms did not converge cleanly
// (RMSE 18.8cm, dominated by anchor2 — looks like NLOS/multipath, not
// a fixed antenna delay), so it would miscalibrate hardware registers
// against noise that isn't actually antenna delay. Revisit with a
// proper fixed-distance calibration rig if you want true per-board
// ANT_DLY calibration later — see GET /api/calibrate below.
//
// NOTE: these six values were fit against the OLD SS-TWR protocol AND
// the old anchor geometry, and against horizontal (not slant) true
// distances. Both the protocol and the geometry have since changed —
// re-derive these before trusting them.
const RANGE_OFFSETS = {
  't0_a0': 0, 't0_a1': 0, 't0_a2': 0,
  't1_a0': 0, 't1_a1': 0, 't1_a2': 0
}
// Zeroed out: the previous six values were fit to the old SS-TWR
// protocol, the old anchor geometry, and horizontal (not slant) true
// distances — none of which apply anymore, so carrying them forward
// would silently bias every measurement by a now-meaningless amount.
// Re-derive via GET /api/calibrate?tag=&anchor=&true_dist=, using the
// real tape-measured straight-line (slant) distance from anchor to
// tag, once the system is running with the new geometry.

const MQTT_URL    = 'mqtt://localhost'
const HTTP_PORT   = 3000

// Each server start ("session"/"run") gets its own timestamped
// subfolder under logs/, e.g. logs/2026-09-02_14-05-33/, so separate
// runs never mix into the same CSV files. Everything downstream
// (getCsv(), the position CSV, the /logs static mount, and the
// startup banner) just uses LOG_DIR, so nothing else needs to know
// about the "session" concept — the dashboard's CSV links still
// resolve as /logs/<name>.csv because /logs is mounted directly at
// this session's folder, not at the parent logs/ directory.
function sessionFolderName() {
  const d = new Date()
  const pad = n => String(n).padStart(2, '0')
  return `${d.getFullYear()}-${pad(d.getMonth()+1)}-${pad(d.getDate())}_` +
         `${pad(d.getHours())}-${pad(d.getMinutes())}-${pad(d.getSeconds())}`
}
const LOG_DIR     = path.join(__dirname, 'logs', sessionFolderName())
const NUM_TAGS    = 2
const NUM_ANCHORS = 3

// Tightened from 5000ms now that the firmware ranges at ~10Hz instead
// of ~0.7Hz (TDMA_FRAME_MS=100ms in the new anchor.ino/tag.ino) — this
// is roughly 3 frame periods, tolerating a couple of missed cycles per
// anchor without holding onto multi-second-old data.
const MAX_AGE_MS  = 300

const ARRIVED_DIST_M = 0.8

// Per-tag Kalman tuning. Tag 0 (helmet) is worn by a moving worker, so
// it needs a much less-smoothed filter to track real movement instead
// of lagging behind it — that lag was very likely part of the "shows
// wrong movement" symptom. Tag 1 (equipment) never actually moves, so
// it keeps the original heavy smoothing to reject noise. Applied to
// x, y AND z alike.
const TAG_KF_CONFIG = [
  { q: 0.05,  r: 0.5 },   // Tag 0 — helmet, moving: track fast
  { q: 0.001, r: 1.0 }    // Tag 1 — equipment, stationary: heavy smoothing
]

if (!fs.existsSync(LOG_DIR)) fs.mkdirSync(LOG_DIR, { recursive: true })

const tags = Array.from({ length: NUM_TAGS }, (_, i) => ({
  id: i,
  ranges:   Array(NUM_ANCHORS).fill(null).map(() => ({ dist: null, ts: 0 })),
  rawPos:   null,
  kfPos:    null,
  kf:       { x: null, y: null, z: null, px: 1, py: 1, pz: 1, init: false },
  kfCfg:    TAG_KF_CONFIG[i] || { q: 0.001, r: 1.0 },
  cycles:   0,
  fails:    0,
  lastSeen: null
}))

const csvStreams = {}

function getCsv(tagId, anchorId) {
  const key = `t${tagId}_a${anchorId}`
  if (csvStreams[key]) return csvStreams[key]
  const p = path.join(LOG_DIR, `tag${tagId}_anchor${anchorId}.csv`)
  const isNew = !fs.existsSync(p)
  const s = fs.createWriteStream(p, { flags: 'a' })
  if (isNew) s.write('timestamp,raw_m,session_ms\n')
  csvStreams[key] = s
  return s
}

function kalmanUpdate(kf, mx, my, mz, q, r) {
  if (!kf.init) {
    kf.x = mx; kf.y = my; kf.z = mz; kf.init = true
    return { x: mx, y: my, z: mz }
  }
  kf.px += q
  const kx = kf.px / (kf.px + r)
  kf.x  += kx * (mx - kf.x)
  kf.px *= (1 - kx)

  kf.py += q
  const ky = kf.py / (kf.py + r)
  kf.y  += ky * (my - kf.y)
  kf.py *= (1 - ky)

  kf.pz += q
  const kz = kf.pz / (kf.pz + r)
  kf.z  += kz * (mz - kf.z)
  kf.pz *= (1 - kz)

  return { x: kf.x, y: kf.y, z: kf.z }
}

// ── Real 3D trilateration ───────────────────────────────────────────
// Solves x, y AND z directly from the three raw (calibrated) slant
// ranges — no assumed tag height, no separate 2D horizontal-distance
// correction step. Replaces the old ANCHOR_HEIGHTS + 2D trilaterate().
//
// Method: subtracting the anchor-1 and anchor-2 sphere equations from
// the anchor-0 one cancels the squared terms and leaves two linear
// equations in (x, y, z). Two linear equations in three unknowns give
// x and y as linear functions of z; substituting those back into the
// anchor-0 sphere equation leaves a single quadratic in z alone, which
// has a closed-form solution — no iterative solver needed.
//
// That quadratic generally has two roots: the true position, and its
// mirror image reflected through the (nearly flat) anchor plane. Since
// these anchors are all mounted above head height, the correct root is
// whichever one sits below the anchors; if both (or neither) qualify,
// the root closest to the tag's own last known height is used instead
// — falling back to NOMINAL_TAG_HEIGHT_M only on a tag's very first fix.
//
// Verified numerically against synthetic ground-truth points across
// this anchor layout (center, near each anchor, on the ground, near
// anchor height) before being wired in — all recovered to machine
// precision.
function trilaterate(r0, r1, r2, prevZ) {
  const [A0, A1, A2] = ANCHORS

  const A1c = 2*(A0.x-A1.x), B1c = 2*(A0.y-A1.y), C1c = 2*(A0.z-A1.z)
  const D1  = r1**2 - r0**2 - (A1.x**2-A0.x**2) - (A1.y**2-A0.y**2) - (A1.z**2-A0.z**2)

  const A2c = 2*(A0.x-A2.x), B2c = 2*(A0.y-A2.y), C2c = 2*(A0.z-A2.z)
  const D2  = r2**2 - r0**2 - (A2.x**2-A0.x**2) - (A2.y**2-A0.y**2) - (A2.z**2-A0.z**2)

  const det = A1c*B2c - A2c*B1c
  if (Math.abs(det) < 1e-9) return null

  // x(z) = px + mx*z ,  y(z) = py + my*z
  const px = (D1*B2c - D2*B1c) / det
  const mx = (C2c*B1c - C1c*B2c) / det
  const py = (A1c*D2 - A2c*D1) / det
  const my = (A2c*C1c - A1c*C2c) / det

  const dx0 = px - A0.x, dy0 = py - A0.y
  const a = mx*mx + my*my + 1
  const b = 2*(mx*dx0 + my*dy0 - A0.z)
  const c = dx0*dx0 + dy0*dy0 + A0.z*A0.z - r0**2

  const disc = b*b - 4*a*c
  if (disc < 0) return null   // no real solution — bad/inconsistent ranges
  const sq = Math.sqrt(disc)
  const z1 = (-b + sq) / (2*a)
  const z2 = (-b - sq) / (2*a)

  const minAnchorZ = Math.min(A0.z, A1.z, A2.z)
  const candidates = [z1, z2]
  const belowAnchors = candidates.filter(z => z < minAnchorZ + 0.05)

  let z
  if (belowAnchors.length === 1) {
    z = belowAnchors[0]
  } else {
    const pool   = belowAnchors.length ? belowAnchors : candidates
    const target = (prevZ !== null && prevZ !== undefined) ? prevZ : NOMINAL_TAG_HEIGHT_M
    z = pool.reduce((best, zc) =>
      Math.abs(zc - target) < Math.abs(best - target) ? zc : best, pool[0])
  }

  const x = px + mx*z
  const y = py + my*z

  const margin = 8
  if (x < -margin || y < -margin) return null
  if (z < -1 || z > minAnchorZ + 1) return null   // sanity bound, not a hard assumption

  return { x: +x.toFixed(4), y: +y.toFixed(4), z: +z.toFixed(4) }
}

function computeGuidance(helmetPos, equipPos) {
  // Deliberately horizontal-only: walking guidance shouldn't factor in
  // a height difference the worker can't do anything about by walking.
  if (!helmetPos || !equipPos) return null
  const dx = equipPos.x - helmetPos.x
  const dy = equipPos.y - helmetPos.y
  const dist = Math.sqrt(dx*dx + dy*dy)
  const bearing = Math.atan2(dy, dx) * 180 / Math.PI
  return { bearing: +bearing.toFixed(2), dist: +dist.toFixed(3) }
}

function getGuidanceCmd(bearing, dist) {
  if (dist < ARRIVED_DIST_M) return 'ARRIVED'
  if (bearing >  30) return 'TURN_RIGHT'
  if (bearing >  10) return 'SLIGHT_RIGHT'
  if (bearing < -30) return 'TURN_LEFT'
  if (bearing < -10) return 'SLIGHT_LEFT'
  return 'STRAIGHT'
}

// ── Apply software calibration to one raw slant range ───────────────
// No height correction here anymore — trilaterate() works directly
// with slant ranges and solves height itself.
function calibrateSlantRange(tagId, anchorId, rawSlantM) {
  const offsetKey = `t${tagId}_a${anchorId}`
  const offset    = RANGE_OFFSETS[offsetKey] || 0
  return rawSlantM - offset
}

// ── After tag.ranges[] is updated, try to trilaterate + publish ────
function tryComputePosition(tagId) {
  const tag = tags[tagId]
  const now = Date.now()
  const fresh = tag.ranges.filter(r => r.dist !== null && (now - r.ts) < MAX_AGE_MS)
  console.log('[PROCESS] T%d fresh ranges:'.replace('%d', tagId), fresh.length, '/', NUM_ANCHORS)

  if (fresh.length < 3) return

  const [r0, r1, r2] = tag.ranges.map(r => r.dist)
  const prevZ = tag.kf.init ? tag.kf.z : null
  const raw = trilaterate(r0, r1, r2, prevZ)
  if (!raw) {
    console.log('[PROCESS] Trilateration failed — degenerate geometry or no real solution')
    return
  }

  tag.rawPos = raw
  const kf   = kalmanUpdate(tag.kf, raw.x, raw.y, raw.z, tag.kfCfg.q, tag.kfCfg.r)
  tag.kfPos  = kf

  const posCsv = path.join(LOG_DIR, `tag${tagId}_position.csv`)
  if (!fs.existsSync(posCsv))
    fs.writeFileSync(posCsv, 'timestamp,raw_x,raw_y,raw_z,kf_x,kf_y,kf_z,r0,r1,r2\n')
  fs.appendFileSync(posCsv,
    `${new Date(now).toISOString()},${raw.x},${raw.y},${raw.z},${kf.x.toFixed(4)},${kf.y.toFixed(4)},${kf.z.toFixed(4)},${r0.toFixed(4)},${r1.toFixed(4)},${r2.toFixed(4)}\n`)

  broadcast({
    type: 'position', tag: tagId,
    raw:  raw,
    kf:   { x: +kf.x.toFixed(4), y: +kf.y.toFixed(4), z: +kf.z.toFixed(4) },
    r:    [r0, r1, r2],
    cycles: tag.cycles, fails: tag.fails, ts: now
  })

  if (tagId === 0 && tags[1].kfPos) {
    const guidance = computeGuidance(kf, tags[1].kfPos)
    if (guidance) {
      broadcast({ type: 'guidance', ...guidance, ts: now })
      mqttClient.publish('helmet/guidance', JSON.stringify({
        bearing: guidance.bearing, dist: guidance.dist,
        cmd: getGuidanceCmd(guidance.bearing, guidance.dist)
      }))
    }
  }
}

// ── New primary ingest path: one anchor's range for one tag ────────
// Every anchor now computes and publishes its own range directly
// (see anchor.ino handleFinalAndRange) instead of the tag aggregating
// all three into one REPORT relayed only through the master anchor —
// so each link updates the server independently and a single dead
// anchor no longer blocks the other two.
function processSingleRange(tagId, anchorId, rawDist) {
  if (tagId === undefined || tagId >= NUM_TAGS) return
  if (anchorId === undefined || anchorId >= NUM_ANCHORS) return

  const tag = tags[tagId]
  tag.cycles++
  tag.lastSeen = Date.now()
  const now = Date.now()
  const sessionMs = now - (tag.sessionStart || (tag.sessionStart = now))

  const valid = typeof rawDist === 'number' && rawDist > 0.05 && rawDist < 100

  if (valid) {
    const calibrated = calibrateSlantRange(tagId, anchorId, rawDist)
    tag.ranges[anchorId] = { dist: calibrated, ts: now }
    const csv = getCsv(tagId, anchorId)
    csv.write(`${new Date(now).toISOString()},${rawDist.toFixed(4)},${calibrated.toFixed(4)},${sessionMs}\n`)
    broadcast({ type: 'range', tag: tagId, anchor: anchorId, dist: calibrated, ts: now })
  } else {
    tag.fails++
    broadcast({ type: 'fail', tag: tagId, anchor: anchorId, ts: now })
  }

  tryComputePosition(tagId)
}

// ── Legacy batch ingest (old REPORT-based firmware) — kept for      ──
// backward compatibility during a firmware rollout; the new firmware ──
// no longer sends this. ────────────────────────────────────────────
function processRanges(data) {
  console.log('[PROCESS] (legacy batch) tag:', data.tag, 'r0:', data.r0, 'r1:', data.r1, 'r2:', data.r2)

  const tagId = data.tag
  if (tagId === undefined || tagId >= NUM_TAGS) return

  const tag = tags[tagId]
  tag.cycles++
  tag.lastSeen = Date.now()
  const now = Date.now()
  const sessionMs = now - (tag.sessionStart || (tag.sessionStart = now))

  for (let a = 0; a < NUM_ANCHORS; a++) {
    const raw   = data[`r${a}`]
    const valid = data[`v${a}`] && raw > 0.05 && raw < 100
    if (valid) {
      const calibrated = calibrateSlantRange(tagId, a, raw)
      tag.ranges[a] = { dist: calibrated, ts: now }
      const csv = getCsv(tagId, a)
      csv.write(`${new Date(now).toISOString()},${raw.toFixed(4)},${calibrated.toFixed(4)},${sessionMs}\n`)
      broadcast({ type: 'range', tag: tagId, anchor: a, dist: calibrated, ts: now })
    } else {
      tag.fails++
      broadcast({ type: 'fail', tag: tagId, anchor: a, ts: now })
    }
  }

  tryComputePosition(tagId)
}

// ── MQTT ──────────────────────────────────────────────────────
const mqttClient = mqtt.connect(MQTT_URL)

mqttClient.on('connect', () => {
  console.log('[MQTT] Connected')
  mqttClient.subscribe('uwb/ranges')  // legacy batch (REPORT-based firmware)
  mqttClient.subscribe('uwb/range')   // primary: one anchor's link at a time
  console.log('[MQTT] Subscribed to uwb/ranges, uwb/range')
})

mqttClient.on('message', (topic, payload) => {
  let d
  try { d = JSON.parse(payload.toString()) }
  catch(e) { console.log('[MQTT] JSON parse error:', payload.toString()); return }

  if (topic === 'uwb/ranges') processRanges(d)

  if (topic === 'uwb/range') {
    processSingleRange(d.tag, d.anchor, d.dist)
  }
})

mqttClient.on('error', e => console.error('[MQTT] Error:', e.message))

// ── HTTP + WebSocket ──────────────────────────────────────────
const app    = express()
const server = http.createServer(app)
const wss    = new WebSocketServer({ server })

app.use(express.static(path.join(__dirname, '..', 'public')))
app.use('/logs', express.static(LOG_DIR))

app.get('/api/state', (req, res) => {
  res.json({
    anchors: ANCHORS,
    tags: tags.map(t => ({
      id: t.id, rawPos: t.rawPos, kfPos: t.kfPos,
      ranges: t.ranges.map(r => r.dist),
      cycles: t.cycles, fails: t.fails, lastSeen: t.lastSeen
    }))
  })
})

app.get('/api/calibrate', (req, res) => {
  const tagId    = parseInt(req.query.tag    || 0)
  const anchorId = parseInt(req.query.anchor || 0)
  // true_dist is now the real STRAIGHT-LINE (slant) distance from the
  // anchor to the tag — tape-measure it directly, not the horizontal
  // floor projection — since ranges are no longer height-corrected
  // before calibration.
  const trueDist = parseFloat(req.query.true_dist)
  if (isNaN(trueDist) || trueDist <= 0)
    return res.status(400).json({ error: 'Provide ?true_dist=1.0 in metres (straight-line anchor-to-tag distance)' })
  const tag   = tags[tagId]
  const range = tag.ranges[anchorId]
  if (!range || !range.dist)
    return res.status(400).json({ error: 'No data yet for this pair' })
  const measured = range.dist
  const errorM   = measured - trueDist
  const corr     = Math.round(errorM * 63897600000 / 299702547 / 2)
  // Baseline TX_ANT_DLY/RX_ANT_DLY every board currently ships with
  // (the shared default — see the RANGE_OFFSETS comment above for why
  // per-board register calibration isn't used instead). Echoed back so
  // the dashboard can show "current -> new" without hardcoding it twice.
  const CURRENT_ANT_DLY = 16385
  res.json({
    tag: tagId, anchor: anchorId,
    true_dist_m: trueDist, measured_m: +measured.toFixed(4),
    error_m: +errorM.toFixed(4), correction_ticks: corr,
    current_delay: CURRENT_ANT_DLY,
    new_ant_delay: CURRENT_ANT_DLY - corr,
    instruction: `Set TX_ANT_DLY and RX_ANT_DLY to ${CURRENT_ANT_DLY - corr}`
  })
})

wss.on('connection', ws => {
  console.log('[WS] Client connected')
  ws.send(JSON.stringify({ type: 'anchors', anchors: ANCHORS }))
  tags.forEach(t => {
    if (t.kfPos) {
      ws.send(JSON.stringify({
        type: 'position', tag: t.id,
        raw: t.rawPos, kf: t.kfPos,
        r: t.ranges.map(r => r.dist),
        cycles: t.cycles, fails: t.fails
      }))
    }
  })
})

function broadcast(obj) {
  const msg = JSON.stringify(obj)
  wss.clients.forEach(c => { if (c.readyState === 1) c.send(msg) })
}

server.listen(HTTP_PORT, () => {
  console.log(`\n${'═'.repeat(52)}`)
  console.log(` SmartHat UWB System — Server running`)
  console.log(` Data dashboard:   http://localhost:${HTTP_PORT}`)
  console.log(` Live simulation:  http://localhost:${HTTP_PORT}/sim.html`)
  console.log(` Logs:             ${LOG_DIR}`)
  console.log(`${'═'.repeat(52)}\n`)
})
