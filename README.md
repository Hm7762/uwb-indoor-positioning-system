# SmartHat — UWB Indoor Positioning & Guidance System

A real-time indoor positioning system built on ultra-wideband (UWB) ranging,
designed to track a worker's helmet and guide them to a piece of stationary
equipment in environments where GPS doesn't work — indoors, underground, or
inside dense industrial structures.

Three fixed anchors and two mobile tags (DW3000 UWB radios on ESP32) perform
genuine two-way time-of-flight ranging over the air, stream raw distances to
a Node.js server over MQTT, and get fused into live 3D (x, y, z) positions
with a Kalman filter — visualized on a browser dashboard in real time.

This started as a lab/thesis-adjacent hardware project. The interesting part
isn't just that it works — it's two specific, non-obvious bugs that showed up
only once real hardware was ranging at full speed, and how they were tracked
down. See [`docs/debugging-notes.md`](docs/debugging-notes.md) for that story.

## What it does

- Ranges two tags against three anchors ~10 times per second each, using
  **SDS-TWR** (symmetric double-sided two-way ranging) — the accuracy-grade
  version of UWB ranging, not the simpler single-exchange method most
  hobbyist UWB projects use.
- Solves each tag's full 3D position (not just x/y with an assumed height)
  directly from the three raw ranges, with no fixed-height assumption.
- Smooths positions with a per-tag-tuned Kalman filter — fast-tracking for
  the worker's moving helmet tag, heavily smoothed for the stationary
  equipment tag.
- Computes a live bearing + distance from the helmet to the equipment and
  streams a turn-by-turn guidance command (`TURN_LEFT`, `STRAIGHT`,
  `ARRIVED`, etc.) back out over MQTT.
- Logs every raw range and every computed position to CSV per session, for
  later accuracy analysis.
- Serves two live dashboards: a numeric/data view and a top-down animated
  simulation of anchor and tag positions.

## Measured accuracy

Across repeated static placements, the system held a standard deviation of
**1.04–1.65 cm** per anchor-tag link — tight enough that most of the
remaining real-world error is the geometry/calibration setup rather than
ranging noise.

## Hardware

- 3× Makerfabs ESP32-UWB-DW3000 boards, flashed as **anchors**
  (`firmware/anchor/anchor.ino`), mounted at fixed, surveyed positions.
- 2× the same board, flashed as **tags** (`firmware/tag/tag.ino`): one worn
  on a helmet (moving), one attached to the equipment (stationary).
- A Wi-Fi network and MQTT broker reachable by all 5 boards and the server.

## Architecture

```
 [Anchor A0] ◄──UWB──┐
 [Anchor A1] ◄──UWB──┼──  ranging  ──►  [Tag 0 / helmet]
 [Anchor A2] ◄──UWB──┘                  [Tag 1 / equipment]
       │                                       │
       └──────────────── MQTT ─────────────────┘
                           │
                           ▼
                 server/server.js (Node.js)
            trilateration + Kalman filter + guidance
                           │
                 ┌─────────┴─────────┐
                 ▼                   ▼
        WebSocket broadcast    HTTP API + CSV logs
                 │
      ┌──────────┴──────────┐
      ▼                     ▼
 public/index.html    public/sim.html
 (data dashboard)     (live top-down sim)
```

Each anchor independently computes and publishes its own range for each tag
(rather than one anchor aggregating all three and relaying — see
`docs/debugging-notes.md` for why that mattered). The server fuses whichever
ranges are freshest, so one anchor dropping out doesn't take the whole system
down. See [`docs/architecture.md`](docs/architecture.md) for the full ranging
protocol, timing model, and trilateration math.

## Repo layout

```
firmware/
  anchor/anchor.ino        flash to all 3 anchor boards (change only ANCHOR_ID)
  tag/tag.ino               flash to both tag boards (change only TAG_ID)
  secrets.example.h         copy to secrets.h and fill in your WiFi/MQTT details
server/
  server.js                MQTT ingest, trilateration, Kalman filter, WebSocket + HTTP API
  package.json
  logs/                     per-session CSV logs are written here at runtime
public/
  index.html                data dashboard
  sim.html                  live top-down simulation
docs/
  architecture.md           ranging protocol, timing, and trilateration in depth
  debugging-notes.md        the two real bugs found on real hardware, and the fixes
```

## Setup

1. **Firmware credentials.** Copy `firmware/secrets.example.h` to
   `firmware/secrets.h` and fill in your Wi-Fi SSID/password and the IP of
   the machine that will run `server.js`. `secrets.h` is gitignored — it
   never gets committed.
2. **Flash the anchors.** Open `firmware/anchor/anchor.ino` in the Arduino
   IDE, set `ANCHOR_ID` to `0xA0` (master), `0xA1`, or `0xA2`, and flash
   each of the 3 boards in turn — nothing else changes between boards.
3. **Flash the tags.** Open `firmware/tag/tag.ino`, set `TAG_ID` to `0`
   (helmet) or `1` (equipment), and flash both boards.
4. **Measure your anchor positions** (x, y, z in metres, relative to any
   fixed origin) and update the `ANCHORS` array near the top of
   `server/server.js`.
5. **Run the server:**
   ```
   cd server
   npm install
   npm start
   ```
6. Open `http://<server-host>:3000/` for the data dashboard, or
   `http://<server-host>:3000/sim.html` for the live simulation.

### Calibration (optional)

`RANGE_OFFSETS` in `server.js` starts at zero. To correct a small systematic
bias on a specific anchor-tag link, tape-measure the real straight-line
(slant, not horizontal) distance between them and hit
`GET /api/calibrate?tag=<id>&anchor=<id>&true_dist=<metres>` — it returns the
measured error and the offset to apply.

## Why SDS-TWR instead of simpler ranging

Most UWB ranging demos use single-sided two-way ranging, where only one side
measures a round trip and clock drift between the two radios' independent
crystals becomes a direct range error. SDS-TWR has each side measure a
matched round trip (anchor replies after a fixed delay; tag replies after
that *same* delay back to the anchor), which cancels the clock-drift term to
a much smaller residual. It costs a third message per exchange (POLL →
RESPONSE → FINAL) but is the right tradeoff once sub-5cm accuracy actually
matters, as it does here for guiding someone to a specific spot rather than
just telling them "nearby."

## License

MIT — see [LICENSE](LICENSE).
