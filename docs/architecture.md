# Architecture

## Ranging protocol: SDS-TWR

Each ranging cycle between a tag and an anchor is a 3-message exchange:

```
Tag                                    Anchor
 │──────────── POLL (broadcast) ──────────▶│
 │                                          │  (replies after a fixed delay Db)
 │◀─────────── RESPONSE ───────────────────│
 │  (replies after the SAME delay Da=Db)   │
 │──────────── FINAL ──────────────────────▶│
```

Six timestamps come out of this (tag's POLL-tx and FINAL-tx, anchor's
POLL-rx and RESPONSE-tx, tag's RESPONSE-rx, anchor's FINAL-rx), from which
time-of-flight is:

```
tof = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db) * DWT_TIME_UNITS
```

where `Ra`/`Da` are the tag's own round-trip/reply intervals and `Rb`/`Db`
are the anchor's. Because the tag's reply delay (`Da`) is deliberately set
equal to the anchor's own reply delay (`Db`), the clock-drift error term in
the standard asymmetric DS-TWR formula cancels far more completely than in
a plain single-exchange scheme — this symmetry is what makes it SDS-TWR
rather than plain DS-TWR, and it's the main reason this system gets
centimeter-level std deviation instead of the 10s-of-cm typical of simpler
UWB demos.

The anchor computes the range itself once it has all 6 timestamps (3 of its
own, 3 relayed by the tag's FINAL payload) and publishes it directly over
MQTT — no master anchor aggregates or relays on another anchor's behalf.

## TDMA timing

One anchor (`0xA0`) is the TDMA master: every 100ms it broadcasts a beacon.
Each tag is assigned a fixed 25ms slot within that 100ms frame (derived from
its `TAG_ID`) and only transmits its POLL inside that slot, so the two tags'
ranging cycles never collide on air. Within a tag's own slot, the three
anchors reply at staggered delays (1000us, 2500us, 6500us) so their
RESPONSEs don't collide with each other either — see
[`debugging-notes.md`](debugging-notes.md) for why the third anchor's delay
specifically had to be 6500us rather than the evenly-spaced 4000us it
started at.

This gives each tag a full ranging update roughly every 100ms (~10Hz),
against all three anchors.

## Message addressing

Every UWB frame carries a PAN ID, a 16-bit destination field, and a 16-bit
source field, plus a 1-byte function code (`POLL` / `RESPONSE` / `FINAL` /
`BEACON`). Because UWB is a broadcast medium, every anchor's RESPONSE and
every tag's FINAL is physically received by *every* nearby radio, not just
the intended recipient — both the anchor and tag firmware have to actively
filter incoming frames by PAN ID, function code, and tag/anchor ID rather
than assume "whatever I just received must be the message I was waiting
for." Treating that assumption as safe was the root cause of one of the two
bugs described in `debugging-notes.md`.

## 3D trilateration (server-side)

Given three calibrated slant ranges `(r0, r1, r2)` from a tag to the three
anchors (each anchor's own measured `x, y, z`), the server solves for the
tag's full 3D position directly — no assumed tag height, no 2D-then-correct
step.

Subtracting the anchor-1 and anchor-2 sphere equations from anchor-0's
cancels the squared terms, leaving two linear equations in `(x, y, z)`.
Solving those gives `x` and `y` as linear functions of `z`; substituting
back into anchor-0's sphere equation leaves a single quadratic in `z` alone
— solved in closed form, no iterative solver.

That quadratic has two roots: the true position and its mirror image
reflected through the (near-flat) anchor plane. Since all anchors are
mounted above head height, the real root is whichever one sits below the
anchor plane; if that's ambiguous, the root closest to the tag's own last
known height is used, falling back to a nominal height only on a tag's very
first fix.

## Filtering and fusion

Each tag's raw trilaterated position is smoothed through its own 1D-per-axis
Kalman filter, independently tuned:

| Tag | Role | Process noise (q) | Measurement noise (r) | Why |
|---|---|---|---|---|
| 0 | Helmet (moving) | 0.05 | 0.5 | tracks real movement quickly, tolerates more jitter |
| 1 | Equipment (stationary) | 0.001 | 1.0 | heavily smoothed — it never actually moves, so noise rejection matters more than responsiveness |

The helmet's smoothed position and the equipment's smoothed position are
then combined into a horizontal-only bearing + distance, which the server
maps to a discrete guidance command (`STRAIGHT`, `TURN_LEFT`,
`SLIGHT_RIGHT`, `ARRIVED`, …) and publishes back out over MQTT
(`helmet/guidance`) in addition to broadcasting it to the dashboard over
WebSocket.

## Data flow summary

1. Tag sends POLL in its TDMA slot → all 3 anchors hear it.
2. Each anchor replies with RESPONSE at its own staggered delay.
3. Tag replies to each anchor with a matched-delay FINAL.
4. Each anchor computes its own range from the 6 timestamps and publishes
   `{tag, anchor, dist}` on MQTT topic `uwb/range`.
5. `server.js` ingests each range independently, calibrates it, and — once
   a tag has 3 fresh ranges (all anchors, all under `MAX_AGE_MS`) —
   trilaterates and Kalman-filters a new 3D position.
6. Position (raw + filtered), per-link ranges, and (for the helmet) guidance
   are broadcast over WebSocket to any connected dashboard, and appended to
   per-session CSV logs.
