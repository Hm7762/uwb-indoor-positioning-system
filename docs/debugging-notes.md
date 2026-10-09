# Debugging notes: two real bugs, found on real hardware

Both of these only showed up once actual boards were ranging at full speed
— neither was visible from reading the code casually, and both would be
easy for someone building a similar system to reintroduce. Documenting them
here mainly for that reason.

## Bug 1 — stale POLL timestamp corrupted every anchor's range identically

**Symptom:** none of the three anchors were producing fresh, valid ranges
("0/3 fresh" in the server logs) — not an isolated anchor, all three at
once.

**Root cause:** in the tag's `performRanging()`, the code read the tag's
own POLL transmit timestamp (`get_tx_timestamp_u64()`) immediately after
calling `dwt_starttx()`, with no wait for the transmission to actually
complete. The DW3000's TX timestamp register isn't guaranteed valid until
the `TXFRS` (TX frame sent) status bit is set — reading it early can return
a stale value left over from a *previous* transmission, or an unsettled
one.

That one timestamp (`pollTx64`) feeds into the `Ra`/`Da` calculation sent
to *every* anchor in that cycle's FINAL messages. A wrong value there
corrupts all three anchors' computed ranges identically — which is exactly
why the symptom was "all three anchors wrong together," not "one anchor
acting up." That pattern was the actual clue that pointed away from
anchor-side causes and toward something shared upstream, on the tag.

**Fix:** add the same bounded wait-for-`TXFRS` pattern already used
elsewhere in the firmware before reading the POLL's TX timestamp:

```c
uint32_t pollTxWaitStart = millis();
while (millis() - pollTxWaitStart < TX_COMPLETE_TIMEOUT_MS) {
  if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) break;
}
dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
uint64_t pollTx64 = get_tx_timestamp_u64();
```

**Why it's worth calling out:** a bounded wait for TX-complete was already
present on the *other* delayed transmissions in this codebase (the FINAL
replies, the anchors' RESPONSEs) — it had just been missed on this one,
earlier, immediate transmission, because "it's immediate, it must be fast
enough" is an easy assumption to make and a wrong one on hardware where the
timestamp register's validity is tied to a status flag, not to wall-clock
time.

## Bug 2 — anchor 2 "never heard," at a specific reply delay

**Symptom:** anchor A2 reliably logged that it received every POLL and
transmitted every RESPONSE (confirmed via its own serial log) — but the tag
never seemed to act on A2's RESPONSE. From the tag's side, it looked like
"A2 just isn't there."

**Root cause — a deterministic timing collision, not noise:** the system
originally staggered the three anchors' reply delays evenly: 1000us,
2500us, 4000us for A0/A1/A2. The DW3000 is half-duplex. Once the tag
receives A1's RESPONSE at ~2500us post-POLL, it immediately commits to
transmitting its own matched-delay FINAL back to A1 at
`2500us (A1's delay) + 2500us (matched delay) = 5000us` post-POLL — which
means the tag's radio is **transmit-committed and deaf to incoming frames
from ~2500us to ~5000us, every single cycle.**

A2's RESPONSE, at the original 4000us delay, landed squarely inside that
2500–5000us window — *every time*. A0 and A1 don't collide with each other
the same way (A0's busy window, 1000–2000us, closes with 500us to spare
before A1's 2500us arrival), but A2 at 4000us was guaranteed to be
swallowed by the tag's own commitment to replying to A1. It wasn't
intermittent interference — it was 100% reproducible, which is itself a
strong signal to look for a structural/timing cause rather than a
signal-quality one.

**Fix:** push A2's reply delay out past the tag's A1-FINAL blackout window,
to 6500us (1500us of margin past the 5000us window close), instead of the
evenly-spaced 4000us:

```c
// anchor.ino
#define RESP_DLY  ((AIDX) == 2 ? 6500 : (1000 + (AIDX) * 1500))

// tag.ino — must match exactly
const uint32_t ANCHOR_DELAYS[NUM_ANCHORS] = { 1000, 2500, 6500 };
```

**A related fix that had to come with it:** the tag's original listen logic
assumed "listen attempt #1 == A0's response, attempt #2 == A1's, attempt #3
== A2's" — i.e. it trusted *when* a frame arrived to infer *whose* frame it
was. That assumption breaks the moment any anchor is late, missed, or
simply not the next one in a fixed sequence: a still-open listen window
meant for one anchor can swallow a different anchor's transmission instead,
and the tag would misattribute it. The fix was to stop assuming order and
instead identify every incoming frame by its actual payload (source anchor
ID, function code, PAN ID), re-arming the receiver in a loop until all
three anchors are accounted for or an overall deadline passes — regardless
of which attempt number each one arrived on. The same "trust the payload,
not the arrival order" principle applies on the anchor side for FINAL
reception, since every anchor's RESPONSE and every tag's FINAL is audible
to every nearby radio on a shared medium (see `architecture.md`).

## A smaller, related fix worth a mention

Several `while (!statusBit);` spins (waiting on a DW3000 status register)
were originally unbounded. If the expected status bit is ever not set —
due to a register race, a rejected `dwt_rxenable()` call, or a delayed
transmission whose scheduled time has already passed — those loops hang
`loop()` forever, which starves `mqtt.loop()` and silently drops the MQTT
connection ("disconnected: exceeded timeout") with no indication of the
real cause. Every such wait was changed to a `millis()`-bounded version
that gives up and logs a diagnostic instead of hanging.

## Takeaways

- A bug that corrupts *multiple seemingly independent* downstream values
  identically is a strong hint to look for one shared upstream cause rather
  than three separate ones.
- A failure that's 100% reproducible under the same conditions is much more
  likely to be a structural timing/logic issue than RF noise — treat
  "it never works" as more diagnosable than "it sometimes works."
- On real half-duplex radio hardware, "received in order" is not a safe
  assumption once any timing margin gets tight — identify messages by
  content, not by arrival sequence.
- An unbounded wait on a hardware status flag is a hang waiting to happen;
  every blocking wait in this firmware now has a software timeout and a
  diagnostic path.
