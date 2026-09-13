# XorFold RoCC Accelerator

A custom RISC-V RoCC accelerator built from scratch on [Chipyard](https://github.com/ucb-bar/chipyard),
covering the full path from register-only compute, to autonomous streaming memory
access over two independent memory interfaces, to a real network-checksum algorithm.
Built as a self-directed learning project to go hands-on with Chisel and Chipyard's
RoCC accelerator interface, modeled structurally on rocket-chip's built-in
`AccumulatorExample` and `CharacterCountExample`.

Every operation described below is verified against real Verilator simulations of
both a Rocket core and a BOOM out-of-order core with this accelerator attached, not
just elaborated — see [Verification](#verification).

## What it does

XorFold is a toy "checksum-style" DPU-offload primitive, loosely analogous in shape
(not in real algorithm, for most of its operations) to a NIC/DPU checksum offload
engine. It exposes custom RISC-V instructions on opcode `custom3`, decoded by a
`funct` field:

| funct | Operation | Description |
|---|---|---|
| 0 | `reset()` | `accum := 0` |
| 1 | `fold(a, b)` | `accum := accum ^ (a + b)` — register-only |
| 2 | `read()` | Returns `accum` |
| 3 | `fold_mem(ptr)` | Loads one word from memory via `HellaCacheIO`, XORs it into `accum` |
| 4 | `fold_mem_n(ptr, n)` | Autonomously streams `n` words from memory, XOR-folding each one, with no further core instructions per word |
| 5 | `write_result(dst)` | Writes the current `accum` value to memory |
| 6 | `checksum_reset()` | Resets the independent RFC 1071 checksum accumulator |
| 7 | `checksum_add_n(ptr, n)` | Streams `n` memory beats into an RFC 1071 (Internet checksum) one's-complement running sum |
| 8 | `checksum_finalize()` | Folds the checksum accumulator to 16 bits and returns the complemented result |
| 9 | `checksum_update(old, new)` | RFC 1624 incremental checksum update — recomputes the checksum after one 16-bit field changes, without re-scanning the buffer |
| 10 | `write_result_n(dst, n)` | Broadcasts the current `accum` value to `n` consecutive destination words |
| 11 | `fold_mem_tl(ptr)` | Raw-TileLink counterpart of `fold_mem` — bypasses the L1 cache entirely via a dedicated `TLClientNode` |
| 12 | `fold_mem_tl_n(ptr, n)` | Streams `n` words over raw TileLink with **two outstanding requests in flight** at once |

Funct=0/1/3/4/5/10/11/12 operate on the XOR accumulator; funct=6/7/8/9 operate on a
completely separate checksum accumulator, so the two capabilities can't interfere with
each other's state.

## Design

The whole design lives in one file, [`src/main/scala/XorFoldAccelerator.scala`](src/main/scala/XorFoldAccelerator.scala),
structured as three independent memory engines that share one command decoder and are
never active at the same time:

- **HellaCache engine** (funct=3/4/5/7/10) — a small `sIdle → sMemReq → sMemResp` FSM
  used by every operation that goes through the cached, VA-translated `HellaCacheIO`
  path (the same interface a core's own load/store pipeline uses). Reads and writes
  share identical `ptr`/`remaining` bookkeeping, so `fold_mem`/`write_result` are
  simply the `n=1` special case of `fold_mem_n`/`write_result_n`.
- **Single-request TileLink engine** (funct=11) — a three-state FSM (`tlIdle → tlReq →
  tlResp`) that issues one raw TileLink `Get` via a dedicated `TLClientNode`
  (`atlNode`), bypassing the L1 cache and any core-side address translation entirely —
  architecturally closer to how a real NIC/DPU touches system memory than the cached
  HellaCacheIO path. Checks the TileLink edge's `legal` bit before ever asserting a
  request, and the D-channel's `denied`/`corrupt` bits on response — both of which the
  reference `CharacterCountExample` this was modeled on silently ignores.
- **Two-outstanding TileLink streaming engine** (funct=12) — the actual payoff of
  moving to TileLink: up to two `Get`s in flight simultaneously, using two TileLink
  source IDs, tracked with a per-source busy vector and issue/in-flight counters
  rather than a monolithic FSM (TileLink's A and D channels can make independent
  progress in the same cycle, which doesn't map onto mutually-exclusive states). XOR's
  associativity means a response can be folded into `accum` the instant it arrives,
  regardless of which of the two outstanding requests it answers — no response
  reordering or per-slot address tracking needed. On an illegal/denied/corrupt access
  mid-stream, further issue stops but already-outstanding requests are still drained
  before the operation completes, rather than leaving the engine's bookkeeping
  permanently wrong.
- **RFC 1071/1624 checksum** (funct=6/7/8/9) reuses the HellaCache engine's memory
  path; the only difference is what the response handler does with loaded data
  (one's-complement add with end-around carry, folded to a running 16-bit scalar sum
  every beat, instead of XOR) plus a byte-swap step per 16-bit lane to correctly
  compute network-byte-order checksums over RISC-V's little-endian memory layout.
  `checksum_finalize` (funct=8) is a separate explicit operation rather than folded
  into the generic read path, so `read()`'s meaning never depends on which operation
  ran before it. `checksum_update` (funct=9) exploits the identity `C = ~S` (the
  accumulator stores the *uncomplemented* sum) to implement RFC 1624's
  `~(~C + ~old + new)` as a direct update to that same running sum, reusing the
  existing one's-complement adder with no new arithmetic.

## Verification

[`chipyard-integration/xorfold.c`](chipyard-integration/xorfold.c) is a baremetal test
covering every operation above (30 checks), run against Verilator simulations of two
Chipyard configs — `XorFoldRoCCConfig` (single Rocket core) and `XorFoldRoCCBoomConfig`
(single BOOM out-of-order core) — both with this accelerator attached. All 30 checks
pass on both cores, confirming the RoCC command/response handshake and the TileLink
concurrency logic aren't accidentally coupled to Rocket-specific timing.

The two-outstanding TileLink engine's concurrency was additionally confirmed by
inspecting a waveform of the `n=5` test directly, not just its pass/fail result: the
trace shows request #1 issue on source 0, request #2 issue on source 1 while #1 is
still outstanding, both sources genuinely busy simultaneously (issue correctly
stalling until one frees), then each subsequent response immediately freeing and
reusing its source for the next unissued word — with `accum` visibly stepping through
`0x11 → 0x33 → 0x77 → 0xff → 0x1ef` in the exact order and values hand-derived during
design, before `tlStreamActive` drops on the completing cycle.

Notable verification patterns used throughout:
- Hand-computed expected values for every check (e.g. `fold(5,3)` then `fold(1,1)` →
  `0 ^ 8 ^ 2 = 10`).
- An independent software reference implementation of the RFC 1071 checksum,
  cross-checked against both a hand-computed value and the hardware's result.
- Direct A/B comparisons between equivalent operations on different interfaces —
  `fold_mem` (HellaCacheIO) vs. `fold_mem_tl` (TileLink) on the same data, and
  `fold_mem_n` vs. `fold_mem_tl_n` on a 5-word buffer specifically chosen so the
  two-outstanding engine is forced to free and reuse a source ID mid-stream, not just
  issue one pair concurrently.
- Dedicated checks that each new capability doesn't disturb unrelated state (e.g.
  `checksum_reset` doesn't touch the XOR accumulator; `write_result_n`/`fold_mem_tl_n`
  with `n=0` are true no-ops, verified against pre-poisoned sentinel values).
- The test's own pass/fail signal was verified non-trivial early on by deliberately
  breaking an expected value and confirming the simulator correctly reports
  `*** FAILED ***` with a nonzero exit code, rather than trusting a green result that
  might not mean anything.

A few real bugs were caught and fixed with hardware evidence rather than guessing —
worth mentioning because they're the kind of bug that doesn't show up until you
actually run something:
- A missing `io.mem.req.bits.mask` on the write path meant stores completed
  "successfully" from the FSM's point of view but never actually wrote any bytes —
  found via a VCD waveform trace showing the request was correct but memory never
  changed.
- A test that looked like it verified a memory write actually didn't, because the
  destination wasn't `volatile` — GCC constant-folded the "read after write" away at
  compile time, confirmed via disassembly.
- `checksumAccum` originally stored four unfolded 16-bit lanes rather than a true
  running scalar sum, which worked for `checksum_finalize` but was numerically wrong
  as an input to `checksum_update`'s single-lane arithmetic — caught by tracing the
  RFC 1624 formula against the accumulator's actual bit layout before writing any
  Chisel, not by a failing test.

## Repository layout

- [`src/main/scala/XorFoldAccelerator.scala`](src/main/scala/XorFoldAccelerator.scala) —
  the accelerator itself. This is the actual design work.
- [`chipyard-integration/`](chipyard-integration/) — everything needed to wire this
  accelerator into a Chipyard checkout (submodule registration, `build.sbt` project,
  config fragments, CMake test target) plus the verification test, packaged so the
  integration is reproducible from a fresh Chipyard clone without needing a maintained
  fork. See [`chipyard-integration/README.md`](chipyard-integration/README.md) for
  exact steps.

The Chipyard-side wiring (config fragments, `build.sbt` project registration) is
necessary "glue" following Chipyard's own documented submodule pattern for adding a
generator — not a novel contribution. The accelerator design and its verification are
the actual work.

## Out of scope

This project intentionally does not attempt real networking, multi-core, or
multi-board work — it's a single core (Rocket or BOOM) plus one custom accelerator,
fully simulated. No real hardware (FPGA/ASIC) target has been attempted.

## Roadmap

- A raw-TileLink write path (`Put`), mirroring `write_result`/`write_result_n` but
  bypassing the L1 cache the way `fold_mem_tl`/`fold_mem_tl_n` already do for reads.
- Scaling the TileLink streaming engine beyond two outstanding requests.
- Software-visible TileLink error reporting — `fold_mem_tl`/`fold_mem_tl_n` currently
  record illegal/denied/corrupt accesses in a debug-only register inspectable only in
  a waveform; there's no instruction to read it back yet.
