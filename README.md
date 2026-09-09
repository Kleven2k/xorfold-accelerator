# XorFold RoCC Accelerator

A custom RISC-V RoCC accelerator built from scratch on [Chipyard](https://github.com/ucb-bar/chipyard),
covering the full path from register-only compute to autonomous memory access to a
real network-checksum algorithm. Built as a self-directed learning project to go
hands-on with Chisel and Chipyard's RoCC accelerator interface, modeled structurally
on rocket-chip's built-in `AccumulatorExample`.

Every operation described below is verified against a real Verilator simulation of a
Rocket core with this accelerator attached, not just elaborated — see
[Verification](#verification).

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
| 3 | `fold_mem(ptr)` | Loads one word from memory, XORs it into `accum` |
| 4 | `fold_mem_n(ptr, n)` | Autonomously streams `n` words from memory, XOR-folding each one, with no further core instructions per word |
| 5 | `write_result(dst)` | Writes the current `accum` value to memory |
| 6 | `checksum_reset()` | Resets the independent checksum accumulator |
| 7 | `checksum_add_n(ptr, n)` | Streams `n` memory beats into an RFC 1071 (Internet checksum) one's-complement running sum |
| 8 | `checksum_finalize()` | Folds the 64-bit checksum accumulator to 16 bits and returns the complemented result |

Funct=1-5 operate on a dedicated XOR accumulator; funct=6-8 operate on a completely
separate checksum accumulator, so the two capabilities can't interfere with each
other's state.

## Design

The whole design lives in one file, [`src/main/scala/XorFoldAccelerator.scala`](src/main/scala/XorFoldAccelerator.scala),
built around a small shared FSM (`sIdle` → `sMemReq` → `sMemResp`) that every
memory-touching operation reuses:

- **Register-only ops** (funct=0/1/2) never leave `sIdle` — pure combinational/register
  logic, no memory involved.
- **Streaming reads** (funct=3/4/7) issue one `HellaCacheIO` load per word, advancing a
  pointer and decrementing a counter as each request is accepted, looping back to
  `sMemReq` until the count reaches zero — the core issues one instruction and the
  accelerator autonomously walks the buffer.
- **Writes** (funct=5) use the same FSM with `M_XWR` instead of `M_XRD`, latching the
  value to store at command-accept time so the transaction is self-contained.
- **The RFC 1071 checksum** (funct=6/7/8) reuses the identical FSM and memory path as
  the XOR fold — the only difference is what the response handler does with the loaded
  data (one's-complement add with end-around carry, instead of XOR) — plus a
  byte-swap step per 16-bit lane to correctly compute network-byte-order checksums
  over RISC-V's little-endian memory layout, and an explicit finalize step (rather than
  folding it into the generic read path) so `read()`'s meaning never depends on which
  operation ran before it.

## Verification

[`chipyard-integration/xorfold.c`](chipyard-integration/xorfold.c) is a baremetal test
covering every operation above, run against
`chipyard/sims/verilator/simulator-chipyard.harness-XorFoldRoCCConfig` — a real
Verilator simulation of a Rocket core with this accelerator attached via a custom
Chipyard config (`XorFoldRoCCConfig`). It includes:

- Hand-computed expected values for every check (e.g. `fold(5,3)` then `fold(1,1)` →
  `0 ^ 8 ^ 2 = 10`).
- An independent software reference implementation of the RFC 1071 checksum,
  cross-checked against both a hand-computed value and the hardware's result.
- A dedicated check that `checksum_reset()` doesn't disturb the unrelated XOR
  accumulator.

Result: **exit code 0** — all checks pass. The test's own pass/fail signal was verified
non-trivial by deliberately breaking an expected value and confirming the simulator
correctly reports `*** FAILED ***` with a nonzero exit code.

Along the way, a couple of real bugs were caught and fixed with hardware evidence
rather than guessing — worth mentioning because they're the kind of bug that doesn't
show up until you actually run something:
- A missing `io.mem.req.bits.mask` on the write path meant stores completed
  "successfully" from the FSM's point of view but never actually wrote any bytes —
  found via a VCD waveform trace showing the request was correct but memory never
  changed.
- A test that looked like it verified a memory write actually didn't, because the
  destination wasn't `volatile` — GCC constant-folded the "read after write" away at
  compile time, confirmed via disassembly.

## Repository layout

- [`src/main/scala/XorFoldAccelerator.scala`](src/main/scala/XorFoldAccelerator.scala) —
  the accelerator itself. This is the actual design work.
- [`chipyard-integration/`](chipyard-integration/) — everything needed to wire this
  accelerator into a Chipyard checkout (submodule registration, `build.sbt` project,
  config fragment, CMake test target) plus the verification test, packaged so the
  integration is reproducible from a fresh Chipyard clone without needing a maintained
  fork. See [`chipyard-integration/README.md`](chipyard-integration/README.md) for
  exact steps.

The Chipyard-side wiring (config fragments, `build.sbt` project registration) is
necessary "glue" following Chipyard's own documented submodule pattern for adding a
generator — not a novel contribution. The accelerator design and its verification are
the actual work.

## Out of scope

This project intentionally does not attempt real networking, multi-core, or
multi-board work — it's a single Rocket core plus one custom accelerator, fully
simulated. No real hardware (FPGA/ASIC) target has been attempted.

## Roadmap

- Memory access beyond simple loads/stores (e.g. a real TileLink-based streaming
  path, closer to rocket-chip's `CharacterCountExample`).
- RFC 1624 incremental checksum update, reusing the existing one's-complement adder.
- BOOM (out-of-order core) instead of Rocket.
