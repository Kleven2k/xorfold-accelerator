# Chipyard integration

This accelerator (`XorFoldAccelerator.scala`) is framework-agnostic Chisel/RoCC IP —
it only depends on rocket-chip's `LazyRoCC` API. It does nothing on its own; it has to
be wired into a Chipyard checkout as a generator submodule plus some SoC-level config.

That wiring lives in the **chipyard clone**, not in this repo, because it only makes
sense bolted onto a full Chipyard tree (build.sbt project graph, config fragments,
the `tests/` CMake build). This directory exists so that wiring is still reproducible
from a fresh chipyard checkout without needing a maintained chipyard fork.

## Contents

- `chipyard-wiring.patch` — diff against chipyard's own tracked files:
  - `.gitmodules` / submodule pointer for `generators/xorfold-accelerator`
  - `build.sbt` — adds the `xorfold` sbt project, depends on `rocketchip`
  - `generators/chipyard/.../config/fragments/RoCCFragments.scala` — `WithXorFoldRoCC`
    config fragment (uses `OpcodeSet.custom3`; custom1/custom2 are already used by
    chipyard's built-in `WithAccumulatorRoCC` / `WithCharacterCountRoCC` examples)
  - `generators/chipyard/.../config/RoCCAcceleratorConfigs.scala` — `XorFoldRoCCConfig`
    (single Rocket core + the accelerator)
  - `tests/CMakeLists.txt` — adds the `xorfold` / `xorfold-dump` build targets
- `xorfold.c` — the baremetal RoCC test (not part of the patch since it's a new file;
  copy it in directly). Exercises funct=0 (reset), funct=1 (fold), funct=2 (read), and
  funct=3 (fold-from-memory via `io.mem`) via `tests/rocc.h`'s `ROCC_INSTRUCTION*`
  macros. See this repo's `CONTEXT.md` for the verified test sequence and pass/fail
  behavior.

**Important — these two files are versioned together with the accelerator submodule
commit, not independently.** `xorfold.c`'s funct=3 test only passes against an
accelerator checkout that actually has the `io.mem` logic (commit `28613a6` or later).
If you pin `generators/xorfold-accelerator` to an older commit (e.g. `b698a25`, funct=0/1/2
only) but use the current `xorfold.c`, the funct=3 check will fail — not because
anything is broken, but because the hardware and test have drifted apart. When applying
this integration, make sure the submodule commit and the copied `xorfold.c` come from
the same point in this repo's history (in practice: just use the `HEAD` of both).

## Applying to a fresh Chipyard checkout

Assumes a working Chipyard install (`./build-setup.sh riscv-tools` already run).

```bash
cd ~/projects/chipyard

# 1. Add this repo as a generator submodule
git -c protocol.file.allow=always submodule add <xorfold-accelerator-repo-url> generators/xorfold-accelerator
# (chipyard-wiring.patch already updates .gitmodules to match this path — if your
#  remote URL differs from what's in the patch, edit .gitmodules after applying, or
#  add the submodule first and let git's own .gitmodules edit take precedence)

# 2. Apply the rest of the wiring
git apply ~/projects/xorfold-accelerator/chipyard-integration/chipyard-wiring.patch

# 3. Copy in the test
cp ~/projects/xorfold-accelerator/chipyard-integration/xorfold.c tests/xorfold.c

# 4. Build and run (see main README/CONTEXT.md for full verified command sequence)
cd sims/verilator
make CONFIG=XorFoldRoCCConfig

cd ../../tests
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
cmake --build build --target xorfold

cd ../sims/verilator
./simulator-chipyard.harness-XorFoldRoCCConfig ../../tests/build/xorfold.riscv
# exit code 0 == pass
```

## Updating xorfold.c after editing the test

`chipyard/tests/xorfold.c` is the real, live test file — it's what actually gets
compiled and run. `xorfold.c` in this directory is a **plain copy**, not a symlink and
not tracked by any build tooling, kept here so this repo has a self-contained record
of the test that verified each hardware milestone. Editing the copy in this repo does
nothing; it won't get compiled, run, or fed back into chipyard.

Workflow whenever you change and verify something in `chipyard/tests/xorfold.c`:

```bash
# 1. Edit chipyard/tests/xorfold.c, then build + run it there to confirm it passes:
cd ~/projects/chipyard/tests
cmake --build build --target xorfold
cd ../sims/verilator
./simulator-chipyard.harness-XorFoldRoCCConfig ../../tests/build/xorfold.riscv
# exit code 0 == pass

# 2. Once it passes, copy it into this repo and commit:
cp ~/projects/chipyard/tests/xorfold.c ~/projects/xorfold-accelerator/chipyard-integration/xorfold.c
cd ~/projects/xorfold-accelerator
git add chipyard-integration/xorfold.c
git commit -m "..."
git push
```

Skipping step 2 just means this repo's copy silently goes stale relative to the real
test — not a build break, just a drift worth avoiding.

## Regenerating this patch after further changes

If `XorFoldAccelerator.scala` changes in a way that needs new chipyard-side config
(e.g. new funct codes, memory-access support requiring `io.mem` wiring changes to
`RoCCFragments.scala`), regenerate the patch from the chipyard clone. Use `git diff
HEAD` (not plain `git diff`) — the `.gitmodules` submodule registration is staged
rather than a plain working-tree edit, so a plain `git diff` silently omits it and
produces a patch that fails to add the submodule on a fresh apply (this happened once
already — see git history on this file):

```bash
cd ~/projects/chipyard
git diff HEAD -- .gitmodules build.sbt \
  generators/chipyard/src/main/scala/config/RoCCAcceleratorConfigs.scala \
  generators/chipyard/src/main/scala/config/fragments/RoCCFragments.scala \
  tests/CMakeLists.txt \
  > ~/projects/xorfold-accelerator/chipyard-integration/chipyard-wiring.patch
cp tests/xorfold.c ~/projects/xorfold-accelerator/chipyard-integration/xorfold.c
```

Then commit the updated patch/test file in this repo. The chipyard clone itself stays
disposable — it does not need its own commit history for these edits.
