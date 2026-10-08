---
name: verify-integrate-firmware
description: Design and execute proportional verification and integration workflows for CurRobo firmware changes. Use when defining tests, proving acceptance criteria, reviewing a completed change, preparing commits, comparing branches, handling merge readiness, or deciding whether a feature is safe to integrate. Covers host tests, static/build checks, target and hardware-in-loop evidence, regression review, merge gates, and rollback planning.
---

# Verify and integrate firmware

## Build an evidence matrix

Map every acceptance criterion and major risk to the cheapest test that can
actually detect failure:

| Layer | Purpose | Typical evidence |
|---|---|---|
| Review/static | Interfaces, ranges, concurrency, undefined behavior | Diff review, compiler warnings, static analysis |
| Host/unit | Pure algorithms, parsers, state machines, filters | Deterministic vectors and boundary tests |
| Build/link | Configuration and integration | Debug/Release build, MAP/ELF, symbol and size inspection |
| Target | HAL, IRQ, DMA, cache, timing, RTOS | Ozone, RTT, DWT, registers, runtime counters |
| HIL/system | Electrical buses, actuators, sensors, safety | CAN analyzer, logic analyzer, oscilloscope, controlled robot test |

Do not use mocks to claim hardware correctness. Do not require hardware for pure
logic that can be proven deterministically on the host.

## Design tests from behavior and risk

Cover as applicable:

- Nominal path
- Minimum/maximum and just-outside-boundary values
- Invalid IDs, lengths, DLC, enums, CRC, NaN/Inf, stale data
- Initialization, repeated initialization, reset, and recovery
- Timeout, missing device, bus-off, sensor disconnect, queue full
- ISR/task interleavings and ownership transitions
- Rate, jitter, worst-case execution time, stack, CPU, bus load
- DMA alignment/cache behavior and neighboring-buffer integrity
- Watchdog behavior and actuator-safe output
- Backward compatibility with existing modules/protocol peers

Record test setup, stimulus, expected result, tolerance, observation tool, and
cleanup/rollback. A passing result without identifying the exact firmware/ELF
and configuration is weak evidence.

## Establish the baseline

Before attributing a failure to the change:

1. Record branch/commit, dirty state, preset, toolchain, target board, and
   relevant configuration.
2. Reproduce the original behavior or run the closest existing check.
3. Build the current change using the repository preset.
4. Distinguish pre-existing failures from regressions.

Never modify unrelated code merely to make a broad test suite green.

## Apply merge gates

Mark the change **ready to propose for integration** only when:

- Requirements and non-goals are traceable to the diff.
- The diff contains no unrelated or unexplained generated changes.
- Debug build passes; Release build is required for timing-, optimization-, or
  release-sensitive changes.
- New warnings are resolved or explicitly justified.
- Required host, target, and HIL evidence is recorded.
- DMA/cache, IRQ/RTOS, FDCAN, timeout, watchdog, and safe-state reviews pass when
  relevant.
- Flash/RAM/stack/timing impact is acceptable or explicitly accepted.
- Public interfaces, protocols, calibration, and `.ioc` changes are documented.
- Rollback is clear and no uncommitted user work is endangered.

Compilation alone can satisfy only the build gate.

## Review and integrate safely

1. Inspect branch history, merge base, status, staged/unstaged/untracked files,
   and the full diff.
2. Group work into buildable, behavior-focused commits and propose Conventional
   Commit messages.
3. Fetch and report ahead/behind/divergence; do not default to `git pull`.
4. Recommend:
   - fast-forward when history is already linear,
   - rebase for a private unpublished feature branch when the user wants a clean
     history,
   - merge commit when preserving shared branch history matters,
   - squash when intermediate commits are not independently meaningful.
5. Predict likely conflicts and identify semantic conflicts in `.ioc`, generated
   code, CMake, callbacks, IRQ priorities, and FDCAN Message RAM.
6. Obtain explicit authorization before any Git state/history mutation.
7. After an authorized merge/rebase, rebuild and rerun integration-sensitive
   tests; do not assume pre-merge results remain valid.

Stop on ambiguous conflicts. Explain both sides and the behavioral consequence;
do not choose based only on line order.

## Report

Produce:

- Gate table: pass, fail, blocked, or not applicable
- Commands/tests actually run and their results
- Evidence not run and why
- Regression and resource assessment
- Recommended integration strategy
- Rollback point
- Residual risk and required user/hardware action

Use "ready to propose for integration," not "safe," when target/HIL evidence is
still missing.
