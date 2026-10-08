# CurRobo repository instructions

## Project facts

- Target: STM32H723VGT6, Cortex-M7.
- Framework: STM32CubeMX-generated STM32 HAL with FreeRTOS.
- Build: CMake 3.22+, Ninja, GNU Arm Embedded toolchain.
- Configure and build with the existing case-sensitive presets:
  `cmake --preset Debug` and `cmake --build --preset Debug`.
- The current CMake executable target is `test`. Do not assume the target is named
  after the repository.
- Debugging uses SEGGER Ozone with DAPLink and the current ELF.
- Treat `CurRobo.ioc`, the schematic/hardware information supplied by the user,
  and the existing code as project facts. Do not invent pin mappings, clocks,
  DMA requests, CAN IDs, units, control rates, or safety limits.

## Architecture and ownership

- The user owns the system architecture. Preserve it unless the user explicitly
  requests an architecture change.
- Put board and HAL adaptation in `BSP/`, reusable device/protocol drivers in
  `Modules/`, application behavior in `App/`, and orchestration/services in
  `Robo/`.
- `Core/` is STM32CubeMX-managed. Do not edit generated content outside explicit
  `USER CODE BEGIN/END` regions. Prefer integration outside `Core/`.
- Do not copy STM32F4 examples from `Docs/` directly into this H7 project.
  Extract behavior and port it against the H723 HAL, memory, cache, DMA, clock,
  interrupt, and FDCAN constraints.

## Engineering rules

- Translate nontrivial prompts into explicit outcomes, non-goals, constraints,
  assumptions, observable acceptance criteria, and a dependency-aware task plan
  before implementation. Keep this proportional for small reversible changes.
- Ask only decision questions that cannot be answered from the repository and
  materially change hardware behavior, interfaces, timing, compatibility, or
  safety. State reversible assumptions instead of blocking on minor details.
- Keep requirement, implementation choice, and recommendation distinct.
- Inspect related declarations, implementations, call sites, callbacks, IRQ
  handlers, `.ioc`, and CMake configuration before editing.
- Make the smallest coherent change. Do not add speculative abstractions.
- Keep HAL access behind BSP/device boundaries; keep control policy out of IRQs.
- Avoid blocking waits and `HAL_Delay()` in runtime control paths. Every wait,
  communication operation, and state transition needs a timeout or safe failure
  path.
- State units, coordinate conventions, execution context, ownership, and valid
  ranges at module boundaries.
- Do not treat `volatile` as synchronization. Make ISR/task sharing explicit.
- Use the existing logging interface when present; do not scatter `printf()`.
- Preserve unrelated user changes and never rewrite files only for formatting.

## STM32H7 mandatory review

For every DMA buffer, determine its memory region, DMA accessibility, cache-line
alignment, ownership, and required D-cache clean/invalidate operation. Check MPU,
linker placement, interrupt priority, FreeRTOS `FromISR` rules, and callback
chains. Compilation success alone is not proof of correct hardware behavior.

## Completion standard

Before claiming completion:

1. Build the applicable preset and report the exact command and result.
2. Confirm new sources are part of the target. This project uses recursive CMake
   globbing, so reconfigure after adding or removing source files.
3. Report warnings, ELF/MAP availability, and Flash/RAM impact when available.
4. Review timeout, failure, concurrency, ISR, DMA/cache, and watchdog behavior.
5. Provide a board verification procedure with expected observations.
6. Summarize changed files, behavior changes, assumptions, and residual risks.

For integration-sensitive work, also provide a verification matrix that maps
acceptance criteria and risks to review/static, host, build/link, target, or HIL
evidence. Report unexecuted evidence explicitly. Do not equate a successful
build with hardware validation.

Before recommending merge, inspect the complete diff and generated changes,
confirm required gates, identify rollback, and recommend an integration strategy
based on branch publication and history. Any Git mutation still requires explicit
user authorization.

## Git safety

- Begin with read-only inspection: `git status --short --branch`,
  `git log --oneline -5`, and relevant diffs.
- The worktree may already contain user changes. Never overwrite or revert them.
- Do not run `git add`, `commit`, `switch`, `checkout`, `merge`, `rebase`,
  `push`, branch deletion, `reset`, or `clean` without explicit user approval.
- Do not use `git pull` as an implicit synchronization strategy. Fetch and report
  ahead/behind state, then let the user choose merge or rebase.
- Never commit build outputs, local Ozone state, secrets, or machine-specific
  toolchain paths.
