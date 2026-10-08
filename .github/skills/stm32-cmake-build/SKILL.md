---
name: stm32-cmake-build
description: Configure, build, and diagnose the CurRobo STM32H723 CMake firmware. Use for compiler or linker errors, source/include discovery, toolchain and preset issues, ELF/HEX/BIN/MAP generation, linker scripts, startup code, firmware size, or validating another code change.
---

# STM32 CMake build

## Use repository facts

- Presets are case-sensitive: `Debug` and `Release`.
- Configure Debug with `cmake --preset Debug`.
- Build Debug with `cmake --build --preset Debug`.
- The current executable target is `test`.
- User sources are recursively globbed from `Driver`, `Modules`, `App`, `BSP`,
  and `Robo`. Reconfigure after adding or removing source files.

Do not invent an IDE build, target name, build directory, or toolchain path.

## Diagnose from the first causal error

1. Capture the exact command and complete first error.
2. Classify configure, compile, assemble, or link failure.
3. Verify compiler identity/version and compile command when relevant.
4. For missing symbols, inspect declaration, definition, linkage, conditional
   compilation, object inclusion, archive order, and C/C++ `extern "C"`.
5. For overflow or placement, inspect the linker script and MAP file.
6. Fix the cause. Do not globally disable warnings or add arbitrary libraries.

## Validate artifacts

Report build exit status, new warnings, ELF/MAP and conversion artifacts that
actually exist, and Flash/RAM/section change when tools expose it. For source
changes, confirm the file appears in `compile_commands.json` or verbose build
output. Compilation validates syntax/linkage, not target hardware behavior.
