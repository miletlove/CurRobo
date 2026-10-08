---
applyTo: "CMakeLists.txt,CMakePresets.json,cmake/**,*.ld,startup_*.s"
---

# Build and linker configuration

- Preserve the existing GNU Arm toolchain and `Debug`/`Release` preset names.
- Reconfigure after adding or deleting sources because the project uses
  `GLOB_RECURSE` without `CONFIGURE_DEPENDS`.
- Verify Cortex-M7 CPU, Thumb, FPU, float ABI, linker script, startup object,
  specs, and section garbage collection as one consistent set.
- Do not suppress a new warning globally to make a change pass.
- Do not change the linker script, startup code, memory regions, or section
  placement without reporting runtime and flashing consequences.
- After linker-related changes, inspect the MAP file and section sizes, not just
  the final exit code.
