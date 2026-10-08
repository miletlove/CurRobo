---
applyTo: "Core/**,CurRobo.ioc,cmake/stm32cubemx/**"
---

# STM32CubeMX-managed content

- Treat `CurRobo.ioc` as the source of truth for generated peripheral, clock,
  GPIO, DMA, NVIC, and middleware configuration.
- Do not edit generated files outside explicit `USER CODE BEGIN/END` regions.
- Do not place substantial application logic in generated files. Add a narrow
  call into code owned by `BSP/`, `Modules/`, `App/`, or `Robo/`.
- Before proposing a generated-code change, explain the corresponding CubeMX
  setting. Prefer asking the user to regenerate from CubeMX.
- After regeneration, inspect the diff for lost user sections, changed IRQ
  priorities, clocks, DMA assignments, middleware configuration, and FDCAN
  Message RAM layout.
- Never edit `cmake/stm32cubemx/CMakeLists.txt` merely to add user-owned sources;
  the top-level CMake owns those sources.
