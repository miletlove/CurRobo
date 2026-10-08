---
applyTo: "App/**,BSP/**,Modules/**,Robo/**,**/*.c,**/*.h,**/*.cpp,**/*.hpp"
---

# CurRobo firmware implementation

- Preserve the current BSP → module → application/service direction. Check
  existing interfaces before creating another abstraction.
- Document units and execution context for public APIs. Use fixed-width integer
  types for protocol and hardware data.
- Keep IRQ and HAL callbacks bounded: capture/acknowledge data, update minimal
  state, and defer control work where practical.
- Shared ISR/task data needs a deliberate atomic, critical-section, queue,
  notification, or ownership design. `volatile` alone is insufficient.
- Validate array bounds, DLC/length fields, device IDs, enum values, NaN/Inf,
  timeout behavior, and actuator-safe defaults.
- Avoid heap allocation and exceptions unless the existing subsystem explicitly
  supports them.
- For periodic control, state the requested frequency, measured worst-case
  execution time, jitter source, overrun behavior, and watchdog interaction.
- For FDCAN changes, verify bit timing, standard/extended ID, DLC encoding,
  filter destination, Message RAM offsets, notification activation, bus-off
  recovery, and concurrency between callbacks and transmitters.
- For DMA changes, perform the STM32H7 memory/cache checklist before completion.
