---
name: stm32h7-feature-dev
description: Implement and review CurRobo STM32H723 firmware features using HAL, FreeRTOS, C/C++, and the existing BSP/Modules/App/Robo architecture. Use for new drivers, communication, motors, sensors, state machines, periodic control, services, algorithms, refactors, and bug fixes that require code changes.
---

# STM32H7 feature development

## Establish the contract

Extract the objective, acceptance criteria, hardware/peripheral instances,
inputs/outputs, units, rate/deadline, execution context, ownership, timeout,
safe state, and permitted file scope. Discover missing noncritical facts from
the repository. Ask only when missing information changes hardware behavior,
public interfaces, or safety.

## Inspect before design

1. Read declarations, implementations, call sites, initialization order,
   callbacks/IRQs, tasks, and nearby documentation.
2. Inspect `CurRobo.ioc` and generated configuration for relevant hardware.
3. Inspect CMake inclusion and compiler language boundaries.
4. Identify existing user changes and keep them intact.
5. State the affected modules, data flow, timing, memory, CPU, bus, and failure
   impact before editing. Keep this proportional to the task.

## Implement

- Preserve user-owned architecture and existing public contracts.
- Make the smallest complete change; avoid demo-style parallel implementations.
- Put HAL adaptation in `BSP/`, reusable device/protocol logic in `Modules/`,
  application policy in `App/`, and orchestration/services in `Robo/`.
- Keep ISR work bounded and nonblocking. Define task/ISR synchronization.
- Give every wait, state transition, and communication path a timeout or safe
  failure behavior.
- Use named constants with units. Validate lengths, IDs, ranges, and invalid
  floating-point values at trust boundaries.
- Avoid runtime heap allocation and hidden global mutable state.
- For control code, make sample time explicit and check saturation, integral
  windup, derivative noise, initialization, reset, and actuator-safe output.

## Verify

1. Reconfigure if files were added or removed.
2. Build with `cmake --preset Debug` then
   `cmake --build --preset Debug`, unless the user requests Release.
3. Review all new warnings and the relevant diff.
4. Apply the hardware-integration checklist for peripherals, DMA, IRQs, FDCAN,
   cache, MPU, or memory placement.
5. Check watchdog servicing and failure behavior.
6. Provide an executable board test: setup, stimulus, observations, expected
   values/timing, and failure signature.
7. Report changed files, assumptions, resource impact, and residual risks.

Do not claim runtime success when only compilation was performed.
