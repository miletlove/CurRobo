---
name: stm32-ozone-debug
description: Diagnose CurRobo STM32H723 target faults using SEGGER Ozone, DAPLink, ELF symbols, breakpoints, registers, RTT, DWT, and hardware measurements. Use for download or startup failures, resets, HardFault and other Cortex-M faults, stuck tasks, missing interrupts, peripheral failures, timing problems, and optimization-sensitive bugs.
---

# Ozone and DAPLink debugging

## Preserve evidence

State expected versus observed behavior, reproduction conditions, frequency,
current Git commit, build preset, ELF path/timestamp, optimization level, and
probe/reset settings. Confirm Ozone loaded the ELF produced by the current build.

Classify the failure before changing code: probe/download, reset/startup,
Cortex-M fault, IRQ/callback, RTOS/deadlock/stack, DMA/cache, peripheral/bus, or
control timing.

## Investigate one hypothesis at a time

1. Reproduce and record the stop PC/call stack.
2. Break at `Reset_Handler`, `main`, initialization, and the narrow target path
   as needed.
3. Inspect peripheral and NVIC state against `.ioc` and initialization.
4. Form one falsifiable hypothesis and define the observation that disproves it.
5. Use Ozone Watch/Memory/Registers, RTT, DWT, GPIO plus logic analyzer, CAN
   analyzer, or oscilloscope as appropriate.
6. Apply the smallest fix and repeat the original reproduction.

## Cortex-M fault record

Capture stacked R0-R3, R12, LR, PC, xPSR; MSP/PSP and EXC_RETURN; SCB CFSR,
HFSR, SHCSR, MMFAR, BFAR, and AFSR; current task and stack high-water mark; the
faulting instruction and source line. Decode valid-address flags before trusting
MMFAR/BFAR.

## Timing faults

Measure rather than infer. Record control period, worst-case execution time,
jitter, ISR duration, task latency, missed deadlines, and watchdog relationship.
Avoid timing conclusions based on UART logs alone.

Separate confirmed facts, leading hypothesis, next experiment, and remaining
unknowns in the report.
