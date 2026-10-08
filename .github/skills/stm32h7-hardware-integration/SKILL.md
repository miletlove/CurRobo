---
name: stm32h7-hardware-integration
description: Analyze STM32H723 peripheral integration, DMA and cache coherency, MPU and linker placement, interrupts, HAL callbacks, FreeRTOS interaction, FDCAN, SPI, UART, ADC, timers, and board bring-up. Use for hardware-facing changes and for faults that compile successfully but fail, corrupt data, or behave intermittently on target.
---

# STM32H7 hardware integration

## Trace the complete path

Trace configuration and execution from `CurRobo.ioc` through generated init/MSP,
NVIC/IRQ, HAL handler/callback, BSP, module, and application consumer. Verify the
actual instance, pins/AF, clocks, timing, DMA request, interrupt priority, and
initialization order. Do not infer these from a similar STM32F4 example.

## DMA and D-cache gate

For every DMA buffer, record:

1. Address, size, lifetime, alignment, and linker section.
2. STM32H7 memory domain and whether the selected DMA can access it.
3. CPU/DMA ownership during transfer.
4. Cache-line coverage, including neighboring-data corruption risk.
5. Clean-before-transmit and invalidate-after-receive requirements.
6. Whether a noncacheable MPU region is preferable.
7. Required barriers and when the buffer becomes valid.

Check cache maintenance address and length alignment; do not call cache
maintenance on an arbitrary unaligned range and assume correctness.

## IRQ and RTOS gate

- Compare IRQ priority against the project's FreeRTOS maximum syscall priority.
- Use only ISR-safe APIs from interrupts and request a context switch correctly.
- Keep callbacks bounded; do not format logs, block, allocate, or execute control
  algorithms in an IRQ.
- Define synchronization for every ISR/task shared object. `volatile` is not a
  critical section or memory-ownership protocol.
- Check callback symbol collisions because HAL callbacks are global weak hooks.

## FDCAN gate

Verify kernel clock and nominal/data bit timing, frame format, ID type, DLC
encoding, filters, FIFO destination, notification, error handling, bus-off
policy, and transmit concurrency. For FDCAN1/FDCAN2, verify nonoverlapping
Message RAM offsets and total layout against the device.

## Evidence

Build first, then specify board measurements using Ozone, RTT, DWT, CAN analyzer,
logic analyzer, or oscilloscope. Separate observed facts, hypotheses, and
unverified assumptions.
