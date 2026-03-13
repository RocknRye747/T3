# ESP32 Reflex Microkernel (Xtensa LX6)

This note documents the cycle-deterministic motor reflex loop implemented in `esp32_reflex_microkernel.S`. The code builds to a flat 331-byte binary (`reflex.bin`) suitable for embedding directly in a boot ROM stub or as a freestanding firmware blob. Placeholder peripheral addresses (`PWM_REG_ADDR`, `ENCODER_REG_ADDR`) can be remapped during linking.

## Memory Layout (12-byte state block)

| Offset | Symbol            | Size | Purpose |
| ------ | ----------------- | ---- | ------- |
| 0x00   | DESIRED_TICKS     | 2    | Target encoder ticks |
| 0x02   | MAX_DRIFT         | 1    | Allowed drift (ticks) |
| 0x03   | PWM_VALUE         | 1    | Current PWM command |
| 0x04   | STATUS_FLAGS      | 1    | OK/DRIFT/STALL/EMERG bits |
| 0x05   | CMD_BUFFER[3]     | 3    | CMD, ARG0, CHECKSUM |
| 0x08   | INTERNAL_TEMP     | 2    | Scratch drift/error |
| 0x0A   | LAST_ENCODER      | 2    | Last encoder sample |
| 0x0C   | SCRATCH           | 4    | Timing start (ccount) |

## Boot / Reset Path

The `_start` routine zeroes command/status/PWM fields, seeds `MAX_DRIFT=8`, forces PWM low, and then jumps into `reflex_loop`. No stack or dynamic memory is touched, keeping execution deterministic. Timing base is tracked via `rsr.ccount` with the start cycle stored at `SCRATCH` for each iteration.

## Core Loop Behavior

1. Capture `ccount` at entry and push to `SCRATCH` for watchdog math.  
2. Sample encoder (`ENCODER_REG_ADDR`) and store to `LAST_ENCODER`.  
3. Compute absolute drift vs `DESIRED_TICKS`, write to `INTERNAL_TEMP`, and compare with `MAX_DRIFT`. Excess drift forces PWM=0 and sets `STATUS_STALL`.  
4. Otherwise, DRIFT/OK bits are updated and a proportional delta (`error>>2`) is applied to PWM with clamp to [0,255].  
5. Command parser handles checksumed 3-byte commands for setpoint, direct PWM, emergency stop, and flag reset.  
6. Watchdog compares elapsed cycles to `MAX_LOOP_CYCLES` (60 µs worst-case at 240 MHz) and forces EMERG + PWM off if exceeded.  
7. Busy-wait until `LOOP_CYCLES` (50 µs) boundary before jumping to the next iteration.

## Machine Code (flat binary)

`reflex.bin` size: **331 bytes**. Hex bytes (space separated):

```
00 00 08 00 00 00 00 00 00 00 00 00 00 00 00 00 05 32 42 06 32 42 07 32 52 04 32 52 05 0c 83 32
42 02 41 00 00 0c 05 52 44 00 70 ea 03 79 32 81 00 00 92 18 00 92 52 05 a2 12 00 a0 b9 c0 cd 0b 96
4c 00 3d f0 86 00 00 c0 c0 60 c2 52 04 d2 02 02 d7 3c 17 d7 1c 14 e2 02 04 0c 43 30 ee 20 e2 42 04
52 42 03 52 44 00 06 37 00 00 e2 02 04 8c 6c 0c 23 30 ee 20 86 02 00 f2 a0 f1 f0 ee 10 0c 13 30 ee
20 e2 42 04 90 fa c0 f0 f2 21 62 02 03 fa 66 96 a6 00 72 a0 ff 77 26 09 6d 07 86 01 00 0c 06 46 00
00 3d f0 62 42 03 62 44 00 72 02 05 16 77 06 82 02 06 92 02 07 80 a7 30 97 9a 52 0c 1b b7 17 13 0c
2b b7 17 17 0c 3b b7 17 1e 0c 4b b7 17 2d c6 0e 00 00 00 80 c0 74 c2 52 00 06 0c 00 80 c0 74 c2 42
03 c2 44 00 06 09 00 52 42 03 52 44 00 c2 02 04 0c 83 30 cc 20 c2 42 04 06 04 00 c2 a0 f1 d2 02 04
c0 dd 10 0c 13 30 dd 20 d2 42 04 52 42 05 52 42 06 52 42 07 70 ea 03 82 22 03 80 97 c0 a1 00 00 a7
39 11 52 42 03 52 44 00 b2 02 04 32 a0 08 30 bb 20 b2 42 04 c2 d8 2f c2 cc e0 d0 ea 03 c7 3d f9 86
b7 ff
```

## Disassembly Map

Key labels (from `reflex.dis`):

- `_start` init and PWM safe state.【F:reflex.dis†L15-L34】
- `reflex_loop` entry, encoder read, drift calc, stall path.【F:reflex.dis†L36-L74】
- Drift OK + proportional PWM clamp and store.【F:reflex.dis†L76-L106】
- Command parser and subcommands (set desired, set PWM, emergency, reset).【F:reflex.dis†L108-L162】
- Watchdog and 50 µs wait spinner.【F:reflex.dis†L164-L205】

## Cycle-Timing Notes (240 MHz reference)

- Typical path (no drift >0, no command): ~80–90 instructions, mostly single-cycle; estimated **~120–140 cycles (~0.5 µs)** for math + status, plus busy-wait to fill the 12,000-cycle budget for 50 µs.  
- Worst-case (stall trigger + command decode + watchdog EMERG): ~160–180 instructions including branches; conservatively **<500 cycles (~2.1 µs)** before the wait spinner pads to 50 µs. Even if watchdog trips, PWM is forced low immediately and flags updated before looping.  
- Busy-wait ensures deterministic 50 µs cadence; watchdog trips at 14,400 cycles (60 µs) to assert EMERG and zero PWM.

## Integration Guide

- Place the 12-byte state block at `reflex_base` (or relocate with linker). Higher-level code writes commands into `CMD_BUFFER` with checksum, updates `DESIRED_TICKS`/`MAX_DRIFT`, and polls `STATUS_FLAGS` + `PWM_VALUE`.  
- Map `PWM_REG_ADDR` to the SOC LEDC/ MCPWM duty register for the motor channel; map `ENCODER_REG_ADDR` to the encoder count register or a GPIO capture latch.  
- Bootloader should jump to `_start` with interrupts masked; the microkernel runs forever without stack usage, suitable for lockstep motor reflex control.
