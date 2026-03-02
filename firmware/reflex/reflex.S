    .section .text
    .align 4
    .global _start

    // Constants
    .equ PWM_REG_ADDR, 0x3FF49000   // placeholder PWM peripheral address
    .equ ENCODER_REG_ADDR, 0x3FF4A000 // placeholder encoder register address
    .equ LOOP_CYCLES, 12000          // 50us at 240MHz
    .equ MAX_LOOP_CYCLES, 14400      // 60us watchdog threshold
    .equ STATUS_OK, 0x01
    .equ STATUS_DRIFT, 0x02
    .equ STATUS_STALL, 0x04
    .equ STATUS_EMERG, 0x08

    // Offsets in data block
    .equ OFF_DESIRED, 0x00
    .equ OFF_MAX_DRIFT, 0x02
    .equ OFF_PWM, 0x03
    .equ OFF_STATUS, 0x04
    .equ OFF_CMD, 0x05
    .equ OFF_CMD_ARG, 0x06
    .equ OFF_CMD_CSUM, 0x07
    .equ OFF_TEMP, 0x08
    .equ OFF_LAST, 0x0A
    .equ OFF_SCRATCH, 0x0C

    .equ MASK_CLEAR_FAULTS, 0xF1   // clear drift/stall/emerg bits

    .section .data
    .align 4
    .global reflex_base
reflex_base:
    .short 0          // DESIRED_TICKS 0x00
    .byte 8           // MAX_DRIFT     0x02
    .byte 0           // PWM_VALUE     0x03
    .byte 0           // STATUS_FLAGS  0x04
    .byte 0,0,0       // CMD_BUFFER    0x05-0x07
    .short 0          // INTERNAL_TEMP 0x08-0x09
    .short 0          // LAST_ENCODER  0x0A-0x0B
    .word 0           // SCRATCH       0x0C-0x0F

    .section .text

_start:
    movi    a2, reflex_base        // base pointer

    // clear fields
    movi    a3, 0
    s16i    a3, a2, OFF_DESIRED
    s8i     a3, a2, OFF_PWM
    s8i     a3, a2, OFF_STATUS
    s8i     a3, a2, OFF_CMD
    s8i     a3, a2, OFF_CMD_ARG
    s8i     a3, a2, OFF_CMD_CSUM
    s16i    a3, a2, OFF_TEMP
    s16i    a3, a2, OFF_LAST

    movi    a3, 8
    s8i     a3, a2, OFF_MAX_DRIFT

    // PWM safe state
    movi    a4, PWM_REG_ADDR
    movi    a5, 0
    s8i     a5, a4, 0

reflex_loop:
    rsr.ccount a7
    s32i    a7, a2, OFF_SCRATCH

    movi    a8, ENCODER_REG_ADDR
    l16ui   a9, a8, 0       // ACTUAL
    s16i    a9, a2, OFF_LAST

    l16ui   a10, a2, OFF_DESIRED

    sub     a11, a9, a10    // error
    mov     a12, a11
    bltz    a12, drift_neg
    nop
    j       drift_abs_done

 drift_neg:
    neg     a12, a12

 drift_abs_done:
    s16i    a12, a2, OFF_TEMP

    l8ui    a13, a2, OFF_MAX_DRIFT
    bltu    a12, a13, drift_ok
    beq     a12, a13, drift_ok

    l8ui    a14, a2, OFF_STATUS
    movi    a3, STATUS_STALL
    or      a14, a14, a3
    s8i     a14, a2, OFF_STATUS
    s8i     a5, a2, OFF_PWM
    s8i     a5, a4, 0
    j       loop_timing_end

 drift_ok:
    l8ui    a14, a2, OFF_STATUS
    beqz    a12, clear_drift
    movi    a3, STATUS_DRIFT
    or      a14, a14, a3
    j       drift_bits_done

 clear_drift:
    movi    a15, MASK_CLEAR_FAULTS
    and     a14, a14, a15
    movi    a3, STATUS_OK
    or      a14, a14, a3

 drift_bits_done:
    s8i     a14, a2, OFF_STATUS

    sub     a15, a10, a9
    srai    a15, a15, 2

    l8ui    a6, a2, OFF_PWM
    add     a6, a6, a15

    bltz    a6, clamp_zero
    movi    a7, 255
    blt     a6, a7, pwm_ok
    mov     a6, a7
    j       pwm_store

 clamp_zero:
    movi    a6, 0
    j       pwm_store

 pwm_ok:
    nop

 pwm_store:
    s8i     a6, a2, OFF_PWM
    s8i     a6, a4, 0

    l8ui    a7, a2, OFF_CMD
    beqz    a7, watchdog
    l8ui    a8, a2, OFF_CMD_ARG
    l8ui    a9, a2, OFF_CMD_CSUM
    xor     a10, a7, a8
    bne     a10, a9, cmd_done

    movi    a11, 1
    beq     a7, a11, cmd_set_desired
    movi    a11, 2
    beq     a7, a11, cmd_set_pwm
    movi    a11, 3
    beq     a7, a11, cmd_emerg
    movi    a11, 4
    beq     a7, a11, cmd_reset
    j       cmd_done

 cmd_set_desired:
    extui   a12, a8, 0, 8
    s16i    a12, a2, OFF_DESIRED
    j       cmd_done

 cmd_set_pwm:
    extui   a12, a8, 0, 8
    s8i     a12, a2, OFF_PWM
    s8i     a12, a4, 0
    j       cmd_done

 cmd_emerg:
    s8i     a5, a2, OFF_PWM
    s8i     a5, a4, 0
    l8ui    a12, a2, OFF_STATUS
    movi    a3, STATUS_EMERG
    or      a12, a12, a3
    s8i     a12, a2, OFF_STATUS
    j       cmd_done

 cmd_reset:
    movi    a12, MASK_CLEAR_FAULTS
    l8ui    a13, a2, OFF_STATUS
    and     a13, a13, a12
    movi    a3, STATUS_OK
    or      a13, a13, a3
    s8i     a13, a2, OFF_STATUS

 cmd_done:
    s8i     a5, a2, OFF_CMD
    s8i     a5, a2, OFF_CMD_ARG
    s8i     a5, a2, OFF_CMD_CSUM

 watchdog:
    rsr.ccount a7
    l32i    a8, a2, OFF_SCRATCH
    sub     a9, a7, a8
    movi    a10, MAX_LOOP_CYCLES
    bltu    a9, a10, loop_wait

    s8i     a5, a2, OFF_PWM
    s8i     a5, a4, 0
    l8ui    a11, a2, OFF_STATUS
    movi    a3, STATUS_EMERG
    or      a11, a11, a3
    s8i     a11, a2, OFF_STATUS

 loop_wait:
    addi    a12, a8, LOOP_CYCLES
 wait_spin:
    rsr.ccount a13
    bltu    a13, a12, wait_spin

 loop_timing_end:
    j       reflex_loop

