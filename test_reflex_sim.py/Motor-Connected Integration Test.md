# Motor-Connected Integration Test Protocol (ESP32-S3 Reflex Loop)

## Safety-first prerequisites
- Confirm motor can spin freely and is physically restrained/safe.
- Keep an accessible hardware kill path (power switch or inline disconnect).
- Use current-limited supply for first power-on.
- Start with command state: `desired=0`, low/zero PWM.

## Test 1: Safe initial conditions
1. Power board and motor driver with motor connected.
2. Send `reset` command.
3. Send `set_desired:0` and `set_pwm:0`.
4. Verify motor remains stopped.
5. Confirm `STATUS_OK` or benign idle status with no emergency/stall bits.

**Pass:** Motor does not move; status is stable and non-fault.
**Fail:** Motion at zero command, fault bits set, or unstable status.

## Test 2: Basic motor response command sequence
1. Send `set_desired` to a small target (example: `20 ticks`).
2. Optionally send conservative baseline PWM (`20-40`) if needed for startup torque.
3. Observe encoder count trend toward desired value.
4. Increase target in small steps (`20 -> 40 -> 60`) and verify tracking.
5. Reduce back to `0` and confirm controlled slowdown/stop.

**Pass:** Motor responds directionally correct, encoder tracks command, no runaway.
**Fail:** Wrong direction, no response, oscillation, or persistent large drift.

## Test 3: Emergency stop verification (critical)
1. While motor is in motion, issue `emerg` command.
2. Measure response time from command send to PWM drop (serial + meter/scope).
3. Confirm PWM becomes `0` and `STATUS_EMERG` latches.
4. Attempt normal command (`set_desired`) without reset and ensure motion does not resume.
5. Issue `reset`, then verify normal control is restored.

**Pass:** Immediate stop, emergency status latched, requires reset to recover.
**Fail:** Delayed/no stop, PWM remains active, or emergency state not latched.

## Test 4: Watchdog behavior
1. Introduce intentional timing overrun (e.g., debug build delay/instrumentation).
2. Run reflex loop under over-budget condition.
3. Verify watchdog status bit is set and PWM forced to `0`.
4. Remove overrun and verify recovery path (`reset` if required by design).

**Pass:** Deterministic watchdog trip on budget violation and safe output shutdown.
**Fail:** No watchdog action during overruns, or unsafe PWM remains active.

## If motor runs away: immediate safety procedure
1. Trigger `emerg` command immediately.
2. If no immediate stop, cut motor power with hardware kill path.
3. Disconnect load/drive stage before further software debugging.
4. Inspect:
   - LEDC/PCNT register address mapping
   - Motor polarity / direction mapping
   - Encoder wiring phase/order
   - Control gain (correction too aggressive)
   - Command checksum/parse integrity
5. Resume only with low-voltage, low-inertia test setup.

## Summary pass/fail gate for integration sign-off
- **Pass sign-off:** All four tests pass with repeatable results across at least 3 power cycles.
- **Fail sign-off:** Any emergency-stop or watchdog failure, or any uncontrolled motion event