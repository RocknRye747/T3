from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class ReflexSim:
    """Cycle-deterministic reflex-loop simulation for host-side validation."""

    DESIRED_TICKS: int = 0
    MAX_DRIFT: int = 8
    PWM_VALUE: int = 0
    STATUS_FLAGS: int = 0
    CMD_BUFFER: list[tuple[str, int | None, int]] = field(default_factory=list)
    INTERNAL_TEMP: int = 25
    LAST_ENCODER: int = 0
    watchdog_budget_us: int = 100

    STATUS_OK: int = 0x01
    STATUS_DRIFT: int = 0x02
    STATUS_STALL: int = 0x04
    STATUS_EMERG: int = 0x08
    STATUS_WATCHDOG: int = 0x10

    def __post_init__(self) -> None:
        self.STATUS_FLAGS = self.STATUS_OK

    @staticmethod
    def _clamp_pwm(value: int) -> int:
        return max(0, min(255, value))

    @staticmethod
    def build_checksum(command: str, value: int | None = None) -> int:
        payload = f"{command}:{'' if value is None else value}".encode("ascii")
        return sum(payload) & 0xFF

    def parse_command(self, command: str, value: int | None, checksum: int) -> bool:
        expected = self.build_checksum(command, value)
        if checksum != expected:
            return False

        self.CMD_BUFFER.append((command, value, checksum))

        if command == "set_desired" and value is not None:
            self.DESIRED_TICKS = int(value)
            self._clear_faults()
            self.STATUS_FLAGS = self.STATUS_OK
        elif command == "set_pwm" and value is not None:
            self.PWM_VALUE = self._clamp_pwm(int(value))
        elif command == "emerg":
            self.PWM_VALUE = 0
            self.STATUS_FLAGS = self.STATUS_EMERG
        elif command == "reset":
            self._clear_faults()
            self.PWM_VALUE = 0
            self.STATUS_FLAGS = self.STATUS_OK
        return True

    def _clear_faults(self) -> None:
        self.STATUS_FLAGS &= ~(self.STATUS_DRIFT | self.STATUS_STALL | self.STATUS_EMERG | self.STATUS_WATCHDOG)

    def step(self, actual_encoder_ticks: int, processing_time_us: int = 0) -> dict[str, int]:
        start = time.perf_counter()

        self.LAST_ENCODER = int(actual_encoder_ticks)
        error = self.DESIRED_TICKS - self.LAST_ENCODER
        drift = abs(error)

        if self.STATUS_FLAGS & self.STATUS_EMERG:
            self.PWM_VALUE = 0
        elif drift > self.MAX_DRIFT:
            self.PWM_VALUE = 0
            self._clear_faults()
            self.STATUS_FLAGS = self.STATUS_STALL
        else:
            correction = error >> 2
            self.PWM_VALUE = self._clamp_pwm(self.PWM_VALUE + correction)
            self._clear_faults()
            self.STATUS_FLAGS = self.STATUS_OK if drift == 0 else self.STATUS_DRIFT

        elapsed_us = int((time.perf_counter() - start) * 1_000_000) + processing_time_us
        if elapsed_us > self.watchdog_budget_us and not (self.STATUS_FLAGS & self.STATUS_EMERG):
            self.PWM_VALUE = 0
            self._clear_faults()
            self.STATUS_FLAGS = self.STATUS_WATCHDOG

        return {
            "drift": drift,
            "error": error,
            "pwm": self.PWM_VALUE,
            "status": self.STATUS_FLAGS,
            "elapsed_us": elapsed_us,
        }


def test_normal_operation_status_ok() -> None:
    sim = ReflexSim(DESIRED_TICKS=100, PWM_VALUE=128)
    result = sim.step(actual_encoder_ticks=100)
    assert result["drift"] == 0
    assert result["status"] == sim.STATUS_OK


def test_small_drift_sets_drift_status_and_adjusts_pwm() -> None:
    sim = ReflexSim(DESIRED_TICKS=100, MAX_DRIFT=8, PWM_VALUE=100)
    result = sim.step(actual_encoder_ticks=105)
    assert result["drift"] == 5
    assert result["status"] == sim.STATUS_DRIFT
    assert result["pwm"] == 98  # error=-5, correction=-2


def test_stall_detection_sets_status_and_pwm_zero() -> None:
    sim = ReflexSim(DESIRED_TICKS=100, MAX_DRIFT=8, PWM_VALUE=120)
    result = sim.step(actual_encoder_ticks=120)
    assert result["drift"] == 20
    assert result["status"] == sim.STATUS_STALL
    assert result["pwm"] == 0


def test_pwm_clamp_high_never_exceeds_255() -> None:
    sim = ReflexSim(DESIRED_TICKS=1000, MAX_DRIFT=5000, PWM_VALUE=250)
    result = sim.step(actual_encoder_ticks=0)
    assert result["pwm"] == 255


def test_pwm_clamp_low_never_below_0() -> None:
    sim = ReflexSim(DESIRED_TICKS=0, MAX_DRIFT=5000, PWM_VALUE=3)
    result = sim.step(actual_encoder_ticks=200)
    assert result["pwm"] == 0


def test_emergency_command_sets_emerg_and_zero_pwm() -> None:
    sim = ReflexSim(PWM_VALUE=120)
    ok = sim.parse_command("emerg", None, sim.build_checksum("emerg", None))
    assert ok
    assert sim.PWM_VALUE == 0
    assert sim.STATUS_FLAGS == sim.STATUS_EMERG


def test_reset_command_clears_faults_and_restores_ok() -> None:
    sim = ReflexSim(PWM_VALUE=120)
    sim.STATUS_FLAGS = sim.STATUS_STALL | sim.STATUS_WATCHDOG
    ok = sim.parse_command("reset", None, sim.build_checksum("reset", None))
    assert ok
    assert sim.PWM_VALUE == 0
    assert sim.STATUS_FLAGS == sim.STATUS_OK


def test_bad_checksum_command_is_ignored() -> None:
    sim = ReflexSim(DESIRED_TICKS=10)
    ok = sim.parse_command("set_desired", 100, checksum=0x00)
    assert not ok
    assert sim.DESIRED_TICKS == 10
    assert sim.CMD_BUFFER == []
