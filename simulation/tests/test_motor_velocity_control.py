"""Compila el núcleo real del motor y lo prueba sin Arduino, USB ni red.

La planta de fricción comprueba el comportamiento del control y su transición
de arranque; no representa una calibración física del robot.
"""
from pathlib import Path
import shutil
import subprocess

import pytest


SCENARIOS = (
    "reset_and_idle",
    "startup_ramp",
    "two_counts_and_smooth_transfer",
    "wrong_encoder_direction_does_not_learn",
    "zero_cancels_and_does_not_restart",
    "stall_is_latched",
    "no_repeated_startup_and_running_stall",
    "reversal_and_target_flapping",
    "zero_then_reverse_waits_for_rest",
    "pi_limits_and_antiwindup",
    "quantized_friction_plant",
    "velocity_300_control_stop_and_reverse",
)


@pytest.fixture(scope="module")
def motor_velocity_binary(tmp_path_factory):
    compiler = shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.fail("Se necesita un compilador C++ para comprobar el control de motores.")
    source = Path(__file__).with_suffix(".cpp")
    binary = tmp_path_factory.mktemp("motor-velocity") / "motor-velocity-test"
    subprocess.run(
        [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
         str(source), "-o", str(binary)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    return binary


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_motor_velocity_native(motor_velocity_binary, scenario):
    result = subprocess.run(
        [str(motor_velocity_binary), scenario], capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
