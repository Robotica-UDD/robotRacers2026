"""Rumbo con señales sintéticas; no importa ni acciona cámaras o motores."""
from __future__ import annotations

import math

import pytest

from simulation.webcam.straight_control import StraightControl


def signal(controller, degrees, *, speed=100., distance=.5, start=0., interval=.05):
    return [controller.update(math.radians(angle), start + index * interval,
                              index, speed, distance) for index, angle in enumerate(degrees)]


def test_deadband_and_alternating_visual_noise_do_not_make_robot_zigzag():
    controller = StraightControl()
    assert signal(controller, [.8, -.8] * 100) == [0.] * 200
    controller.reset()
    rates = signal(controller, [3., -3.] * 100)
    assert max(abs(value) for value in rates) < math.radians(1.)
    assert max(abs(value) for value in rates[20:]) < math.radians(.05)


@pytest.mark.parametrize("bearing", [-12., -8., 8., 12.])
def test_constant_curve_response_has_correct_sign_and_is_gentler_than_old_gain(bearing):
    controller = StraightControl()
    rates = signal(controller, [bearing] * 40)
    assert rates[0] == 0.
    assert all(value * bearing >= 0 for value in rates)
    assert 0 < abs(rates[-1]) <= math.radians(8.)
    assert abs(rates[-1]) < abs(math.radians(bearing) * 1.5)
    assert abs(rates[-1]) > math.radians(1.)


def test_pd_derivative_damps_a_decreasing_error_instead_of_amplifying_it():
    damped, proportional_only = StraightControl(), StraightControl()
    proportional_only.KD = 0.
    inputs = [10.] * 30 + [6., 5., 4., 3.] + [3.] * 15
    damped_rates = signal(damped, inputs)
    plain_rates = signal(proportional_only, inputs)
    assert damped_rates[29] == pytest.approx(plain_rates[29], abs=1e-8)
    assert any(damped_rates[index] < plain_rates[index] - math.radians(.05)
               for index in range(30, 40))
    assert all(value >= 0 for value in damped_rates)


def test_repeated_frame_never_refilters_or_accelerates_output():
    controller = StraightControl()
    controller.update(.2, 1., 10, 100., .5)
    previous = controller.update(.2, 1.05, 11, 100., .5)
    assert previous > 0
    for stamp in (1.06, 1.10, 1.15, 2.):
        assert controller.update(-.4, stamp, 11, 100., .5) == previous
    assert controller._stamp == 1.05


def test_angular_output_slew_is_bounded_through_direction_changes():
    controller = StraightControl()
    rates = signal(controller, [15.] * 30 + [-15.] * 60 + [0.] * 60)
    for previous, current in zip(rates, rates[1:]):
        assert abs(current - previous) <= math.radians(25.) * .05 + 1e-12
    assert max(rates) <= math.radians(8.)
    assert min(rates) >= -math.radians(8.)
    assert abs(rates[-1]) < math.radians(.05)


@pytest.mark.parametrize("speed", [1., 5., 20., 50., 100., 1000.])
@pytest.mark.parametrize("bearing", [-20., 20.])
def test_forward_wheels_remain_positive_and_differential_correction_is_moderate(speed, bearing):
    rates = signal(StraightControl(), [bearing] * 50, speed=speed)
    for rate in rates:
        correction = 95. * rate
        assert abs(correction) <= speed * .25 + 1e-10
        assert speed - correction >= speed * .75 - 1e-10
        assert speed + correction >= speed * .75 - 1e-10


def test_approach_and_speed_drop_reduce_curvature_even_for_a_repeated_frame():
    controller = StraightControl()
    steady = signal(controller, [15.] * 30)[-1]
    near = controller.update(.3, 1.5, 29, 100., .025)
    slow = controller.update(.3, 1.5, 29, 1., .025)
    assert 0 < slow < near < steady
    assert abs(95 * slow) <= .25 + 1e-10
    assert controller.update(.3, 1.55, 30, 0., .025) == 0.


@pytest.mark.parametrize("field,value", [
    ("bearing", float("nan")), ("bearing", float("inf")), ("bearing", True),
    ("stamp", float("nan")), ("stamp", -1.), ("stamp", True),
    ("sequence", -1), ("sequence", True), ("sequence", None),
    ("speed", -1.), ("speed", float("inf")), ("speed", "100"),
    ("distance", -.1), ("distance", float("nan")), ("distance", False),
])
def test_invalid_sample_resets_to_safe_zero_without_nan_or_derivative_spike(field, value):
    controller = StraightControl()
    assert signal(controller, [10.] * 10)[-1] > 0
    values = {"bearing": .2, "stamp": .5, "sequence": 10, "speed": 100., "distance": .5}
    values[field] = value
    assert controller.update(*values.values()) == 0.
    assert controller.update(.2, 1., 11, 100., .5) == 0.


@pytest.mark.parametrize("stamp,sequence", [(1., 2), (.9, 2), (2., 2), (1.1, 0)])
def test_nonmonotonic_time_or_new_capture_session_resets_controller(stamp, sequence):
    controller = StraightControl()
    controller.update(.3, 1., 1, 100., .5)
    assert controller.update(.3, stamp, sequence, 100., .5) == 0.
    assert controller._rate == 0.


def test_circular_bearing_wrap_is_equivalent_and_has_no_full_revolution_derivative_jump():
    direct, equivalent = StraightControl(), StraightControl()
    previous = 0.
    for index, degrees in enumerate([175., 178., 179., 181., 183., 185., 187.]):
        rate = direct.update(math.radians(degrees), index * .05, index, 100., .5)
        other = equivalent.update(math.radians(degrees - 360), index * .05, index, 100., .5)
        assert rate == pytest.approx(other, abs=1e-12)
        assert abs(rate - previous) <= math.radians(25.) * .05 + 1e-12
        assert abs(direct._derivative) < math.radians(90.)
        previous = rate


def test_reset_drops_previous_goal_and_starts_new_target_without_a_step():
    controller = StraightControl()
    assert signal(controller, [15.] * 20)[-1] > 0
    controller.reset()
    assert controller.update(math.radians(-15.), 5., 100, 100., .5) == 0.
    next_rate = controller.update(math.radians(-15.), 5.05, 101, 100., .5)
    assert -math.radians(1.25) - 1e-12 <= next_rate < 0
