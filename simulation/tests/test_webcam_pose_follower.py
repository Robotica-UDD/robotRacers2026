"""Maniobras puras: sin cámara, transporte, firmware ni motores reales."""
import math

import pytest

from simulation.webcam.pose_follower import PoseFollower


SPEEDS = {"forward_mm_s": 200, "turn_deg_s": 400}


def step(kind="drive", direction=1, **extra):
    return {"kind": kind, "direction": direction, "target_px": [100., 100.],
            "heading_rad": 0., **extra}


def observation(distance=.2, bearing=0., **extra):
    return {"distance_m": distance, "bearing_rad": bearing, **extra}


@pytest.mark.parametrize("error", [-.5, .5])
def test_turn_uses_physical_heading_error_even_when_the_center_is_already_at_target(error):
    result = PoseFollower().update(step("turn", heading_rad=2.), observation(0., 0.), error, 1., SPEEDS)
    assert result["phase"] == "turning" and not result["done"] and not result["replan"]
    assert result["left"] * error < 0 and result["right"] * error > 0


def test_turn_requires_a_continuous_tenth_second_inside_three_degrees():
    follower = PoseFollower()
    for stamp in (1., 1.05, 1.099):
        result = follower.update(step("turn"), observation(0.), math.radians(2.), stamp, SPEEDS)
        assert result["left"] == result["right"] == 0 and not result["done"]
    assert follower.update(step("turn"), observation(0.), 0., 1.1, SPEEDS)["done"]


def test_turn_hold_restarts_after_heading_leaves_tolerance_or_step_changes():
    follower = PoseFollower()
    follower.update(step("turn"), observation(), 0., 1., SPEEDS)
    assert not follower.update(step("turn"), observation(), math.radians(4.), 1.09, SPEEDS)["done"]
    assert not follower.update(step("turn"), observation(), 0., 1.10, SPEEDS)["done"]
    assert not follower.update(step("turn", heading_rad=1.), observation(), 0., 1.2, SPEEDS)["done"]
    assert follower.update(step("turn", heading_rad=1.), observation(), 0., 1.3, SPEEDS)["done"]


@pytest.mark.parametrize("flag", ["escape", "escaping"])
def test_escape_turn_is_capped_at_thirty_degrees_per_second(flag):
    result = PoseFollower().update(step("turn", **{flag: True}), observation(), math.pi / 2, 1., SPEEDS)
    assert 0 < result["right"] <= math.ceil(95 * math.radians(30))
    assert result["left"] == -result["right"]


@pytest.mark.parametrize("direction,bearing", [(1, .1), (-1, -math.pi + .1)])
def test_forward_and_reverse_keep_the_same_physical_angular_sign(direction, bearing):
    follower = PoseFollower()
    for index in range(12):
        result = follower.update(step(direction=direction), observation(bearing=bearing, sequence=index),
                                 None, 1. + index * .05, SPEEDS)
    assert result["left"] * direction > 0 and result["right"] * direction > 0
    assert result["right"] - result["left"] > 0
    assert not result["done"] and not result["replan"]


def test_reverse_is_negative_swapped_forward_instead_of_negating_both_wheels():
    forward, reverse = PoseFollower(), PoseFollower()
    for index in range(12):
        first = forward.update(step(), observation(bearing=.12, sequence=index), None, 1. + index * .05, SPEEDS)
        second = reverse.update(step(direction=-1), observation(bearing=-math.pi + .12, sequence=index),
                                None, 1. + index * .05, SPEEDS)
    assert (second["left"], second["right"]) == (-first["right"], -first["left"])


@pytest.mark.parametrize("kind,direction,bearing", [("drive", 1, .5), ("drive", -1, 0.), ("escape", -1, .5)])
def test_translation_requests_replan_instead_of_improvising_an_unvalidated_turn(kind, direction, bearing):
    result = PoseFollower().update(step(kind, direction), observation(bearing=bearing), None, 1., SPEEDS)
    assert result["replan"] and not result["done"]
    assert result["left"] == result["right"] == 0


@pytest.mark.parametrize("direction", [-1, 1])
def test_arrival_at_eight_mm_requests_zero_without_falsely_rotating(direction):
    result = PoseFollower().update(step(direction=direction), observation(.008, .8), None, 1., SPEEDS)
    assert result["done"] and not result["replan"]
    assert result["left"] == result["right"] == 0


def test_translation_brakes_progressively_and_retains_an_internal_twenty_mm_s_floor():
    commands = []
    for distance in (.3, .05, .02, .009, .0081):
        result = PoseFollower().update(step(), observation(distance), None, 1., SPEEDS)
        commands.append(result["left"])
        assert result["left"] == result["right"] and not result["done"]
    assert commands == sorted(commands, reverse=True)
    assert commands[0] == 200 and commands[-1] == 20


def test_existing_small_local_speed_cap_is_not_raised_to_the_internal_floor():
    result = PoseFollower().update(step(), observation(.009), None, 1., {**SPEEDS, "forward_mm_s": 7})
    assert result["left"] == result["right"] == 7


@pytest.mark.parametrize("direction,bearing", [(1, .1), (-1, -math.pi + .1)])
def test_escape_is_cautious_and_keeps_both_wheels_in_the_planned_direction(direction, bearing):
    follower = PoseFollower()
    for index in range(12):
        result = follower.update(step("escape", direction), observation(bearing=bearing, sequence=index),
                                 None, 1. + index * .05, SPEEDS)
    assert 0 < abs(result["left"]) <= 80 and 0 < abs(result["right"]) <= 80
    assert result["left"] * direction > 0 and result["right"] * direction > 0
    assert abs(math.degrees((result["right"] - result["left"]) / 190.)) <= 30


def test_repeated_camera_sequence_does_not_integrate_an_old_bearing():
    follower = PoseFollower()
    follower.update(step(), observation(bearing=.1, sequence=1), None, 1., SPEEDS)
    expected = follower.update(step(), observation(bearing=.1, sequence=2), None, 1.05, SPEEDS)
    repeated = follower.update(step(), observation(bearing=.2, sequence=2), None, 1.2, SPEEDS)
    assert (repeated["left"], repeated["right"]) == (expected["left"], expected["right"])


@pytest.mark.parametrize("stamp", [None, True, float("nan"), float("inf"), -1.])
def test_invalid_time_never_issues_motion(stamp):
    result = PoseFollower().update(step(), observation(), None, stamp, SPEEDS)
    assert result["replan"] and result["left"] == result["right"] == 0


def test_backwards_time_and_reset_discard_a_previous_turn_hold():
    follower = PoseFollower()
    follower.update(step("turn"), observation(), 0., 2., SPEEDS)
    assert follower.update(step("turn"), observation(), 0., 1., SPEEDS)["replan"]
    assert not follower.update(step("turn"), observation(), 0., 3., SPEEDS)["done"]
    follower.reset()
    assert not follower.update(step("turn"), observation(), 0., 4., SPEEDS)["done"]


@pytest.mark.parametrize("distance,bearing", [(float("nan"), 0.), (.2, float("inf")), (-.1, 0.), (True, 0.)])
def test_invalid_pose_requests_replan_at_zero(distance, bearing):
    result = PoseFollower().update(step(), observation(distance, bearing), None, 1., SPEEDS)
    assert result["replan"] and result["left"] == result["right"] == 0


def test_direction_changes_leave_the_zero_crossing_to_the_existing_host_ramp():
    from simulation.webcam.navigation import Navigation

    follower = PoseFollower()
    first = follower.update(step(), observation(), None, 1., SPEEDS)
    second = follower.update(step(direction=-1), observation(bearing=math.pi), None, 1.05, SPEEDS)
    assert first["left"] > 0 and second["left"] < 0
    # El seguidor no evade ni duplica la rampa de Navigation: al componer ambos,
    # la primera inversión frena hasta cero y solo otro ciclo cambia el signo.
    zero = Navigation._ramp_value(first["left"], second["left"], 1.)
    assert zero == 0
    assert Navigation._ramp_value(zero, second["left"], .05) < 0
