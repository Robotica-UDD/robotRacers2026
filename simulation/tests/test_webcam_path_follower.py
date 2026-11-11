"""Control continuo con una planta diferencial ideal, sin hardware ni red."""
from __future__ import annotations

import math

import pytest

from simulation.webcam.path_follower import PathFollower


SPEEDS = {"forward_mm_s": 300, "turn_deg_s": 400}


def local_path(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)]
            for px, py in points]


def move(x, y, yaw, left, right, dt):
    speed, angular = (left + right) / 2000., (right - left) / 190.
    angle = angular * dt
    if abs(angular) > 1e-9:
        x += speed / angular * (math.sin(yaw + angle) - math.sin(yaw))
        y += speed / angular * (math.cos(yaw) - math.cos(yaw + angle))
    else:
        x += speed * dt * math.cos(yaw)
        y += speed * dt * math.sin(yaw)
    return x, y, math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))


def test_first_straight_sample_moves_and_does_not_wait_for_another_image():
    follower = PathFollower()
    result = follower.update([[0., 0.], [.5, 0.]], 1., SPEEDS)
    assert result["left"] == result["right"] == 300
    assert result["phase"] == "advancing" and not result["blocked"]
    assert not result["done"]


def test_straight_vision_noise_is_filtered_without_stopping_or_large_wheel_corrections():
    follower = PathFollower()
    outputs = []
    for i in range(100):
        deviation = .0007 if i % 2 else -.0007
        outputs.append(follower.update([[0., deviation], [.5, deviation]], 1. + i * .02, SPEEDS))
    assert all(result["left"] > 0 and result["right"] > 0 for result in outputs)
    assert max(abs(result["left"] - result["right"]) for result in outputs) < 10
    assert all(not result["blocked"] for result in outputs)


def test_ninety_degree_corner_is_followed_continuously_without_stopping_at_the_vertex():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4)]
    x = y = yaw = 0.
    outputs, trace = [], []
    previous = (0., 0.)
    for i in range(1000):
        covered = math.hypot(x - .4, y - .4) < .035
        result = follower.update(local_path(route, x, y, yaw), 1. + i * .02, SPEEDS,
                                 goal_covered=covered, previous_wheels=previous)
        outputs.append(result)
        if result["done"]:
            break
        assert not result["blocked"], result
        assert result["left"] + result["right"] > 0, result
        previous = (result["left"], result["right"])
        x, y, yaw = move(x, y, yaw, *previous, .02)
        trace.append((x, y, yaw))
    assert outputs[-1]["done"]
    assert math.hypot(x - .4, y - .4) < .035
    assert any(row[0] > .25 and row[1] > .03 for row in trace)
    assert max(row[0] for row in trace) < .49
    assert outputs[-1]["left"] == outputs[-1]["right"] == 0
    assert max(result["path_index"] for result in outputs) == 2


def test_collinear_subdivision_vertices_do_not_produce_zeros_or_replanning():
    follower = PathFollower()
    route = [(0., 0.), (.15, 0.), (.30, 0.), (.45, 0.), (.60, 0.)]
    indexes = []
    for i in range(45):
        x = i * .012
        result = follower.update(local_path(route, x, 0., 0.), 1. + i * .04, SPEEDS)
        assert result["left"] > 0 and result["right"] > 0
        assert not result["replan"] and not result["done"]
        indexes.append(result["path_index"])
    assert indexes == sorted(indexes)
    assert len(set(indexes)) == 4


def test_progress_does_not_skip_a_future_return_to_the_start_or_move_backwards():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4), (0., .4), (0., 0.)]
    first = follower.update(local_path(route, 0., 0., 0.), 1., SPEEDS)
    assert first["path_index"] == 1
    second = follower.update(local_path(route, .39, .10, math.pi / 2), 1.1, SPEEDS)
    assert second["path_index"] == 2
    third = follower.update(local_path(route, .05, 0., 0.), 1.2, SPEEDS)
    assert third["path_index"] >= 2


def test_repeated_corner_vertices_do_not_leave_the_lookahead_behind_the_robot():
    follower = PathFollower()
    route = [(0., 0.), (.3, 0.), (.3, 0.), (.3, 0.), (.3, .3)]
    result = follower.update(local_path(route, .3, .1, math.pi / 2), 1., SPEEDS)
    assert result["path_index"] == 4
    assert result["target_local_m"][0] > 0
    assert result["left"] > 0 and result["right"] > 0
    assert not result["blocked"]


def test_repeated_initial_vertices_do_not_create_a_false_return_to_the_start():
    follower = PathFollower()
    route = [(0., 0.), (0., 0.), (0., 0.), (.4, 0.)]
    first = follower.update(local_path(route, 0., 0., 0.), 1., SPEEDS)
    assert first["path_index"] == 3
    second = follower.update(local_path(route, .15, 0., 0.), 1.05, SPEEDS)
    assert second["path_index"] == 3
    assert second["target_local_m"][0] > 0
    assert second["left"] == second["right"] > 0


def test_pending_user_point_caps_the_carrot_before_a_future_corner():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4)]
    result = follower.update(local_path(route, .28, .05, 0.), 1., SPEEDS,
                             previous_wheels=(300., 300.), stop_at_index=1)
    assert result["path_index"] == 1
    assert result["target_local_m"] == pytest.approx([.12, -.05])
    assert result["goal_remaining_m"] == pytest.approx(.12)
    assert result["remaining_m"] == pytest.approx(.52)
    assert not result["done"]


def test_expanding_the_pending_user_point_keeps_filter_state_and_motion():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4)]
    first = follower.update(local_path(route, .37, 0., 0.), 1., SPEEDS, stop_at_index=1)
    second = follower.update(local_path(route, .37, 0., 0.), 1.02, SPEEDS,
                             previous_wheels=(first["left"], first["right"]), stop_at_index=2)
    assert second["path_index"] >= first["path_index"]
    assert second["target_local_m"][1] > 0
    assert (second["left"], second["right"]) != (0, 0)
    assert not second["done"] and not second["blocked"]


def test_capped_lookahead_visits_both_user_points_without_a_corner_hold():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4)]
    x = y = yaw = 0.
    previous = (0., 0.)
    pending = 1
    visited = []
    for i in range(1500):
        if math.hypot(x - route[pending][0], y - route[pending][1]) < .035:
            visited.append(pending)
            if pending == 2:
                break
            pending = 2
        result = follower.update(local_path(route, x, y, yaw), 1. + i * .02, SPEEDS,
                                 previous_wheels=previous, stop_at_index=pending)
        assert not result["blocked"] and not result["done"]
        assert (result["left"], result["right"]) != (0, 0)
        previous = (result["left"], result["right"])
        x, y, yaw = move(x, y, yaw, *previous, .02)
    assert visited == [1, 2]


def test_completed_goal_floor_chooses_the_return_leg_when_it_overlaps_the_approach():
    follower = PathFollower()
    route = [(0., 0.), (0., .4), (0., .05)]
    result = follower.update(local_path(route, 0., .33, math.pi / 2), 1., SPEEDS,
                             start_at_index=1, stop_at_index=2)
    assert result["path_index"] == 2
    assert result["target_local_m"][0] < 0
    assert result["goal_remaining_m"] == pytest.approx(.28)
    assert result["remaining_m"] == pytest.approx(.28)
    assert result["phase"] == "turning"


def test_out_and_back_body_coverage_does_not_create_repeated_revolutions():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.05, 0.)]
    x = y = yaw = 0.
    previous = (0., 0.)
    pending = 1
    visited = []
    rotation = 0.
    for i in range(1500):
        if math.hypot(x - route[pending][0], y - route[pending][1]) < .075:
            visited.append(pending)
            if pending == 2:
                break
            pending = 2
        result = follower.update(local_path(route, x, y, yaw), 1. + i * .02,
                                 {"forward_mm_s": 200, "turn_deg_s": 100},
                                 previous_wheels=previous, start_at_index=pending - 1,
                                 stop_at_index=pending)
        assert not result["blocked"] and not result["done"]
        assert (result["left"], result["right"]) != (0, 0)
        assert result["left"] + result["right"] >= 0
        previous = (result["left"], result["right"])
        rotation += abs((previous[1] - previous[0]) / 190. * .02)
        x, y, yaw = move(x, y, yaw, *previous, .02)
    assert visited == [1, 2]
    assert rotation < 2. * math.pi


@pytest.mark.parametrize("start", [-1, 3, True, 1.5, "1"])
def test_invalid_completed_point_floor_never_generates_motion(start):
    result = PathFollower().update([[0., 0.], [.4, 0.], [.4, .4]], 1., SPEEDS,
                                   start_at_index=start, stop_at_index=1)
    assert result["blocked"] and result["left"] == result["right"] == 0


@pytest.mark.parametrize("bound", [-1, 3, True, 1.5, "1"])
def test_invalid_pending_point_index_never_generates_motion(bound):
    result = PathFollower().update([[0., 0.], [.4, 0.], [.4, .4]], 1., SPEEDS,
                                   stop_at_index=bound)
    assert result["blocked"] and result["left"] == result["right"] == 0


def test_collision_predictions_choose_a_slower_safe_motion_without_a_pause():
    calls = []

    def clear(left, right, horizon):
        calls.append((left, right, horizon))
        return 0 < left <= 100 and 0 < right <= 100

    result = PathFollower().update([[0., 0.], [.5, 0.]], 1., SPEEDS, is_clear=clear)
    assert 0 < result["left"] <= 100 and 0 < result["right"] <= 100
    assert not result["blocked"] and not result["replan"]
    assert len(calls) <= 12
    assert all(0 < call[2] <= .5 for call in calls)


def test_when_advancing_arcs_do_not_fit_it_turns_without_an_idle_confirmation():
    result = PathFollower().update([[0., 0.], [.5, 0.]], 1., SPEEDS,
        is_clear=lambda left, right, _: left < 0 and right > 0)
    assert result["left"] < 0 < result["right"]
    assert result["phase"] == "turning"
    assert not result["blocked"]


def test_a_goal_behind_the_robot_uses_a_controlled_pivot_not_reverse():
    result = PathFollower().update([[0., 0.], [-.4, 0.]], 1., SPEEDS)
    assert result["left"] * result["right"] < 0
    assert result["left"] + result["right"] == 0
    assert result["phase"] == "turning"


def test_subpixel_noise_behind_the_robot_does_not_reverse_the_half_turn_each_frame():
    follower = PathFollower()
    previous = (0., 0.)
    for i in range(50):
        y = .0007 if i % 2 == 0 else -.0007
        result = follower.update([[0., 0.], [-.4, y]], 1. + i * .02,
                                 SPEEDS, previous_wheels=previous)
        assert result["left"] < 0 < result["right"]
        assert result["phase"] == "turning" and not result["blocked"]
        previous = (result["left"], result["right"])


def test_low_turn_limit_reduces_forward_speed_so_a_sharp_curve_is_not_understeered():
    result = PathFollower().update([[0., 0.], [.1, .1]], 1.,
                                   {"forward_mm_s": 600, "turn_deg_s": 12})
    assert 0 < result["linear_mm_s"] <= 20.
    assert abs(result["angular_deg_s"]) <= 12.1
    assert result["left"] + result["right"] > 0


def test_a_tight_fifty_mm_forward_arc_can_reverse_the_inner_wheel():
    follower = PathFollower()
    follower.LOOKAHEAD_MIN_M = follower.LOOKAHEAD_MAX_M = .070
    previous = (0., 0.)
    for i in range(30):
        result = follower.update([[0., 0.], [.1, .1]], 1. + i * .05,
                                 SPEEDS, previous_wheels=previous)
        assert result["left"] + result["right"] > 0
        previous = (result["left"], result["right"])
    angular = math.radians(result["angular_deg_s"])
    radius_m = result["linear_mm_s"] / 1000. / angular
    assert result["left"] < 0 < result["right"]
    assert radius_m == pytest.approx(.050, abs=.004)
    assert max(abs(result["left"]), abs(result["right"])) <= SPEEDS["forward_mm_s"]


def test_a_free_clockwise_half_turn_is_used_and_latched_when_counterclockwise_is_blocked():
    follower = PathFollower()
    previous = (0., 0.)
    for i in range(20):
        y = .0007 if i % 2 == 0 else -.0007
        result = follower.update([[0., 0.], [-.4, y]], 1. + i * .02, SPEEDS,
            previous_wheels=previous, is_clear=lambda left, right, _: left > 0 > right)
        assert result["left"] > 0 > result["right"]
        assert result["phase"] == "turning" and not result["blocked"]
        previous = (result["left"], result["right"])


def test_residual_opposite_angular_velocity_does_not_choose_the_long_half_turn():
    follower = PathFollower()
    previous = (-190., 190.)  # Todavía gira a la izquierda antes de frenar.
    route = [[0., 0.], [-.4, -.05]]  # El nuevo giro corto va a la derecha.
    angular_rates = []
    for i in range(45):
        result = follower.update(route, 1. + i * .02, SPEEDS, previous_wheels=previous)
        assert not result["blocked"]
        assert follower._pivot_sign == -1.
        angular_rates.append(result["angular_deg_s"])
        previous = (result["left"], result["right"])
    assert angular_rates[0] > 0 > angular_rates[-1]
    assert angular_rates == sorted(angular_rates, reverse=True)


def test_opposite_angular_slew_brakes_before_moving_toward_a_new_forward_curve():
    follower = PathFollower()
    result = follower.update([[0., 0.], [.4, .1]], 1., SPEEDS,
                             previous_wheels=(190., -190.))
    assert result["left"] > 0 > result["right"]  # Desacelera el giro anterior.
    assert result["linear_mm_s"] == 0.
    assert -math.degrees(2.) < result["angular_deg_s"] < 0.
    assert not result["blocked"]


def test_all_rejected_predictions_request_a_replan_without_driving_into_the_obstacle():
    calls = []

    def occupied(left, right, horizon):
        calls.append((left, right))
        return False

    result = PathFollower().update([[0., 0.], [.4, .1]], 1., SPEEDS, is_clear=occupied)
    assert result["blocked"] and result["replan"]
    assert result["left"] == result["right"] == 0
    assert 1 <= len(calls) <= 12
    assert len(calls) == len(set(calls))


def test_a_safe_counter_goal_pivot_cannot_trap_the_controller_in_an_opposite_turn_loop():
    result = PathFollower().update([[0., 0.], [.4, .2]], 1., SPEEDS,
        is_clear=lambda left, right, _: left > 0 and right < 0)
    # El objetivo está a la izquierda, pero solo cabe comenzar un giro hacia
    # la derecha. La salida debe calcularla el plan global desde esta postura.
    assert result["blocked"] and result["replan"]
    assert result["left"] == result["right"] == 0


def test_only_final_goal_coverage_requests_a_stop_during_a_long_route():
    follower = PathFollower()
    path = [[0., 0.], [.05, 0.], [.5, 0.]]
    moving = follower.update(path, 1., SPEEDS)
    assert moving["left"] > 0 and not moving["done"]
    stopped = follower.update(path, 1.02, SPEEDS, goal_covered=True)
    assert stopped["done"] and not stopped["blocked"]
    assert stopped["left"] == stopped["right"] == 0


@pytest.mark.parametrize("path", [[], None, [[0., float("nan")]], [[float("inf"), 0.]],
                                  [[True, 0.]], [["1", 0.]], [[0.]], [0.]])
def test_invalid_route_requests_zero_and_replanning(path):
    result = PathFollower().update(path, 1., SPEEDS)
    assert result["left"] == result["right"] == 0
    assert result["blocked"]


@pytest.mark.parametrize("stamp", [None, float("nan"), float("inf"), -1., True])
def test_invalid_time_never_generates_motion(stamp):
    result = PathFollower().update([[0., 0.], [.5, 0.]], stamp, SPEEDS)
    assert result["blocked"] and result["left"] == result["right"] == 0


def test_reset_allows_a_new_route_after_previous_progress():
    follower = PathFollower()
    route = [(0., 0.), (.4, 0.), (.4, .4)]
    follower.update(local_path(route, .39, .2, math.pi / 2), 1., SPEEDS)
    assert follower._segment == 1
    follower.reset()
    result = follower.update(local_path(route, 0., 0., 0.), 2., SPEEDS)
    assert result["path_index"] == 1


def test_angular_acceleration_is_bounded_for_ordinary_tracking():
    follower = PathFollower()
    previous = (300, 300)
    angular = 0.
    for i in range(20):
        result = follower.update([[0., 0.], [.4, .15]], 1. + i * .02, SPEEDS,
                                 previous_wheels=previous)
        current = math.radians(result["angular_deg_s"])
        assert abs(current - angular) <= follower.ANGULAR_ACCELERATION_RAD_S2 * (.05 if i == 0 else .02) + .006
        previous = (result["left"], result["right"])
        angular = current
