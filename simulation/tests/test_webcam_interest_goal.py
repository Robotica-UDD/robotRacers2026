"""Presupuesto del acercamiento geométrico, sin cámara ni robot reales."""
from types import SimpleNamespace

import pytest

from simulation.tests.test_webcam_curiosity_navigation import HAND, STAND_OFF_MM
from simulation.tests.test_webcam_pose_navigation import projected_obstacles, synthetic_scene
from simulation.webcam import interest_goal


def test_nearly_spent_budget_never_requests_an_invalid_planner_window(monkeypatch):
    # El planificador admite como mínimo 50 ms. Con solo 40 ms restantes no
    # debe recibir una duración inválida ni consumir otro presupuesto completo.
    ticks = iter([100.])
    clock = SimpleNamespace(monotonic=lambda: next(ticks, 100.76))
    calls = []

    def planner(*args, **kwargs):
        calls.append(kwargs["max_seconds"])
        assert kwargs["max_seconds"] >= .05
        return {}

    barrier = [[(.100, -.060), (.140, -.060), (.140, .060), (.100, .060)]]
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, _, navigation):
        monkeypatch.setattr(interest_goal, "time", clock)
        monkeypatch.setattr(interest_goal, "plan_pose_path", planner)
        with pytest.raises(ValueError, match="acercamiento|punto libre"):
            interest_goal.choose_interest_point(camera.state()["marker_corners"],
                navigation._calibration, HAND, projected_obstacles(barrier), STAND_OFF_MM)
        assert calls == []
