"""Confirmación, planificación y ruedas virtuales detrás de un perímetro."""
from simulation.tests.test_webcam_home_behavior import Clock
from simulation.tests.test_webcam_curiosity_navigation import HAND, SOURCE, use_plant_clock
from simulation.tests.test_webcam_point_navigation import wait_for
from simulation.tests.test_webcam_pose_navigation import pixel, synthetic_scene
from simulation.webcam.home_behavior import HomeBehavior


def hand_result(clock, sequence):
    return {"ready": True, "status": "ready", "model": "MediaPipe de prueba",
            "captured_at": clock(), "sequence": sequence, **SOURCE,
            "hands": [{"bbox_px": list(HAND), "confidence": 1.}]}


def test_hand_confirmation_survives_marker_flicker_and_drives_to_an_accessible_interest_point():
    # La mano está fuera del recinto, pero se puede acercar por dentro sin
    # cruzar la pared. No se conecta ningún transporte ni se abren cámaras.
    vertices = [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)]
    walls = [[pixel(point), pixel(vertices[(index + 1) % len(vertices)])]
             for index, point in enumerate(vertices)]
    with synthetic_scene(start=(0., 0., 0.), polygons=[], plant=True) as (camera, robot, nav):
        nav.set_obstacles(walls, **SOURCE, expected_route_revision=nav.state()["route_revision"])
        nav.set_speeds(240, 90)
        use_plant_clock(nav)
        clock = Clock()
        behavior = HomeBehavior(camera, nav, robot, clock=clock)
        try:
            behavior.set_home(*pixel((-.20, 0.)), **SOURCE)
            behavior.set_enabled(True)
            nav.arm_home_watch()
            for sequence in range(1, 51):
                clock.advance(.05)
                frame = camera.state()
                frame["visible"] = sequence % 2 == 1
                state = behavior.observe(hand_result(clock, sequence), frame)
                if state["curiosity_count"]:
                    break
            assert state["curiosity_count"] == 1, state
            assert state["curious"] and not state["returning"]
            assert state["interest_target_px"] is not None
            assert nav.state()["behavior_purpose"] == "curiosity"
            wait_for(lambda: not nav.state()["active"], timeout=8.)
            assert nav.state()["phase"] == "arrived", nav.state()["message"]
            assert .10 < robot.x < .35
            assert all(-.35 < row["position"][0] < .35 and
                       -.5 < row["position"][1] < .5 for row in robot.trace)
            assert robot.starts == 1
        finally:
            behavior.close()
