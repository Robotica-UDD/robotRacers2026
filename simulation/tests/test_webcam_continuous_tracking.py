"""Pérdidas breves de detección sobre arcos validados; sin acceso al hardware."""
import time
from contextlib import contextmanager

from simulation.tests.test_webcam_point_navigation import wait_for
from simulation.tests.test_webcam_pose_navigation import synthetic_scene, select


REMOTE_OBSTACLE = [[(.4, .3), (.5, .3), (.5, .4), (.4, .4)]]


@contextmanager
def live_frames(camera):
    """Cada lectura entrega un fotograma nuevo de la cámara virtual fija.

    Estas pruebas separan la pérdida del marcador de una captura congelada;
    no dependen de que otro hilo consiga CPU durante una ventana de 100 ms.
    """
    original = camera.state

    def capture():
        camera.update(frame_age_ms=0.)
        return original()

    camera.state = capture
    try:
        yield
    finally:
        camera.state = original


def test_single_missing_marker_frame_does_not_interrupt_a_validated_obstacle_route():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav), live_frames(camera):
        select(nav, (.3, 0.))
        nav.start()
        wait_for(lambda: any(left > 0 and right > 0 for left, right in robot.drives))
        corners = camera.state()["marker_corners"]
        before = len(robot.drives)
        camera.update(visible=False, marker_corners=None)
        # Consultar el indicador sin recalcular toda la vista geométrica;
        # esa consulta puede consumir por sí sola la ventana de una imagen.
        wait_for(lambda: nav._vision_hold)
        time.sleep(.02)
        assert robot.drives[before:]
        assert all(command != (0, 0) for command in robot.drives[before:])
        camera.update(visible=True, marker_corners=corners, frame_age_ms=0.)
        wait_for(lambda: not nav._vision_hold)
        assert all(command != (0, 0) for command in robot.drives[before:])
        assert nav.state()["waypoint_index"] == 0


def test_long_marker_loss_expires_the_validated_arc_and_keeps_the_destination():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav), live_frames(camera):
        before = select(nav, (.3, 0.))
        nav.start()
        wait_for(lambda: any(command != (0, 0) for command in robot.drives))
        camera.update(visible=False, marker_corners=None)
        wait_for(lambda: nav.state()["phase"] == "paused_vision")
        assert robot.drives[-1] == (0, 0)
        state = nav.state()
        assert state["active"] and state["waypoints_px"] == before["waypoints_px"]
        assert state["waypoint_index"] == 0


def test_d48_overrides_the_brief_validated_motion_window():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav), live_frames(camera):
        select(nav, (.3, 0.))
        nav.start()
        wait_for(lambda: any(command != (0, 0) for command in robot.drives))
        camera.update(visible=False, marker_corners=None)
        wait_for(lambda: nav._vision_hold)
        robot.update(stop_reason="button_stop", navigation_active=False, moving=False)
        wait_for(lambda: not nav.state()["active"])
        assert robot.stops >= 1 and not robot.state()["moving"]
        assert nav.state()["last_left"] == nav.state()["last_right"] == 0
        assert "D48" in nav.state()["message"]
        starts = robot.starts
        time.sleep(.02)
        assert robot.starts == starts
