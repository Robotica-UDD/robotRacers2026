"""Ruta visible persistente y origen sincronizado con el vídeo, sin hardware."""
import threading

import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, calibrate, wait_for
from simulation.tests.test_webcam_pose_navigation import synthetic_scene, select
from simulation.tests.test_webcam_continuous_tracking import live_frames, REMOTE_OBSTACLE
from simulation.webcam.detector import Detection, annotate_frame
from simulation.webcam.navigation import Navigation
from simulation.webcam.navigation_geometry import Calibration, observe_robot


def test_last_route_stays_visible_while_an_alternative_is_being_calculated():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, _, nav):
        before = select(nav, (.3, 0.))
        started, release = threading.Event(), threading.Event()
        calculate = nav._calculate_plan

        def delayed(*args, **kwargs):
            started.set()
            assert release.wait(2.)
            return calculate(*args, **kwargs)

        nav._calculate_plan = delayed
        worker = threading.Thread(target=lambda: nav._refresh_plan(force=True, runtime=True))
        worker.start()
        try:
            assert started.wait(1.)
            camera.pose(x=655., y=480., scale=800.)
            state = nav.state()
            assert state['planning_status'] == 'waiting'
            assert state['display_path_px'][1:] == before['display_path_px'][1:]
            assert state['display_path_px'][0] == pytest.approx(state['center_px'])
        finally:
            release.set()
            worker.join(2.)


def test_missing_marker_keeps_the_route_but_not_a_false_current_pose():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav):
        before = select(nav, (.3, 0.))
        camera.update(visible=False, marker_corners=None)
        state = nav.state()
        assert state['display_path_px'] == before['display_path_px']
        assert state['planned_path_px'] == [] and state['center_px'] is None
        assert robot.starts == 0 and robot.drives == []
        nav.clear_targets()
        assert nav.state()['display_path_px'] == []


def test_an_actual_camera_change_never_reuses_the_previous_visual_route():
    camera, robot = FakeCamera(), FakeRobot()
    nav = Navigation(camera, robot)
    try:
        calibrate(nav)
        nav.select_target(1000., 480., 1280, 960, 'usb-camera')
        assert nav.state()['display_path_px']
        camera.update(camera_id='other-camera')
        assert nav.state()['display_path_px'] == []
    finally:
        nav.close()


def test_vision_recovery_reprojects_the_existing_route_without_global_replanning():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav), live_frames(camera):
        select(nav, (.3, 0.))
        calls = []
        original = nav._runtime_plan

        def record(*args):
            calls.append(True)
            return original(*args)

        nav._runtime_plan = record
        nav.start()
        wait_for(lambda: nav._phase == 'advancing' and any(sum(pair) > 0 for pair in robot.drives))
        stamp = nav._plan_attempted_at
        corners = camera.state()['marker_corners']
        camera.update(visible=False, marker_corners=None)
        wait_for(lambda: nav._phase == 'paused_vision')
        assert nav.state()['display_path_px']
        assert robot.drives[-1] == (0, 0)
        camera.update(visible=True, marker_corners=corners)
        wait_for(lambda: nav._phase == 'advancing' and sum(robot.drives[-1]) > 0)
        assert not calls and nav._plan_attempted_at == stamp
        assert nav.state()['planning_status'] == 'ready'


def test_video_route_uses_the_calibrated_center_of_the_same_detection(monkeypatch):
    camera, robot = FakeCamera(), FakeRobot()
    nav = Navigation(camera, robot)
    try:
        nav._calibration = Calibration((.075, -.04), .4, 'usb-camera', 1280, 960)
        nav._display_path = [[10., 10.], [1000., 300.]]
        old_center = nav.state()['center_px']
        # Nueva imagen adelantada 100 px sin actualizar ninguna respuesta API.
        newer = FakeCamera()
        newer.pose(x=740., y=480.)
        corners = newer.state()['marker_corners']
        detection = Detection(True, (740., 480.), 0., corners)
        pose = observe_robot(corners, nav._calibration)
        overlay = nav.overlay_route(detection, pose)
        assert overlay['path_px'][0] == pytest.approx(pose['center_px'])
        assert overlay['path_px'][0][0] - old_center[0] == pytest.approx(100.)
        lines, crosses = [], []
        monkeypatch.setattr('cv2.polylines', lambda image, points, *args, **kwargs: lines.append(points) or image)
        monkeypatch.setattr('cv2.drawMarker', lambda image, center, *args, **kwargs: crosses.append(center) or image)
        image = np.zeros((960, 1280, 3), np.uint8)
        annotate_frame(image, detection, pose, navigation_overlay=overlay)
        assert lines[0][0][0].tolist() == np.rint(pose['center_px']).astype(int).tolist()
        assert crosses == [tuple(np.rint(pose['center_px']).astype(int))]
        assert crosses[0] != (740, 480)
        # Sin detección se mantiene solo el camino, sin inventar centro ni flecha.
        lost = nav.overlay_route(Detection(False))
        assert lost['path_px'] and not lost['footprint_px']
        assert lost['steering_target_px'] is None
        arrows = []
        monkeypatch.setattr('cv2.arrowedLine', lambda image, *args, **kwargs: arrows.append(args) or image)
        annotated = annotate_frame(image, Detection(False), navigation_overlay=lost)
        assert not arrows and annotated.shape == image.shape
        # Una calibración cuya pose no se puede usar tampoco debe volver a
        # mostrar una dirección desde el marcador desplazado.
        annotate_frame(image, detection, navigation_overlay=lost)
        assert not arrows
    finally:
        nav.close()
