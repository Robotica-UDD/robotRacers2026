"""Pruebas de navegación con dobles: no abren cámaras, USB ni red del robot."""
from __future__ import annotations

import copy
import json
import math
import threading
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from simulation.webcam.navigation import Navigation, NavigationError, make_navigation_router
from simulation.webcam.detector import Detection
from simulation.webcam.navigation_geometry import observe_robot


class FakeCamera:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {"visible": True, "frame_age_ms": 0., "sequence": 1,
                      "frame_width": 1280, "frame_height": 960, "camera_id": "usb-camera"}
        self.pose()

    def pose(self, x=640., y=480., heading=0., scale=2000.):
        c, s = math.cos(heading), math.sin(heading)
        corners = [[x + scale * (c * tx - s * ty), y - scale * (s * tx + c * ty)]
                   for tx, ty in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
        self.update(marker_corners=corners)

    def update(self, **values):
        with self.lock:
            self.value.update(values)
            self.value["sequence"] += 1

    def state(self):
        with self.lock:
            return copy.deepcopy(self.value)


class FakeRobot:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {"connected": True, "ready": True, "moving": False,
                      "navigation_active": False, "control_session": None,
                      "stop_reason": "idle", "boot_id": "boot-1",
                      "velocity_control": True, "encoder_ready": True, "max_velocity_mm_s": 100}
        self.starts = 0
        self.drives = []
        self.stops = 0
        self.abandons = 0
        self.stale_start = False
        self.fail_drive = False
        self.reject_resume = False
        self.resume_calls = []
        self.velocity_requirements = []
        self.required_caps = []

    def state(self):
        with self.lock:
            return dict(self.value)

    def update(self, **values):
        with self.lock:
            self.value.update(values)

    def start_navigation(self, *, resume=False, require_velocity=False, required_velocity_mm_s=100):
        with self.lock:
            self.resume_calls.append(resume)
            self.velocity_requirements.append(require_velocity)
            self.required_caps.append(required_velocity_mm_s)
            if required_velocity_mm_s > self.value.get("max_velocity_mm_s", 100):
                raise NavigationError("Actualiza el firmware para usar esta velocidad.")
            if resume and self.reject_resume:
                raise NavigationError("D48 detectado en el estado fresco antes de armar.", 423)
            self.starts += 1
            session = f"session-{self.starts}"
            if not self.stale_start:
                self.value.update(navigation_active=True, control_session=session, stop_reason="navigating")
            return {"session": session}

    def drive_navigation(self, left, right):
        raise AssertionError("La navegación no puede utilizar órdenes de PWM abierto.")

    def drive_velocity_navigation(self, left, right):
        with self.lock:
            if self.fail_drive:
                raise RuntimeError("Simulated unavailable transport")
            self.drives.append((left, right))
            self.value["moving"] = left != 0 or right != 0

    def stop_navigation(self):
        with self.lock:
            self.stops += 1
            self.value.update(navigation_active=False, moving=False, stop_reason="remote")

    def abandon_navigation(self):
        with self.lock:
            self.abandons += 1
            self.value.update(navigation_active=False, control_session=None, moving=False,
                              stop_reason="navigation_stop")


class ClosedLoopRobot(FakeRobot):
    """Planta diferencial virtual: cada mando produce una imagen nueva.

    Integra ruedas ideales separadas 190 mm a pasos simulados de 20 ms. Las
    coordenadas físicas X/Y son derecha/arriba; la cámara las proyecta en píxeles.
    Este modelo comprueba realimentación y signos, no valida la dinámica real.
    """
    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.x = self.y = self.yaw = 0.
        self.trace = []

    def drive_velocity_navigation(self, left, right):
        super().drive_velocity_navigation(left, right)
        dt, wheelbase = .02, .19
        left_mps, right_mps = left / 1000, right / 1000
        speed = (left_mps + right_mps) / 2
        angular_speed = (right_mps - left_mps) / wheelbase
        delta_angle = angular_speed * dt
        # Integración exacta del arco para consignas constantes durante el paso.
        if abs(angular_speed) > 1e-9:
            self.x += speed / angular_speed * (math.sin(self.yaw + delta_angle) - math.sin(self.yaw))
            self.y += speed / angular_speed * (math.cos(self.yaw) - math.cos(self.yaw + delta_angle))
        else:
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt
        self.yaw = (self.yaw + delta_angle + math.pi) % (2 * math.pi) - math.pi
        self.trace.append((self.x, self.y, self.yaw, left, right))
        self.camera.pose(x=640 + self.x * 2000, y=480 - self.y * 2000, heading=self.yaw)


def wait_for(predicate, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    assert predicate(), "No se completó la condición del controlador simulado."


def calibrate(nav):
    return nav.calibrate([640., 480.], [740., 480.], 1280, 960, "usb-camera")


def target(nav, x=900., y=480.):
    return nav.select_target(x, y, 1280, 960, "usb-camera")


def stable_vision(camera, nav, **pose):
    camera.update(visible=True, frame_age_ms=0.)
    for _ in range(nav.RESUME_FRAMES + 1):
        camera.pose(**pose)
        time.sleep(max(nav.CONTROL_INTERVAL * 1.5, nav.RESUME_STABLE_SECONDS / nav.RESUME_FRAMES))


@pytest.fixture
def rig():
    camera, robot = FakeCamera(), FakeRobot()
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .01
    nav.SETTLE_SECONDS = .04
    nav.RESUME_STABLE_SECONDS = .03
    nav.RESUME_RETRY_SECONDS = .03
    try:
        yield camera, robot, nav
    finally:
        nav.close()


def test_calibration_selection_and_get_do_not_send_commands(rig):
    camera, robot, nav = rig
    assert nav.state()["calibrated"] is False
    assert calibrate(nav)["calibrated"] is True
    result = target(nav)
    assert result["phase"] == "selected"
    assert result["active"] is False
    assert result["target_px"] == [900., 480.]
    assert result["remaining_mm"] == pytest.approx(130., abs=.01)
    assert result["center_px"] == pytest.approx([640., 480.], abs=.01)
    assert result["front_px"] == pytest.approx([680., 480.], abs=.01)
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0
    camera.pose(x=660.)
    assert nav.state()["center_px"] == pytest.approx([660., 480.], abs=.01)
    assert robot.drives == []


def test_start_moves_then_settles_and_stops_at_selected_point(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    assert nav.start()["active"] is True
    wait_for(lambda: bool(robot.drives))
    wait_for(lambda: any(left > 0 and right > 0 for left, right in robot.drives))
    assert all(0 <= left <= 100 and 0 <= right <= 100 for left, right in robot.drives)
    camera.pose(x=900.)
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["phase"] == "arrived"
    assert (0, 0) in robot.drives
    assert robot.stops == 1
    count = len(robot.drives)
    time.sleep(.04)
    assert len(robot.drives) == count and robot.starts == 1


def test_turn_sign_and_small_forward_correction_are_positive_left(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav, 840, 280)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    wait_for(lambda: any(left < 0 and right > 0 for left, right in robot.drives))
    left, right = next(command for command in robot.drives if command != (0, 0))
    assert -53 <= left < 0 and 0 < right <= 53
    # Una vez orientado, una pequeña desviación mantiene avance con corrección.
    camera.pose(heading=math.radians(40))  # Destino a 45°, quedan 5° a izquierda.
    wait_for(lambda: any(l >= 0 and r > l for l, r in robot.drives))
    assert nav.state()["phase"] == "advancing"
    # La histéresis no vuelve a giro con 15°; sí cuando supera 25°.
    camera.pose(heading=math.radians(30))
    time.sleep(.03)
    assert nav.state()["phase"] == "advancing"
    camera.pose(heading=math.radians(10))
    wait_for(lambda: nav.state()["phase"] == "turning")


@pytest.mark.parametrize("bearing", [-1., -.05, 0., .05, 1.])
def test_commands_are_bounded_physical_wheel_velocities(bearing):
    for mode in ("turning", "advancing"):
        left, right = Navigation._wheel_commands(mode, bearing, .1, .3)
        assert -100 <= left <= 100 and -100 <= right <= 100
    assert Navigation._wheel_commands("turning", .5, .1, 0) == (-38, 38)
    assert Navigation._wheel_commands("turning", -.5, .1, 0) == (38, -38)
    if bearing > 0:
        left, right = Navigation._wheel_commands("advancing", bearing, .1, .3)
        assert left < right
    if bearing < 0:
        left, right = Navigation._wheel_commands("advancing", bearing, .1, .3)
        assert left > right


def test_speed_reduces_with_distance_and_angle_continuously():
    assert Navigation._wheel_commands("advancing", 0, .3, .3) == (100, 100)
    near = Navigation._wheel_commands("advancing", 0, .03, .3)
    assert near == (50, 50)
    assert Navigation._wheel_commands("turning", math.radians(40), .3, .3) == (-53, 53)
    assert Navigation._wheel_commands("turning", math.radians(10), .3, .3) == (-20, 20)


def test_lost_vision_pauses_without_stale_motion_and_resumes_same_target(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    camera.update(visible=False, marker_corners=None)
    assert nav.state()["center_px"] is None
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    assert nav.state()["active"] is True and nav.state()["calibrated"] is True
    assert nav.state()["target_px"] == [900., 480.]
    count = len(robot.drives)
    time.sleep(.05)
    assert robot.drives[count:] and all(command == (0, 0) for command in robot.drives[count:])
    assert robot.stops == 0
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.starts == 1 and nav.state()["active"] is True


@pytest.mark.parametrize("change", [{"camera_id": "another-camera"}, {"frame_width": 1920},
                                      {"frame_height": 1080}])
def test_source_change_cancels_and_invalidates_calibration(rig, change):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    camera.update(**change)
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["calibrated"] is False
    assert nav.state()["target_px"] is None
    assert robot.stops == 1
    with pytest.raises(NavigationError):
        nav.start()
    assert robot.starts == 1


@pytest.mark.parametrize("reason", ["button_stop", "driver_fault", "remote_stop", "remote", "manual_stop"])
def test_remote_stop_cancels_without_rearming(rig, reason):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(navigation_active=False, stop_reason=reason)
    wait_for(lambda: not nav.state()["active"])
    count = len(robot.drives)
    robot.update(ready=True, stop_reason="idle")
    time.sleep(.04)
    assert robot.starts == 1 and len(robot.drives) == count
    assert robot.stops == 1


def test_changed_boot_waits_and_rearms_new_session(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(boot_id="boot-2", navigation_active=False, control_session=None, stop_reason="idle")
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.starts == 2 and robot.abandons == 1


def test_initial_cached_status_has_bounded_guard(rig):
    _, robot, nav = rig
    robot.stale_start = True
    nav.STATUS_STARTUP_SECONDS = .08
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: len(robot.drives) >= 2)
    assert nav.state()["active"] is True
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    assert robot.stops == 0 and robot.abandons == 1


def test_other_session_pauses_without_stealing_control(rig):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(control_session="different-session")
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    assert robot.stops == 0 and robot.starts == 1


def test_no_progress_retries_without_cancelling_or_increasing_cap(rig):
    _, robot, nav = rig
    nav.NO_PROGRESS_SECONDS = .05
    nav.RETRY_PAUSE_SECONDS = .03
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: nav.state()["phase"] == "retrying")
    assert nav.state()["active"] is True and robot.stops == 0
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert all(abs(left) <= 100 and abs(right) <= 100 for left, right in robot.drives)


def test_elapsed_duration_is_diagnostic_and_does_not_cancel(rig):
    _, robot, nav = rig
    nav.MAX_DURATION_SECONDS = .03
    calibrate(nav)
    target(nav)
    nav.start()
    time.sleep(.07)
    assert nav.state()["active"] is True and robot.stops == 0


def test_transport_failure_pauses_then_rearms_after_connection_recovers(rig):
    camera, robot, nav = rig
    robot.fail_drive = True
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    assert robot.starts == 1 and robot.stops == 0 and nav.state()["active"] is True
    robot.fail_drive = False
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.starts == 2


def test_cancel_local_has_no_io_and_worker_stops_after_cancel(rig):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    # Bloquear el snapshot pausa el trabajador; el callback no espera a este lock.
    with robot.lock:
        before = robot.stops
        nav.cancel_local("D48")
        assert robot.stops == before
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["message"] == "D48"


def test_start_requires_calibration_target_fresh_image_and_ready_robot(rig):
    camera, robot, nav = rig
    with pytest.raises(NavigationError):
        nav.start()
    calibrate(nav)
    with pytest.raises(NavigationError):
        nav.start()
    target(nav)
    camera.update(frame_age_ms=201)
    with pytest.raises(NavigationError):
        nav.start()
    camera.update(frame_age_ms=0)
    robot.update(ready=False)
    with pytest.raises(NavigationError):
        nav.start()
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0


@pytest.mark.parametrize("x,y", [(-1, 480), (1280, 480), (640, -1), (640, 960),
                                  (30, 480), (640, 20), (float("nan"), 480),
                                  (True, 480), ("900", 480)])
def test_invalid_or_border_target_is_rejected_without_motion(rig, x, y):
    _, robot, nav = rig
    calibrate(nav)
    with pytest.raises(NavigationError):
        target(nav, x, y)
    assert nav.state()["target_px"] is None
    assert robot.starts == 0 and robot.drives == []


def test_far_target_is_rejected_and_request_source_must_match(rig):
    camera, robot, nav = rig
    camera.pose(scale=500.)
    calibrate(nav)
    with pytest.raises(NavigationError, match="600 mm"):
        target(nav, 1100, 480)
    with pytest.raises(NavigationError, match="imagen cambió"):
        nav.select_target(800, 480, 640, 480, "usb-camera")
    assert robot.starts == 0


def test_cannot_retarget_or_recalibrate_while_moving(rig):
    _, _, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    with pytest.raises(NavigationError, match="Detén"):
        target(nav, 800, 480)
    with pytest.raises(NavigationError, match="Detén"):
        calibrate(nav)
    with pytest.raises(NavigationError, match="Detén"):
        nav.start()


def test_calibration_and_target_persist_without_automatic_movement(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    path = tmp_path / "calibration.json"
    nav = Navigation(camera, robot, path)
    calibrate(nav)
    target(nav)
    restored = Navigation(camera, robot, path)
    assert restored.state()["calibrated"] is True
    assert restored.state()["target_px"] == [900., 480.] and restored.state()["active"] is False
    assert set(json.loads(path.read_text())) == {"version", "calibration", "target_px", "pending_mission"}
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0
    camera.update(camera_id="new-camera")
    assert restored.state()["calibrated"] is False
    assert json.loads(path.read_text())["calibration"] is None
    nav.close()
    restored.close()


def test_overlay_uses_calibrated_body_center_and_hides_lost_detection(rig):
    camera, robot, nav = rig
    nav.calibrate([660., 500.], [760., 500.], 1280, 960, "usb-camera")
    assert nav.overlay_pose(camera.state())["center_px"] == pytest.approx([660., 500.], abs=.01)
    detected = Detection(True, marker_corners=camera.state()["marker_corners"])
    assert nav.overlay_pose(detected)["center_px"] == pytest.approx([660., 500.], abs=.01)
    assert nav.overlay_pose(Detection(False)) is None
    assert nav.overlay_pose({"visible": False}) is None
    assert robot.starts == 0 and robot.drives == []


def test_overlay_does_not_invalidate_or_write_calibration(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    path = tmp_path / "calibration.json"
    nav = Navigation(camera, robot, path)
    calibrate(nav)
    before = path.read_bytes()
    camera.update(camera_id="different-camera")
    assert nav.overlay_pose(Detection(True, marker_corners=camera.state()["marker_corners"])) is None
    assert path.read_bytes() == before
    nav.close()


def test_routes_require_panel_header_and_bounded_exact_json(rig):
    _, robot, nav = rig
    app = FastAPI()
    app.include_router(make_navigation_router(nav))
    with TestClient(app) as client:
        headers = {"X-Requested-With": "XMLHttpRequest"}
        assert client.get("/api/navigation/state").status_code == 200
        assert client.post("/api/navigation/start", json={}).status_code == 403
        for body in ("[]", "{", "null", '{"x": NaN}', '{"secret": "dont-return-this"}'):
            response = client.post("/api/navigation/target", content=body, headers=headers)
            assert response.status_code == 422
            assert "dont-return-this" not in response.text
        assert client.post("/api/navigation/start", content=" " * 8193, headers=headers).status_code == 413
        payload = {"center_px": [640, 480], "front_px": [740, 480],
                   "frame_width": 1280, "frame_height": 960, "camera_id": "usb-camera"}
        response = client.post("/api/navigation/calibrate", json=payload, headers=headers)
        assert response.status_code == 200 and response.json()["calibrated"] is True
        payload.pop("center_px")
        payload.pop("front_px")
        payload.update(x=900, y=480)
        response = client.post("/api/navigation/target", json=payload, headers=headers)
        assert response.status_code == 200 and response.json()["phase"] == "selected"
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0


@pytest.mark.parametrize("goal", [(.18, 0.), (0., .15), (0., -.15), (-.18, 0.),
                                  (-.14, .12), (-.14, -.12)])
@pytest.mark.parametrize("profile", [(100, 32), (300, 100)])
def test_closed_loop_camera_robot_reaches_forward_side_and_behind_goals(goal, profile):
    camera = FakeCamera()
    robot = ClosedLoopRobot(camera)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SETTLE_SECONDS = .01
    # La planta virtual avanza 20 ms por ciclo de control de 1 ms: acelerar
    # también las rampas conserva sus pendientes en el tiempo de esa planta.
    nav.SPEED_RISE_MM_S2 = 2400.
    nav.SPEED_FALL_MM_S2 = 3600.
    try:
        robot.update(max_velocity_mm_s=300)
        nav.set_speeds(*profile)
        calibrate(nav)
        selected = target(nav, 640 + goal[0] * 2000, 480 - goal[1] * 2000)
        initial_bearing = selected["bearing_deg"]
        assert robot.starts == 0 and robot.drives == []  # Seleccionar no actúa.
        nav.start()
        wait_for(lambda: not nav.state()["active"], timeout=3.)
        result = nav.state()
        assert result["phase"] == "arrived", result["message"]
        assert math.hypot(robot.x - goal[0], robot.y - goal[1]) <= nav.ARRIVAL_M + .001
        assert result["remaining_mm"] <= 26
        first_left, first_right = next(command for command in robot.drives if command != (0, 0))
        if abs(initial_bearing) <= 8:
            assert 0 < first_left <= 100 and 0 < first_right <= 100
        elif initial_bearing > 0:
            assert -53 <= first_left < 0 and 0 < first_right <= 53
        else:
            assert 0 < first_left <= 53 and -53 <= first_right < 0
        assert any(left > 0 and right > 0 for left, right in robot.drives)
        assert robot.drives[-1] == (0, 0) and robot.stops == 1 and robot.starts == 1
        assert all(abs(left) <= profile[0] and abs(right) <= profile[0] for left, right in robot.drives)
        assert robot.required_caps == [max(profile[0], math.ceil(95 * math.radians(profile[1])))]
        assert result["last_left"] == 0 and result["last_right"] == 0
        assert result["elapsed_ms"] > 0 and result["sequence"] > 1
    finally:
        nav.close()


def test_diagnostic_state_tracks_live_bearing_commands_and_elapsed_time(rig):
    camera, robot, nav = rig
    calibrate(nav)
    selected = target(nav, 840, 280)
    assert selected["bearing_deg"] == pytest.approx(45., abs=.01)
    assert selected["last_left"] == selected["last_right"] == selected["elapsed_ms"] == 0
    nav.start()
    wait_for(lambda: bool(robot.drives))
    state = nav.state()
    assert -53 <= state["last_left"] <= 0 and 0 <= state["last_right"] <= 53
    assert state["sequence"] == camera.state()["sequence"]
    time.sleep(.02)
    assert nav.state()["elapsed_ms"] > state["elapsed_ms"]
    camera.update(visible=False, marker_corners=None)
    assert nav.state()["bearing_deg"] is None
    nav.cancel_local()
    wait_for(lambda: not nav.state()["active"])
    stopped = nav.state()
    assert stopped["last_left"] == stopped["last_right"] == 0
    elapsed = stopped["elapsed_ms"]
    time.sleep(.02)
    assert nav.state()["elapsed_ms"] == elapsed


def test_resume_requires_three_distinct_frames_and_stable_interval(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    camera.update(visible=False)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    camera.update(visible=True)  # Un solo fotograma no basta, aunque el fake declare edad cero.
    time.sleep(nav.RESUME_STABLE_SECONDS * 2)
    assert nav.state()["phase"] == "paused_vision"
    assert robot.drives[-1] == (0, 0)
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.starts == 1


def test_temporary_unknown_camera_source_preserves_calibration_and_target(rig):
    camera, _, nav = rig
    calibrate(nav)
    target(nav)
    saved = nav._calibration
    nav.start()
    camera.update(visible=False, camera_id=None, frame_width=0, frame_height=0)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    assert nav._calibration == saved and nav.state()["target_px"] == [900., 480.]
    camera.update(camera_id="usb-camera", frame_width=1280, frame_height=960)
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert nav.state()["calibrated"] is True


@pytest.mark.parametrize("reason", ["wifi_disconnected", "control_timeout"])
def test_connection_and_watchdog_pause_then_rearm_new_session(rig, reason):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(connected=False, ready=False, navigation_active=False, control_session=None,
                 stop_reason=reason)
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    count = len(robot.drives)
    time.sleep(.06)
    assert len(robot.drives) == count and robot.stops == 0 and robot.abandons == 1
    assert nav.state()["active"] is True and nav.state()["pending_mission"] is True
    robot.update(connected=True, ready=True)
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.starts == 2 and robot.value["control_session"] == "session-2"
    assert nav.state()["target_px"] == [900., 480.]


@pytest.mark.parametrize("reason", ["button_stop", "remote_stop", "manual_stop"])
def test_manual_stop_during_connection_pause_permanently_cancels_mission(rig, reason):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(connected=False, ready=False)
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    robot.update(stop_reason=reason)
    wait_for(lambda: not nav.state()["active"])
    assert "cancelado" in nav.state()["message"]
    robot.update(connected=True, ready=True, stop_reason="idle")
    stable_vision(camera, nav)
    assert robot.starts == 1 and nav.state()["pending_mission"] is False


def test_active_deadlines_freeze_during_vision_wait(rig):
    camera, robot, nav = rig
    nav.MAX_DURATION_SECONDS = .12
    nav.NO_PROGRESS_SECONDS = .08
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    camera.update(visible=False)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    time.sleep(.18)  # Más que ambos presupuestos: la espera no los consume.
    assert nav.state()["active"] is True and robot.stops == 0
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    wait_for(lambda: nav.state()["phase"] == "retrying")
    assert nav.state()["active"] is True and robot.stops == 0


def test_cancel_during_delayed_rearm_stops_new_ack_without_drive(rig):
    camera, _, original = rig
    original.close()

    class DelayedRobot(FakeRobot):
        entered = threading.Event()
        release = threading.Event()

        def start_navigation(self, *, resume=False, require_velocity=False, required_velocity_mm_s=100):
            if self.starts:
                self.entered.set()
                self.release.wait(2.)
            return super().start_navigation(resume=resume, require_velocity=require_velocity,
                                             required_velocity_mm_s=required_velocity_mm_s)

    robot = DelayedRobot()
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .005
    nav.RESUME_STABLE_SECONDS = .02
    try:
        calibrate(nav)
        target(nav)
        nav.start()
        wait_for(lambda: bool(robot.drives))
        robot.update(connected=False, ready=False)
        wait_for(lambda: nav.state()["phase"] == "paused_connection")
        robot.update(connected=True, ready=True)
        stable_vision(camera, nav)
        assert robot.entered.wait(.5)
        nav.cancel_local("Parada manual.")
        wait_for(lambda: not nav.state()["active"])
        count = len(robot.drives)
        with pytest.raises(NavigationError, match="Detén"):
            nav.start()
        robot.release.set()
        wait_for(lambda: not nav._arm_thread.is_alive())
        assert robot.starts == 2 and robot.abandons == 2
        assert robot.value["navigation_active"] is False and len(robot.drives) == count
        nav.start()  # Un inicio posterior no hereda el ACK de la sesión cancelada.
        wait_for(lambda: len(robot.drives) > count)
        assert robot.starts == 3 and nav._session == "session-3"
    finally:
        robot.release.set()
        nav.close()


def test_pending_mission_survives_service_restart_but_needs_explicit_start(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    path = tmp_path / "navigation-state.json"
    nav = Navigation(camera, robot, path)
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    nav.close()
    assert json.loads(path.read_text())["pending_mission"] is True
    other = FakeRobot()
    restored = Navigation(camera, other, path)
    try:
        state = restored.state()
        assert state["pending_mission"] is True and state["active"] is False
        assert state["target_px"] == [900., 480.]
        assert other.starts == other.stops == 0 and other.drives == []
        stable_vision(camera, restored)
        assert other.starts == 0
        restored.start()
        wait_for(lambda: bool(other.drives))
        restored.stop()
        wait_for(lambda: not restored.state()["active"])
        assert json.loads(path.read_text())["pending_mission"] is False
    finally:
        restored.close()


def test_old_calibration_file_loads_without_device_io_or_implicit_migration_write(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    original = Navigation(camera, robot)
    calibrate(original)
    legacy = tmp_path / "navigation-calibration.json"
    legacy.write_text(json.dumps(original._calibration.to_dict()))
    path = tmp_path / "navigation-state.json"
    restored = Navigation(camera, robot, path)
    try:
        assert restored.state()["calibrated"] is True and not path.exists()
        assert robot.starts == robot.stops == 0 and robot.drives == []
        target(restored)
        assert json.loads(path.read_text())["version"] == 1
        assert json.loads(path.read_text())["target_px"] == [900., 480.]
    finally:
        original.close()
        restored.close()


def test_manual_stop_found_by_fresh_rearm_status_cancels_without_retry(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(connected=False, ready=False)
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    robot.reject_resume = True  # D48 llega después del snapshot, antes del GET fresco de armado.
    robot.update(connected=True, ready=True)
    stable_vision(camera, nav)
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["pending_mission"] is False and nav.state()["phase"] == "error"
    assert "cancelado" in nav.state()["message"]
    assert robot.resume_calls == [False, True] and robot.starts == 1
    time.sleep(.07)
    assert robot.resume_calls == [False, True]


def test_old_but_visible_frame_pauses_without_using_cached_motion(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    camera.update(frame_age_ms=190.)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    count = len(robot.drives)
    time.sleep(.03)
    assert robot.drives[count:] and all(command == (0, 0) for command in robot.drives[count:])
    assert nav.state()["active"] is True and nav.state()["target_px"] == [900., 480.]


def test_metric_acceleration_ramp_and_reverse_require_an_intermediate_zero():
    value = 0.
    values = []
    for _ in range(12):
        previous = value
        value = Navigation._ramp_value(value, 65., .05)
        values.append(value)
        assert 0 <= value - previous <= 6. + 1e-8
    assert values[-1] == pytest.approx(65.)
    reverse = []
    for _ in range(20):
        previous = value
        value = Navigation._ramp_value(value, -55., .05)
        reverse.append(value)
        if previous > 0:
            assert value >= 0 and previous - value <= 9. + 1e-8
    assert 0. in reverse and min(reverse) < 0
    zero_index = reverse.index(0.)
    assert all(item >= 0 for item in reverse[:zero_index])
    assert reverse[zero_index + 1] < 0


def test_physical_targets_use_geometry_and_do_not_limit_motor_pwm_from_visual_speed(rig):
    _, _, nav = rig
    assert Navigation._wheel_commands("advancing", 0., .3, 0.) == (100, 100)
    assert Navigation._wheel_commands("turning", math.pi / 2, .3, 0.) == (-53, 53)
    for visual_speed in (-1., 0., .05, .1, 1.):
        nav._measured_speed = visual_speed  # Diagnóstico; no modifica la consigna física.
        assert nav._wheel_commands("advancing", 0., .3, 0.) == (100, 100)
        left, right = nav._wheel_commands("advancing", .1, .3, 0.)
        assert 0 <= left < right <= 100
    # 53 mm/s por rueda opuesta equivale a ~32°/s con ejes a190 mm.
    left, right = nav._wheel_commands("turning", math.pi / 2, .3, 0.)
    assert math.degrees((right - left) / 190.) == pytest.approx(32., abs=.2)


def test_visual_velocity_estimate_uses_distinct_frames_and_metric_tag_scale(rig):
    camera, _, nav = rig
    calibrate(nav)
    frame = camera.state()
    nav._update_motion_estimate(frame, observe_robot(frame["marker_corners"], nav._calibration), 1.)
    camera.pose(x=660.)  # Diez milímetros en 200 ms.
    frame = camera.state()
    nav._update_motion_estimate(frame, observe_robot(frame["marker_corners"], nav._calibration), 1.2)
    assert nav.state()["measured_speed_mps"] == pytest.approx(.05, abs=.001)
    assert nav.state()["measured_turn_deg_s"] == pytest.approx(0., abs=.001)
    nav._update_motion_estimate(frame, observe_robot(frame["marker_corners"], nav._calibration), 1.4)
    assert nav.state()["measured_speed_mps"] == pytest.approx(.05, abs=.001)  # No duplica una imagen.
    nav._update_motion_estimate(frame, None, 1.5)
    assert nav.state()["measured_speed_mps"] is None


def test_latched_resume_block_cancels_even_if_reason_was_overwritten(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(resume_blocked=True, stop_reason="wifi_disconnected", connected=False, ready=False)
    wait_for(lambda: not nav.state()["active"])
    robot.update(connected=True, ready=True, stop_reason="idle")
    stable_vision(camera, nav)
    assert robot.starts == 1 and nav.state()["pending_mission"] is False


def test_saved_camera_identity_is_readonly_even_while_capture_source_is_unknown(rig):
    camera, robot, nav = rig
    assert nav.saved_camera_id() is None
    calibrate(nav)
    target(nav)
    camera.update(camera_id=None, frame_width=0, frame_height=0, visible=False)
    assert nav.saved_camera_id() == "usb-camera"
    assert nav.state()["target_px"] == [900., 480.]
    assert robot.starts == robot.stops == 0 and robot.drives == []
    camera.update(camera_id="different-camera", frame_width=1280, frame_height=960)
    # Leer el getter no ejecuta _check_source ni borra la calibración.
    assert nav.saved_camera_id() == "usb-camera"
    assert nav.state()["calibrated"] is False
    assert nav.saved_camera_id() is None


@pytest.mark.parametrize("field,value", [("velocity_control", False), ("velocity_control", None),
                                          ("encoder_ready", False), ("encoder_ready", None)])
def test_start_rejects_unavailable_encoder_velocity_capabilities_without_fallback(rig, field, value):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    robot.update(**{field: value})
    with pytest.raises(NavigationError, match="encoders"):
        nav.start()
    assert robot.starts == robot.stops == 0 and robot.drives == []


def test_velocity_target_telemetry_has_explicit_units_and_fresh_arm_requirement(rig):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: any(left > 0 for left, _ in robot.drives))
    state = nav.state()
    assert state["left_target_mm_s"] == state["last_left"]
    assert state["right_target_mm_s"] == state["last_right"]
    assert robot.velocity_requirements == [True]


def test_encoder_control_becoming_unavailable_cancels_instead_of_open_loop_pwm(rig):
    camera, robot, nav = rig
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(encoder_ready=False)
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["phase"] == "error" and nav.state()["pending_mission"] is False
    robot.update(encoder_ready=True)
    stable_vision(camera, nav)
    assert robot.starts == 1


def begin_observed_motion(camera, robot, nav):
    camera.update(running=True, opening=False)
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: any(left > 0 and right > 0 for left, right in robot.drives))
    camera.pose()
    sequence = camera.state()["sequence"]
    wait_for(lambda: nav._velocity_sample is not None and nav._velocity_sample[0] == sequence)


def test_single_missing_detection_keeps_exact_command_without_zero_or_ramp_reset(rig):
    camera, robot, nav = rig
    begin_observed_motion(camera, robot, nav)
    camera.update(visible=False, marker_corners=None)
    wait_for(lambda: nav.state()["vision_grace_active"])
    held = robot.drives[-1]
    count = len(robot.drives)
    time.sleep(.025)
    assert held != (0, 0) and robot.drives[count:]
    assert all(command == held for command in robot.drives[count:])
    assert nav.state()["center_px"] is None  # No publica una pose vieja como detección actual.
    camera.pose()
    camera.update(visible=True)
    wait_for(lambda: not nav.state()["vision_grace_active"])
    assert nav.state()["phase"] == "advancing"
    assert nav.state()["brief_vision_gaps"] == 1 and nav.state()["vision_pauses"] == 0
    assert (0, 0) not in robot.drives[count:]


def test_invalid_fresh_frames_do_not_extend_100ms_hold_then_stable_recovery_is_required(rig):
    camera, robot, nav = rig
    begin_observed_motion(camera, robot, nav)
    camera.update(visible=False, marker_corners=None)
    wait_for(lambda: nav.state()["vision_grace_active"])
    deadline = time.monotonic() + .15
    while time.monotonic() < deadline:
        camera.update(visible=False, frame_age_ms=0.)  # Cámaras activas pueden producir muchos rechazos.
        state = nav.state()
        if state["vision_grace_active"]:
            assert state["vision_gap_ms"] <= 100.
        time.sleep(.012)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    assert robot.drives[-1] == (0, 0)
    assert nav.state()["brief_vision_gaps"] == 1 and nav.state()["vision_pauses"] == 1
    camera.pose()
    camera.update(visible=True)
    time.sleep(.02)
    assert nav.state()["phase"] == "paused_vision"  # Un único buen fotograma no reactiva una pausa larga.
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert nav.state()["target_px"] == [900., 480.]


@pytest.mark.parametrize("failure", [{"running": False}, {"opening": True},
                                     {"frame_age_ms": 190.}, {"camera_id": None},
                                     {"frame_width": 0, "frame_height": 0}])
def test_camera_disconnect_or_stale_source_bypasses_brief_hold(rig, failure):
    camera, robot, nav = rig
    begin_observed_motion(camera, robot, nav)
    camera.update(visible=False, **failure)
    wait_for(lambda: nav.state()["phase"] == "paused_vision", timeout=.08)
    assert robot.drives[-1] == (0, 0)
    assert nav.state()["brief_vision_gaps"] == 0


def test_missing_detection_after_arm_does_not_start_from_previous_pose(rig):
    camera, robot, nav = rig
    camera.update(running=True, opening=False)
    calibrate(nav)
    target(nav)
    original = robot.start_navigation

    def arm(**kwargs):
        ack = original(**kwargs)
        camera.update(visible=False)
        return ack

    robot.start_navigation = arm
    nav.start()
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    assert all(command == (0, 0) for command in robot.drives)
    assert nav.state()["brief_vision_gaps"] == 0


def test_brief_hold_never_confirms_arrival_and_manual_stop_overrides_it(rig):
    camera, robot, nav = rig
    begin_observed_motion(camera, robot, nav)
    camera.pose(x=900.)
    camera.update(visible=False)
    wait_for(lambda: nav.state()["vision_grace_active"])
    assert nav.state()["phase"] == "advancing" and nav.state()["active"] is True
    robot.update(stop_reason="button_stop")
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["phase"] != "arrived" and robot.stops == 1


def test_moderate_speed_increase_keeps_precision_floor():
    assert Navigation._wheel_commands("advancing", 0., .3, 0.) == (100, 100)
    assert Navigation._wheel_commands("advancing", 0., 0., 0.) == (50, 50)
    assert Navigation._wheel_commands("turning", math.pi / 2, .3, 0.) == (-53, 53)
    assert math.degrees(106. / 190.) == pytest.approx(32., abs=.2)


def test_forward_floor_does_not_clamp_inner_wheel_and_distort_steering():
    left, right = Navigation._wheel_commands("advancing", .3, .03, 0.)
    assert (left, right) == (30, 70)  # Centro a50 mm/s, corrección diferencial conservada.
    assert left < 50 and (left + right) / 2 == 50
    for bearing in (-.4, -.1, 0., .1, .4):
        left, right = Navigation._wheel_commands("advancing", bearing, .6, 0.)
        assert 0 <= left <= 100 and 0 <= right <= 100


def test_speed_floor_never_applies_to_arrival_or_manual_stop(rig):
    _, robot, nav = rig
    calibrate(nav)
    target(nav, 640., 480.)
    nav.start()
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["phase"] == "arrived"
    assert robot.drives and all(command == (0, 0) for command in robot.drives)
    assert nav.state()["left_target_mm_s"] == nav.state()["right_target_mm_s"] == 0
    nav.stop()
    assert nav.state()["left_target_mm_s"] == nav.state()["right_target_mm_s"] == 0


@pytest.mark.parametrize("forward,turn", [(49, 32), (301, 32), (100, 11), (100, 101),
                                          (True, 32), (100, False), (100., 32),
                                          (100, float("nan"))])
def test_speed_settings_reject_invalid_values_without_changing_state(rig, forward, turn):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    before = nav.state()
    with pytest.raises(NavigationError):
        nav.set_speeds(forward, turn)
    after = nav.state()
    assert after["speeds"] == before["speeds"] and after["speed_revision"] == 0
    assert after["target_px"] == before["target_px"] and after["calibrated"] is True
    assert robot.starts == robot.stops == 0 and robot.drives == []


def test_speed_settings_persist_separately_without_starting_or_resetting_geometry(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    robot.update(max_velocity_mm_s=300)
    path = tmp_path / "navigation-state.json"
    nav = Navigation(camera, robot, path)
    calibrate(nav)
    target(nav)
    geometry = path.read_bytes()
    result = nav.set_speeds(220, 75)
    assert result["speeds"] == {"forward_mm_s": 220, "turn_deg_s": 75}
    assert result["speed_revision"] == 1 and result["active"] is False
    assert path.read_bytes() == geometry
    assert json.loads(path.with_name("navigation-speeds.json").read_text()) == {
        **result["speeds"], "speed_revision": 1}
    restored = Navigation(camera, robot, path)
    try:
        assert restored.get_speeds() == result["speeds"]
        assert restored.state()["calibrated"] is True and restored.state()["target_px"] == [900., 480.]
        assert restored.state()["active"] is False and restored.state()["speed_revision"] == 1
        assert robot.starts == robot.stops == 0 and robot.drives == []
        copy = restored.get_speeds()
        copy["forward_mm_s"] = 50
        assert restored.get_speeds()["forward_mm_s"] == 220
        assert restored.set_speeds(200, 50)["speed_revision"] == 2
    finally:
        nav.close()
        restored.close()


def test_speed_settings_old_firmware_rejects_connected_but_allows_offline_configuration(rig):
    _, robot, nav = rig
    calibrate(nav)
    target(nav)
    with pytest.raises(NavigationError, match="firmware"):
        nav.set_speeds(300, 100)
    with pytest.raises(NavigationError, match="firmware"):
        nav.set_speeds(50, 100)  # El giro requiere166 mm/s aunque el avance sea50.
    assert nav.state()["speed_revision"] == 0
    robot.update(connected=False, ready=False)
    nav.set_speeds(300, 100)
    assert nav.state()["speed_revision"] == 1
    robot.update(connected=True, ready=True)
    with pytest.raises(NavigationError, match="firmware"):
        nav.start()
    assert robot.starts == robot.stops == 0 and robot.drives == []


def test_speed_change_during_motion_keeps_mission_and_uses_slew_without_direct_commands(rig):
    _, robot, nav = rig
    robot.update(max_velocity_mm_s=300)
    samples = []
    original = robot.drive_velocity_navigation

    def record(left, right):
        samples.append((time.monotonic(), left, right, threading.current_thread().name))
        original(left, right)

    robot.drive_velocity_navigation = record
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: any(left >= 10 for left, _ in robot.drives))
    result = nav.set_speeds(300, 100)
    assert result["active"] is True and result["speed_revision"] == 1
    assert result["target_px"] == [900., 480.] and result["calibrated"] is True
    time.sleep(.06)
    assert robot.starts == 1 and all(sample[3] == "webcam-point-navigation" for sample in samples)
    for previous, current in zip(samples, samples[1:]):
        assert current[1] - previous[1] <= 120 * (current[0] - previous[0]) + 2


def test_capacity_loss_pauses_without_clipping_then_recovers_or_manual_stop_cancels(rig):
    camera, robot, nav = rig
    robot.update(max_velocity_mm_s=300)
    nav.set_speeds(300, 100)
    calibrate(nav)
    target(nav)
    nav.start()
    wait_for(lambda: bool(robot.drives))
    robot.update(max_velocity_mm_s=100)
    wait_for(lambda: nav.state()["phase"] == "paused_connection")
    count = len(robot.drives)
    time.sleep(.03)
    assert len(robot.drives) == count and nav.state()["speeds"]["forward_mm_s"] == 300
    assert robot.stops == 0 and robot.abandons == 1
    robot.update(max_velocity_mm_s=300)
    stable_vision(camera, nav)
    wait_for(lambda: nav.state()["phase"] == "advancing")
    assert robot.required_caps == [300, 300]
    robot.update(max_velocity_mm_s=100, resume_blocked=True, stop_reason="button_stop")
    wait_for(lambda: not nav.state()["active"])
    assert nav.state()["pending_mission"] is False


def test_high_speed_profiles_brake_from_geometry_before_the_fixed_approach_zone():
    high_far = Navigation._wheel_commands("advancing", 0., .6, 0., forward_mm_s=300, turn_deg_s=100)
    high_mid = Navigation._wheel_commands("advancing", 0., .3, 0., forward_mm_s=300, turn_deg_s=100)
    high_near = Navigation._wheel_commands("advancing", 0., .1, 0., forward_mm_s=300, turn_deg_s=100)
    assert high_far == (300, 300)
    assert 100 < high_near[0] < high_mid[0] < high_far[0]  # Frenado antes de150 mm.
    assert Navigation._wheel_commands("advancing", 0., .03, 0., forward_mm_s=300) == (50, 50)
    far_turn = Navigation._wheel_commands("turning", math.pi / 2, .3, 0., turn_deg_s=100)
    mid_turn = Navigation._wheel_commands("turning", math.radians(40), .3, 0., turn_deg_s=100)
    near_turn = Navigation._wheel_commands("turning", math.radians(9), .3, 0., turn_deg_s=100)
    assert 20 <= abs(near_turn[0]) < abs(mid_turn[0]) < abs(far_turn[0]) <= 166
    for mode in ("turning", "advancing"):
        for angle in (-math.pi, -.3, 0., .3, math.pi):
            assert all(abs(value) <= 300 for value in Navigation._wheel_commands(
                mode, angle, .6, 0., forward_mm_s=300, turn_deg_s=100))


def test_speed_routes_exact_json_header_revision_and_no_go(rig):
    _, robot, nav = rig
    robot.update(max_velocity_mm_s=300)
    app = FastAPI()
    app.include_router(make_navigation_router(nav))
    headers = {"X-Requested-With": "XMLHttpRequest"}
    with TestClient(app) as client:
        initial = client.get("/api/navigation/state").json()
        assert initial["speeds"] == {"forward_mm_s": 100, "turn_deg_s": 32}
        assert initial["speed_limits"] == {"forward_min_mm_s": 50, "forward_max_mm_s": 300,
                                            "turn_min_deg_s": 12, "turn_max_deg_s": 100}
        assert client.post("/api/navigation/speeds", json=initial["speeds"]).status_code == 403
        for payload in ({"forward_mm_s": True, "turn_deg_s": 32}, {"forward_mm_s": 300},
                        {"forward_mm_s": 301, "turn_deg_s": 32},
                        {"forward_mm_s": 100, "turn_deg_s": 32, "extra": 0}):
            assert client.post("/api/navigation/speeds", json=payload, headers=headers).status_code == 422
        assert client.post("/api/navigation/speeds", content='{"forward_mm_s":NaN,"turn_deg_s":32}',
                           headers=headers).status_code == 422
        first = client.post("/api/navigation/speeds", json={"forward_mm_s": 300, "turn_deg_s": 100},
                            headers=headers).json()
        second = client.post("/api/navigation/speeds", json={"forward_mm_s": 50, "turn_deg_s": 12},
                             headers=headers).json()
        assert (first["speed_revision"], second["speed_revision"]) == (1, 2)
        assert first["speeds"] == {"forward_mm_s": 300, "turn_deg_s": 100}
        assert second["speeds"] == {"forward_mm_s": 50, "turn_deg_s": 12}
        assert first["active"] is False and second["active"] is False
    assert robot.starts == robot.stops == 0 and robot.drives == []


def test_loading_legacy_speed_configuration_and_getters_never_touch_robot(tmp_path):
    class NoHardware:
        def __getattr__(self, name):
            raise AssertionError(f"El constructor o el getter intentó acceder al robot: {name}")

    path = tmp_path / "navigation-state.json"
    speed_path = path.with_name("navigation-speeds.json")
    speed_path.write_text(json.dumps({"forward_mm_s": 300, "turn_deg_s": 100}))
    nav = Navigation(FakeCamera(), NoHardware(), path)
    assert nav.get_speeds() == {"forward_mm_s": 300, "turn_deg_s": 100}
    assert nav.state()["speed_revision"] == 0 and nav.state()["active"] is False
    # No crea hilos ni recursos del robot que deban cerrarse en este caso.
    assert nav._thread is None and nav._arm_thread is None


def test_old_firmware_capacity_fallback_and_turn_conversion_are_explicit(rig):
    _, robot, nav = rig
    with robot.lock:
        robot.value.pop("max_velocity_mm_s")
    nav.set_speeds(50, 60)  # 60°/s requiere aproximadamente99,5 mm/s por rueda.
    with pytest.raises(NavigationError, match="firmware"):
        nav.set_speeds(50, 61)
    assert nav.get_speeds() == {"forward_mm_s": 50, "turn_deg_s": 60}
    assert robot.starts == robot.stops == 0 and robot.drives == []
