"""Manos y regreso a Casa con dobles locales; no abre cámara ni motores."""
import json
import threading

import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot
from simulation.webcam.home_behavior import HomeBehavior, HomeBehaviorError, hand_clearance_mm
from simulation.webcam.navigation_geometry import Calibration


class Clock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now

    def advance(self, seconds=.2):
        self.now += seconds


class Navigation:
    def __init__(self, camera):
        self.robot = FakeRobot()
        self._calibration = Calibration((0., 0.), 0., 'usb-camera', 1280, 960)
        self.value = {'phase': 'selected', 'active': False, 'route_revision': 1}
        self.returns, self.starts, self.stops = [], 0, 0

    def state(self):
        return dict(self.value)

    def return_home(self, point, source, *, authorize):
        assert authorize()
        self.returns.append((point, source))
        self.starts += 1
        self.value.update(phase='advancing', active=True)
        self.robot.update(stop_reason='navigating', navigation_active=True)
        return self.state()

    def stop(self):
        self.stops += 1
        self.value.update(phase='stopped', active=False)


@pytest.fixture
def rig():
    camera, clock = FakeCamera(), Clock()
    camera.update(running=True, opening=False)
    navigation = Navigation(camera)
    behavior = HomeBehavior(camera, navigation, clock=clock)
    yield camera, navigation, behavior, clock
    behavior.close()


def prepare(rig, point=(1100., 480.)):
    _, _, behavior, _ = rig
    behavior.set_home(*point, 1280, 960, 'usb-camera')
    behavior.set_enabled(True)


def sample(rig, sequence=1, *, box=(850., 440., 930., 520.), confidence=.9, **changes):
    _, _, behavior, clock = rig
    state = {'ready': True, 'status': 'ready', 'model': 'YOLO hand', 'error': None,
             'captured_at': clock.now, 'sequence': sequence, 'frame_width': 1280,
             'frame_height': 960, 'hands': [{'bbox_px': list(box), 'confidence': confidence}]}
    state.update(changes)
    return behavior.observe(state)


def trigger(rig):
    first = sample(rig)
    rig[3].advance()
    return first, sample(rig, 2)


def test_configuration_and_arming_never_move_or_change_the_normal_route(rig):
    _, navigation, behavior, _ = rig
    prepare(rig)
    assert behavior.state()['phase'] == 'watching'
    assert not navigation.returns and navigation.starts == navigation.stops == 0
    behavior.set_distance(125)
    assert behavior.state()['distance_mm'] == 125
    behavior.set_enabled(False)
    assert not navigation.returns and navigation.starts == navigation.stops == 0


def test_two_recent_confirmed_samples_replace_the_mission_once(rig):
    prepare(rig)
    first, second = trigger(rig)
    assert first['phase'] == 'near_hand'
    assert second['phase'] == 'returning' and second['returning']
    assert second['trigger_count'] == 1
    assert rig[1].returns == [([1100., 480.], {'frame_width': 1280, 'frame_height': 960, 'camera_id': 'usb-camera'})]
    for sequence in range(3, 10):
        rig[3].advance()
        sample(rig, sequence)
    assert len(rig[1].returns) == 1


def test_duplicate_frame_cannot_confirm_a_nearby_hand(rig):
    prepare(rig)
    sample(rig)
    rig[3].advance()
    sample(rig)
    assert not rig[1].returns
    assert rig[2].state()['phase'] == 'near_hand'


def test_brief_nearby_detection_requires_the_confirmation_interval(rig):
    prepare(rig)
    sample(rig)
    rig[3].advance(.04)
    sample(rig, 2)
    assert not rig[1].returns
    rig[3].advance(.12)
    sample(rig, 3)
    assert rig[1].starts == 1


@pytest.mark.parametrize('change', [
    {'captured_at': 9.}, {'captured_at': 11.}, {'ready': False, 'status': 'missing_model'},
    {'frame_width': 640}, {'camera_id': 'different-camera'}, {'sequence': True},
])
def test_stale_invalid_or_wrong_camera_hand_results_never_trigger(rig, change):
    prepare(rig)
    sample(rig, **change)
    rig[3].advance()
    sample(rig, **({'sequence': 2} | change))
    assert not rig[1].returns
    assert rig[2].state()['phase'] == 'waiting_detector'


def test_far_or_low_confidence_hands_do_not_trigger(rig):
    prepare(rig)
    sample(rig, box=(1150., 440., 1250., 520.))
    rig[3].advance()
    sample(rig, 2, box=(1150., 440., 1250., 520.))
    rig[3].advance()
    sample(rig, 3, confidence=.4)
    assert not rig[1].returns
    assert rig[2].state()['phase'] == 'watching'


def test_missing_marker_never_uses_an_old_robot_pose(rig):
    prepare(rig)
    rig[0].update(visible=False, marker_corners=None)
    sample(rig)
    rig[3].advance()
    sample(rig, 2)
    assert not rig[1].returns
    assert rig[2].state()['phase'] == 'waiting_vision'


@pytest.mark.parametrize('reason', ['button_stop', 'encoder_stall', 'driver_fault', 'remote_stop'])
def test_a_new_manual_stop_or_fault_disables_reaction_without_rearming(rig, reason):
    prepare(rig)
    rig[1].robot.update(stop_reason=reason, resume_blocked=True)
    sample(rig)
    assert not rig[2].state()['enabled']
    rig[3].advance()
    sample(rig, 2)
    assert not rig[1].returns


def test_initial_encoder_fault_cannot_be_cleared_by_enabling_reaction(rig):
    behavior, navigation = rig[2], rig[1]
    behavior.set_home(1100., 480., 1280, 960, 'usb-camera')
    navigation.robot.update(stop_reason='encoder_stall', resume_blocked=True)
    with pytest.raises(HomeBehaviorError):
        behavior.set_enabled(True)
    assert not behavior.state()['enabled'] and not navigation.returns


def test_arrival_stays_at_home_without_restarting_the_trip(rig):
    prepare(rig)
    trigger(rig)
    rig[1].value.update(active=False, phase='arrived')
    rig[0].pose(x=1100., y=480.)
    rig[3].advance()
    state = sample(rig, 3, box=(1150., 440., 1250., 520.))
    assert state['phase'] == 'at_home' and not state['returning']
    rig[3].advance(2.)
    sample(rig, 4, box=(1150., 440., 1250., 520.))
    assert rig[1].starts == 1


def test_hand_near_a_robot_already_covering_home_never_launches_a_mission(rig):
    prepare(rig, point=(640., 480.))
    _, state = trigger(rig)
    assert state['phase'] == 'at_home'
    assert rig[1].starts == 0


def test_leaving_home_during_another_route_returns_the_status_to_watching(rig):
    prepare(rig, point=(640., 480.))
    trigger(rig)
    assert rig[2].state()['phase'] == 'at_home'
    rig[0].pose(x=900., y=480.)
    rig[3].advance(1.2)
    state = sample(rig, 3, hands=[])
    assert state['phase'] == 'watching'
    assert rig[1].starts == 0


def test_disabling_cancels_its_own_return_but_not_an_unrelated_normal_mission(rig):
    prepare(rig)
    rig[1].value.update(active=True, phase='advancing')
    rig[2].set_enabled(False)
    assert rig[1].stops == 0
    rig[2].set_enabled(True)
    trigger(rig)
    rig[2].set_enabled(False)
    assert rig[1].stops == 1


def test_adjusting_proximity_does_not_cancel_a_trip_already_returning(rig):
    prepare(rig)
    trigger(rig)
    generation = rig[2]._generation
    state = rig[2].set_distance(200)
    assert state['enabled'] and state['returning']
    assert rig[2]._generation == generation
    assert rig[1].starts == 1 and rig[1].stops == 0


def test_a_late_plan_is_not_authorized_after_manual_cancel(rig):
    prepare(rig)
    entered, release = threading.Event(), threading.Event()
    navigation, behavior = rig[1:3]

    def delayed(point, source, *, authorize):
        entered.set()
        assert release.wait(2.)
        if not authorize():
            raise HomeBehaviorError('Cancelado antes de armar.')
        navigation.starts += 1

    navigation.return_home = delayed
    sample(rig)
    rig[3].advance()
    worker = threading.Thread(target=lambda: sample(rig, 2))
    worker.start()
    try:
        assert entered.wait(1.)
        behavior.cancel('D48')
    finally:
        release.set()
        worker.join(2.)
    assert navigation.starts == 0
    assert not behavior.state()['enabled'] and behavior.state()['message'] == 'D48'


def test_camera_change_requires_an_explicit_new_home(rig):
    prepare(rig)
    rig[0].update(camera_id='other-camera')
    sample(rig)
    assert not rig[2].state()['enabled']
    assert not rig[1].returns


def test_persisted_home_and_distance_restore_without_automatic_arming(tmp_path, rig):
    camera, navigation, _, clock = rig
    path = tmp_path / 'home.json'
    first = HomeBehavior(camera, navigation, runtime_path=path, clock=clock)
    first.set_home(1100., 480., 1280, 960, 'usb-camera')
    first.set_distance(175)
    first.set_enabled(True)
    second = HomeBehavior(camera, navigation, runtime_path=path, clock=clock)
    assert second.state()['home_px'] == [1100., 480.]
    assert second.state()['distance_mm'] == 175
    assert not second.state()['enabled']
    assert 'enabled' not in json.loads(path.read_text())
    assert navigation.starts == 0


@pytest.mark.parametrize('value', [True, float('nan'), float('inf'), 0, -10, 2000])
def test_invalid_proximity_settings_are_rejected(rig, value):
    with pytest.raises(HomeBehaviorError):
        rig[2].set_distance(value)


def test_clear_home_never_leaves_an_active_reaction(rig):
    prepare(rig)
    state = rig[2].clear_home()
    assert state['home_px'] is None and not state['enabled']
    assert not rig[1].returns


def test_hand_distance_uses_the_calibrated_hull_not_the_marker_center(rig):
    corners = rig[0].state()['marker_corners']
    calibration = rig[1]._calibration
    assert hand_clearance_mm([850., 440., 930., 520.], corners, calibration) == pytest.approx(21.85, abs=.01)
    shifted = Calibration((.05, 0.), 0., 'usb-camera', 1280, 960)
    assert hand_clearance_mm([850., 440., 930., 520.], corners, shifted) == 0.
    # Una mano que entra por el borde de cámara conserva su caja recortada.
    assert hand_clearance_mm([1150., 440., 1280., 520.], corners, calibration) > 150.
