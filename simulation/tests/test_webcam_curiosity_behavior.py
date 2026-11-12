"""Curiosidad por una mano lejana: destino fijo y prioridad del refugio."""
import threading

import pytest

from simulation.tests.test_webcam_home_behavior import Clock, Navigation, sample
from simulation.tests.test_webcam_point_navigation import FakeCamera
from simulation.webcam.home_behavior import HomeBehavior, HomeBehaviorError


FAR = (1080., 420., 1180., 500.)
NEAR = (760., 440., 840., 520.)
OTHER = (80., 420., 180., 500.)


class CuriousNavigation(Navigation):
    def __init__(self, camera):
        super().__init__(camera)
        self.approaches = []
        self.fixed_target = [810., 480.]

    def approach_hand(self, box, corners, source, *, stand_off_mm, authorize):
        assert authorize()
        self.approaches.append((list(box), corners, dict(source), stand_off_mm))
        self.value.update(phase='advancing', active=True)
        self.robot.update(stop_reason='navigating', navigation_active=True)
        return {'active': True, 'target_px': list(self.fixed_target), 'purpose': 'curiosity'}


@pytest.fixture
def rig():
    camera, clock = FakeCamera(), Clock()
    camera.pose(scale=800.)
    camera.update(running=True, opening=False)
    navigation = CuriousNavigation(camera)
    behavior = HomeBehavior(camera, navigation, clock=clock)
    behavior.set_home(350., 480., 1280, 960, 'usb-camera')
    behavior.set_enabled(True)
    yield camera, navigation, behavior, clock
    behavior.close()


def distant(rig, sequence, **changes):
    return sample(rig, sequence, box=FAR, **changes)


def confirm(rig):
    distant(rig, 1)
    for sequence in range(2, 22):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    return state


def test_four_seconds_of_a_far_hand_choose_a_single_fixed_point(rig):
    first = distant(rig, 1)
    assert first['phase'] == 'observing_hand' and first['far_hand_seconds'] == 0
    for sequence in range(2, 21):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert not rig[1].approaches
    rig[3].advance(.2)
    state = distant(rig, 21)
    assert state['curious'] and state['phase'] == 'curious'
    assert state['interest_target_px'] == [810., 480.]
    assert state['curiosity_count'] == 1 and state['far_hand_seconds'] == 4.
    assert len(rig[1].approaches) == 1
    box, corners, source, gap = rig[1].approaches[0]
    assert box == list(FAR) and corners == rig[0].state()['marker_corners']
    assert source == {'frame_width': 1280, 'frame_height': 960, 'camera_id': 'usb-camera'}
    assert gap == 250. and not rig[1].returns


def test_curiosity_wait_and_spacing_stay_outside_a_larger_fear_threshold(rig):
    state = rig[2].set_distance(450.)
    assert state['curiosity_distance_mm'] == 650.
    assert state['interest_stand_off_mm'] == 550.
    assert state['curiosity_wait_seconds'] == 4.


def test_repeated_reads_of_one_frame_cannot_create_four_seconds_of_presence(rig):
    distant(rig, 1)
    for _ in range(25):
        rig[3].advance(.2)
        distant(rig, 1)
    assert not rig[1].approaches and rig[2].state()['far_hand_seconds'] == 0.
    state = distant(rig, 2)
    assert state['far_hand_seconds'] == 0. and not state['curious']


def test_a_different_hand_does_not_inherit_the_previous_hands_wait(rig):
    distant(rig, 1)
    for sequence in range(2, 20):
        rig[3].advance(.2)
        distant(rig, sequence)
    rig[3].advance(.2)
    state = sample(rig, 20, box=OTHER)
    assert state['far_hand_seconds'] == 0. and not rig[1].approaches
    for sequence in range(21, 41):
        rig[3].advance(.2)
        state = sample(rig, sequence, box=OTHER)
    assert state['curious'] and rig[1].approaches[0][0] == list(OTHER)


def test_gradual_hand_motion_keeps_one_episode_and_freezes_its_final_position(rig):
    distant(rig, 1)
    for sequence in range(2, 22):
        rig[3].advance(.2)
        shift = sequence * 1.
        state = sample(rig, sequence, box=tuple(value + (shift if index % 2 == 0 else 0)
                                               for index, value in enumerate(FAR)))
    assert state['curious'] and len(rig[1].approaches) == 1
    assert rig[1].approaches[0][0] == [1101., 420., 1201., 500.]
    rig[3].advance(.2)
    sample(rig, 22, box=OTHER)
    assert len(rig[1].approaches) == 1
    assert rig[2].state()['interest_target_px'] == [810., 480.]


def test_brief_missing_detection_does_not_reset_the_same_hand_episode(rig):
    distant(rig, 1)
    for sequence in range(2, 12):
        rig[3].advance(.2)
        distant(rig, sequence)
    rig[3].advance(.2)
    distant(rig, 12, hands=[])
    assert rig[2].state()['far_hand_seconds'] == 2.
    for sequence in range(13, 22):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['curious']


def test_a_long_missing_detection_requires_a_new_four_second_episode(rig):
    distant(rig, 1)
    rig[3].advance(.6)
    distant(rig, 2, hands=[])
    rig[3].advance(.2)
    state = distant(rig, 3)
    assert state['far_hand_seconds'] == 0. and not rig[1].approaches


def test_an_intermediate_distance_is_neither_fear_nor_curiosity(rig):
    for sequence in range(30):
        sample(rig, sequence, box=(850., 420., 930., 500.))
        rig[3].advance(.2)
    assert not rig[1].approaches and not rig[1].returns
    assert not rig[2].state()['curious']


def test_low_confidence_cannot_accumulate_curiosity(rig):
    for sequence in range(30):
        distant(rig, sequence, confidence=.4)
        rig[3].advance(.2)
    assert not rig[1].approaches


def test_disappearing_hand_never_cancels_the_committed_trip(rig):
    confirm(rig)
    for sequence in range(22, 40):
        rig[3].advance(.2)
        state = distant(rig, sequence, hands=[])
    assert state['curious'] and state['phase'] == 'curious'
    assert state['interest_target_px'] == [810., 480.]
    assert len(rig[1].approaches) == 1 and rig[1].stops == 0
    assert rig[1].value['active']


@pytest.mark.parametrize('failure', ['marker', 'detector'])
def test_loss_of_marker_or_detector_does_not_discard_the_fixed_destination(rig, failure):
    confirm(rig)
    rig[3].advance(.2)
    if failure == 'marker':
        rig[0].update(visible=False, marker_corners=None)
        state = distant(rig, 22)
    else:
        state = distant(rig, 22, ready=False, status='error')
    assert state['curious'] and state['interest_target_px'] == [810., 480.]
    assert rig[1].stops == 0 and rig[1].value['active']


def test_confirmed_near_hand_interrupts_curiosity_and_returns_home(rig):
    confirm(rig)
    rig[3].advance(.2)
    first = sample(rig, 22, box=NEAR)
    assert first['curious'] and not rig[1].returns
    rig[3].advance(.2)
    state = sample(rig, 23, box=NEAR)
    assert state['returning'] and not state['curious']
    assert state['interest_target_px'] is None
    assert len(rig[1].returns) == len(rig[1].approaches) == 1


def test_curiosity_never_interrupts_a_return_home(rig):
    sample(rig, 1, box=NEAR)
    rig[3].advance(.2)
    sample(rig, 2, box=NEAR)
    for sequence in range(3, 30):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['returning'] and not state['curious'] and not rig[1].approaches


def test_arrival_waits_without_repeatedly_approaching_the_same_hand(rig):
    confirm(rig)
    rig[1].value.update(phase='arrived', active=False)
    rig[0].pose(x=810., y=480., scale=800.)
    for sequence in range(22, 55):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['phase'] == 'interest_reached' and not state['curious']
    assert len(rig[1].approaches) == 1
    assert state['interest_target_px'] == [810., 480.]


@pytest.mark.parametrize('reason', ['button_stop', 'driver_fault', 'encoder_stall', 'remote_stop'])
def test_manual_stop_and_faults_cancel_curiosity_without_rearming(rig, reason):
    confirm(rig)
    rig[1].robot.update(stop_reason=reason, resume_blocked=True)
    rig[3].advance(.2)
    state = distant(rig, 22)
    assert not state['enabled'] and not state['curious']
    assert state['interest_target_px'] is None
    assert len(rig[1].approaches) == 1


def test_disabling_stops_its_curiosity_mission(rig):
    confirm(rig)
    state = rig[2].set_enabled(False)
    assert not state['enabled'] and not state['curious'] and rig[1].stops == 1


def test_adjusting_reaction_distance_does_not_retarget_a_committed_interest(rig):
    confirm(rig)
    generation = rig[2]._generation
    state = rig[2].set_distance(200.)
    assert state['curious'] and state['interest_target_px'] == [810., 480.]
    assert rig[2]._generation == generation and rig[1].stops == 0
    assert len(rig[1].approaches) == 1


def test_enabling_an_already_enabled_reaction_keeps_its_curiosity_mission(rig):
    confirm(rig)
    generation = rig[2]._generation
    state = rig[2].set_enabled(True)
    assert state['curious'] and state['interest_target_px'] == [810., 480.]
    assert rig[2]._generation == generation and len(rig[1].approaches) == 1


def test_camera_change_cancels_curiosity_and_forgets_its_target(rig):
    confirm(rig)
    rig[0].update(camera_id='different-camera')
    rig[3].advance(.2)
    state = distant(rig, 22)
    assert not state['enabled'] and state['interest_target_px'] is None


def test_unreachable_interest_keeps_the_home_reaction_and_does_not_retry_the_same_hand(rig):
    def blocked(*args, **kwargs):
        raise HomeBehaviorError('No hay espacio para acercarse.')
    rig[1].approach_hand = blocked
    state = confirm(rig)
    assert state['enabled'] and not state['curious']
    assert 'espacio' in state['message']
    for sequence in range(22, 50):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert rig[2].state()['curiosity_count'] == 0
    rig[3].advance(.2)
    sample(rig, 50, box=NEAR)
    rig[3].advance(.2)
    state = sample(rig, 51, box=NEAR)
    assert state['returning'] and rig[1].starts == 1


def test_manual_cancel_during_interest_planning_invalidates_the_late_authorization(rig):
    entered, release = threading.Event(), threading.Event()
    def delayed(*args, authorize, **kwargs):
        entered.set()
        assert release.wait(2.)
        if not authorize():
            raise HomeBehaviorError('Cancelado.')
        rig[1].starts += 1
        return {'target_px': [810., 480.]}
    rig[1].approach_hand = delayed
    distant(rig, 1)
    for sequence in range(2, 21):
        rig[3].advance(.2)
        distant(rig, sequence)
    rig[3].advance(.2)
    worker = threading.Thread(target=lambda: distant(rig, 21))
    worker.start()
    try:
        assert entered.wait(1.)
        rig[2].cancel('D48')
    finally:
        release.set()
        worker.join(2.)
    assert not rig[2].state()['enabled'] and rig[2].state()['message'] == 'D48'
    assert rig[1].starts == 0
