"""Curiosidad por una mano lejana: destino fijo y prioridad del refugio."""
import threading

import pytest

from simulation.tests.test_webcam_home_behavior import Clock, Navigation, sample
from simulation.tests.test_webcam_point_navigation import FakeCamera
from simulation.webcam.home_behavior import HomeBehavior, HomeBehaviorError, hand_clearance_mm


FAR = (1080., 420., 1180., 500.)
NEAR = (760., 440., 840., 520.)
OTHER = (80., 420., 180., 500.)


class CuriousNavigation(Navigation):
    def __init__(self, camera):
        super().__init__(camera)
        self.approaches = []
        self.fixed_target = [930., 480.]

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
    for sequence in range(2, 12):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    return state


def test_two_seconds_of_a_far_hand_choose_a_single_fixed_point(rig):
    first = distant(rig, 1)
    assert first['phase'] == 'observing_hand' and first['far_hand_seconds'] == 0
    for sequence in range(2, 11):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert not rig[1].approaches
    rig[3].advance(.2)
    state = distant(rig, 11)
    assert state['curious'] and state['phase'] == 'curious'
    assert state['interest_target_px'] == [930., 480.]
    assert state['curiosity_count'] == 1 and state['far_hand_seconds'] == 2.
    assert len(rig[1].approaches) == 1
    box, corners, source, gap = rig[1].approaches[0]
    assert box == list(FAR) and corners == rig[0].state()['marker_corners']
    assert source == {'frame_width': 1280, 'frame_height': 960, 'camera_id': 'usb-camera'}
    assert gap == 100. and not rig[1].returns


def test_curiosity_wait_and_spacing_stay_outside_a_larger_fear_threshold(rig):
    state = rig[2].set_distance(450.)
    assert state['curiosity_distance_mm'] == 500.
    assert state['interest_stand_off_mm'] == 100.
    assert state['curiosity_wait_seconds'] == 2.


def test_a_stationary_hand_more_than_one_meter_away_still_confirms_curiosity_in_two_seconds(rig):
    # La misma imagen cubre más suelo: la mano está a más de 1 m de la huella.
    rig[0].pose(scale=300.)
    first = distant(rig, 1)
    assert first['nearest_hand_mm'] > 1000.
    assert first['curiosity_status'] == 'confirming'
    for sequence in range(2, 12):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['far_hand_seconds'] == 2. and state['curious']
    assert state['curiosity_attempt_count'] == state['curiosity_count'] == 1
    assert len(rig[1].approaches) == 1 and not rig[1].returns


def test_a_farther_than_one_meter_hand_reports_route_retries_instead_of_missing_detection(rig):
    rig[0].pose(scale=300.)
    attempts = []
    def blocked(*args, **kwargs):
        attempts.append(1)
        raise HomeBehaviorError('El punto de acercamiento no tiene un camino disponible.')
    rig[1].approach_hand = blocked
    state = confirm(rig)
    assert state['nearest_hand_mm'] > 1000. and state['far_hand_seconds'] == 2.
    assert state['curiosity_status'] == 'retrying'
    assert state['curiosity_attempt_count'] == 1 and state['curiosity_count'] == 0
    assert 'camino' in state['curiosity_blocked_reason']
    for sequence in range(12, 22):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['curiosity_attempt_count'] == len(attempts) == 2
    assert state['curiosity_status'] == 'retrying' and state['enabled']


def test_a_hand_box_at_120mm_returns_home_even_when_its_palm_is_farther_than_200mm(rig):
    # Caso observado: los dedos están cerca aunque la palma parezca lejana.
    hands = [{'bbox_px': [803., 420., 1000., 520.],
              'palm_px': [926., 460.], 'confidence': 1.}]
    palm_gap = hand_clearance_mm([925.9, 459.9, 926.1, 460.1],
                                rig[0].state()['marker_corners'], rig[1]._calibration)
    assert 250. < palm_gap < 300.
    state = distant(rig, 1, hands=hands)
    assert 119. < state['nearest_hand_mm'] < 122.
    assert state['near_hand'] and not state['curious']
    rig[3].advance(.2)
    state = distant(rig, 2, hands=hands)
    assert state['returning'] and state['curiosity_status'] == 'returning'
    assert state['far_hand_seconds'] == 0. and state['curiosity_attempt_count'] == 0
    assert len(rig[1].returns) == 1 and not rig[1].approaches


def test_repeated_reads_of_one_frame_cannot_create_two_seconds_of_presence(rig):
    distant(rig, 1)
    for _ in range(25):
        rig[3].advance(.2)
        distant(rig, 1)
    assert not rig[1].approaches and rig[2].state()['far_hand_seconds'] == 0.
    state = distant(rig, 2)
    assert state['far_hand_seconds'] == 0. and not state['curious']


def test_a_different_hand_does_not_inherit_the_previous_hands_wait(rig):
    distant(rig, 1)
    for sequence in range(2, 10):
        rig[3].advance(.2)
        distant(rig, sequence)
    rig[3].advance(.2)
    state = sample(rig, 10, box=OTHER)
    assert state['far_hand_seconds'] == 0. and not rig[1].approaches
    for sequence in range(11, 21):
        rig[3].advance(.2)
        state = sample(rig, sequence, box=OTHER)
    assert state['curious'] and rig[1].approaches[0][0] == list(OTHER)


def test_gradual_hand_motion_keeps_one_episode_and_freezes_its_final_position(rig):
    distant(rig, 1)
    for sequence in range(2, 12):
        rig[3].advance(.2)
        shift = sequence * 1.
        state = sample(rig, sequence, box=tuple(value + (shift if index % 2 == 0 else 0)
                                               for index, value in enumerate(FAR)))
    assert state['curious'] and len(rig[1].approaches) == 1
    assert rig[1].approaches[0][0] == [1091., 420., 1191., 500.]
    rig[3].advance(.2)
    sample(rig, 22, box=OTHER)
    assert len(rig[1].approaches) == 1
    assert rig[2].state()['interest_target_px'] == [930., 480.]


def test_one_second_missing_detection_pauses_without_resetting_or_counting_absence(rig):
    distant(rig, 1)
    for sequence in range(2, 7):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert rig[2].state()['far_hand_seconds'] == 1.
    for sequence in range(7, 11):
        rig[3].advance(.2)
        distant(rig, sequence, hands=[])
    rig[3].advance(.2)
    state = distant(rig, 11)
    assert state['far_hand_seconds'] == 1. and not rig[1].approaches
    for sequence in range(12, 17):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['curious'] and state['far_hand_seconds'] == 2.


def test_a_long_missing_detection_requires_a_new_two_second_episode(rig):
    distant(rig, 1)
    rig[3].advance(.2)
    distant(rig, 2)
    rig[3].advance(1.6)
    distant(rig, 3, hands=[])
    rig[3].advance(.2)
    state = distant(rig, 4)
    assert state['far_hand_seconds'] == 0. and not rig[1].approaches


def test_a_large_change_in_box_size_does_not_reset_the_same_hand(rig):
    distant(rig, 1)
    large = (1005., 335., 1255., 585.)
    for sequence in range(2, 12):
        rig[3].advance(.2)
        state = sample(rig, sequence, box=large if sequence % 2 else FAR)
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert len(rig[1].approaches) == 1


def test_palm_position_is_preferred_over_box_center_for_hand_continuity(rig):
    # Los dedos cambian la caja; la palma conserva la misma posición.
    boxes = ((920., 300., 1180., 620.), (1100., 420., 1180., 500.))
    for sequence in range(1, 12):
        state = distant(rig, sequence, hands=[{'bbox_px': list(boxes[sequence % 2]),
                                             'palm_px': [1130., 460.], 'confidence': 1.}])
        rig[3].advance(.2)
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert rig[2]._attractor_anchor == [1130., 460.]


@pytest.mark.parametrize('failure', ['marker', 'detector'])
def test_brief_loss_counts_only_fresh_visible_hands_even_when_the_marker_is_missing(rig, failure):
    distant(rig, 1)
    rig[3].advance(.2)
    distant(rig, 2)
    rig[3].advance(.2)
    if failure == 'marker':
        rig[0].update(visible=False)
        distant(rig, 3)
        rig[0].update(visible=True)
    else:
        distant(rig, 3, ready=False)
    rig[3].advance(.6)
    state = distant(rig, 4)
    expected = .4 if failure == 'marker' else .2
    assert state['far_hand_seconds'] == expected and not state['curious']
    for sequence in range(5, 14):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['curious'] and state['far_hand_seconds'] == 2.


def test_a_gap_without_any_results_does_not_invent_hand_presence(rig):
    distant(rig, 1)
    rig[3].advance(.2)
    distant(rig, 2)
    rig[3].advance(.8)
    state = distant(rig, 3)
    assert state['far_hand_seconds'] == .2 and not state['curious']


def test_an_intermediate_distance_is_neither_fear_nor_curiosity(rig):
    for sequence in range(30):
        sample(rig, sequence, box=(850., 420., 930., 500.))
        rig[3].advance(.2)
    assert not rig[1].approaches and not rig[1].returns
    assert not rig[2].state()['curious']


def test_the_gap_between_fear_and_curiosity_is_reported_instead_of_silently_watching(rig):
    state = sample(rig, 1, box=(850., 420., 930., 500.))
    assert state['curiosity_status'] == 'too_close'
    assert state['curiosity_distance_mm'] == 200.
    assert 150. < state['nearest_hand_mm'] < 200.
    assert '200' in state['message'] and state['far_hand_seconds'] == 0.


def test_threshold_jitter_does_not_reset_a_hand_already_being_confirmed(rig):
    for sequence in range(1, 12):
        # Comienza por encima de 200 mm y oscila a 192 mm, dentro de la histéresis.
        left = 870. if sequence % 2 else 860.
        state = sample(rig, sequence, box=(left, 420., left + 80., 500.))
        rig[3].advance(.2)
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert len(rig[1].approaches) == 1


def test_apriltag_flicker_does_not_freeze_the_counter_of_a_continuously_visible_hand(rig):
    distant(rig, 1)
    for sequence in range(2, 22):
        rig[3].advance(.1)
        rig[0].update(visible=bool(sequence % 2))
        state = distant(rig, sequence)
        if sequence % 2 == 0:
            assert state['curiosity_status'] == 'waiting_vision'
            assert not rig[1].approaches
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert len(rig[1].approaches) == 1


def test_visible_hand_can_finish_confirmation_without_marker_but_trip_waits_for_a_current_pose(rig):
    distant(rig, 1)
    rig[0].update(visible=False, marker_corners=None)
    for sequence in range(2, 22):
        rig[3].advance(.1)
        state = distant(rig, sequence)
    assert state['far_hand_seconds'] == 2. and state['curiosity_status'] == 'waiting_vision'
    assert not rig[1].approaches
    rig[0].pose(scale=800.)
    rig[0].update(visible=True)
    rig[3].advance(.1)
    state = distant(rig, 22)
    assert state['curious'] and len(rig[1].approaches) == 1


def test_hand_absence_during_marker_loss_does_not_count_as_visible_presence(rig):
    distant(rig, 1)
    rig[3].advance(.2)
    distant(rig, 2)
    rig[0].update(visible=False, marker_corners=None)
    for sequence in range(3, 7):
        rig[3].advance(.2)
        distant(rig, sequence, hands=[])
    rig[0].pose(scale=800.)
    rig[0].update(visible=True)
    rig[3].advance(.2)
    state = distant(rig, 7)
    assert state['far_hand_seconds'] == .2 and not rig[1].approaches


def test_invalid_detector_polls_between_fresh_results_do_not_freeze_visible_presence(rig):
    distant(rig, 1)
    for sequence in range(2, 22):
        rig[3].advance(.1)
        state = distant(rig, sequence, ready=bool(sequence % 2))
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert len(rig[1].approaches) == 1


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
    assert state['interest_target_px'] == [930., 480.]
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
    assert state['curious'] and state['interest_target_px'] == [930., 480.]
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
    rig[0].pose(x=930., y=480., scale=800.)
    for sequence in range(22, 55):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['phase'] == 'interest_reached' and not state['curious']
    assert len(rig[1].approaches) == 1
    assert state['interest_target_px'] == [930., 480.]
    assert 0. < state['nearest_hand_mm'] < state['distance_mm']
    assert not state['near_hand'] and not rig[1].returns


def test_approaching_the_stationary_attractor_does_not_trigger_fear(rig):
    confirm(rig)
    rig[0].pose(x=930., y=480., scale=800.)
    for sequence in range(12, 24):
        rig[3].advance(.2)
        state = distant(rig, sequence)
    assert state['curious'] and not state['returning']
    assert 0. < state['nearest_hand_mm'] < state['distance_mm']
    assert not state['near_hand'] and not rig[1].returns
    assert len(rig[1].approaches) == 1


@pytest.mark.parametrize('arrived', [False, True])
def test_another_near_hand_can_interrupt_the_stationary_attractor_exemption(rig, arrived):
    confirm(rig)
    rig[0].pose(x=930., y=480., scale=800.)
    if arrived:
        rig[1].value.update(phase='arrived', active=False)
    hands = [{'bbox_px': list(FAR), 'confidence': 1.},
             {'bbox_px': list(NEAR), 'confidence': 1.}]
    for sequence in (12, 13):
        rig[3].advance(.2)
        state = distant(rig, sequence, hands=hands)
    assert state['returning'] and len(rig[1].returns) == 1


def test_the_attractor_invading_the_footprint_triggers_home_even_if_the_palm_is_stationary(rig):
    confirm(rig)
    rig[0].pose(x=930., y=480., scale=800.)
    # Una mano que ocupa la huella nunca recibe la excepción de curiosidad.
    hands = [{'bbox_px': [980., 420., 1180., 500.],
              'palm_px': [1130., 460.], 'confidence': 1.}]
    for sequence in (12, 13):
        rig[3].advance(.2)
        state = distant(rig, sequence, hands=hands)
    assert state['returning'] and state['nearest_hand_mm'] == 0.


def test_an_attractor_that_moves_towards_the_robot_loses_its_exception(rig):
    confirm(rig)
    # La mano se desplaza más de 120 mm hacia el robot y ahora está cerca.
    moved = (780., 420., 880., 500.)
    for sequence in (12, 13):
        rig[3].advance(.2)
        state = sample(rig, sequence, box=moved)
    assert state['returning'] and not state['curious']
    assert len(rig[1].returns) == 1


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
    assert state['curious'] and state['interest_target_px'] == [930., 480.]
    assert rig[2]._generation == generation and rig[1].stops == 0
    assert len(rig[1].approaches) == 1


def test_enabling_an_already_enabled_reaction_keeps_its_curiosity_mission(rig):
    confirm(rig)
    generation = rig[2]._generation
    state = rig[2].set_enabled(True)
    assert state['curious'] and state['interest_target_px'] == [930., 480.]
    assert rig[2]._generation == generation and len(rig[1].approaches) == 1


def test_camera_change_cancels_curiosity_and_forgets_its_target(rig):
    confirm(rig)
    rig[0].update(camera_id='different-camera')
    rig[3].advance(.2)
    state = distant(rig, 22)
    assert not state['enabled'] and state['interest_target_px'] is None


def test_unreachable_interest_keeps_home_available_and_limits_retries_without_hiding_the_reason(rig):
    attempts = []
    def blocked(*args, **kwargs):
        attempts.append(rig[3].now)
        raise HomeBehaviorError('No hay espacio para acercarse.')
    rig[1].approach_hand = blocked
    state = confirm(rig)
    assert state['enabled'] and not state['curious']
    assert 'espacio' in state['curiosity_blocked_reason']
    assert state['curiosity_status'] == 'retrying' and state['curiosity_retry_in_s'] == 2.
    assert not rig[2]._interest_latched
    for sequence in range(12, 41):
        rig[3].advance(.2)
        state = distant(rig, sequence)
        assert state['curiosity_status'] == 'retrying'
        assert state['curiosity_blocked_reason'] == 'No hay espacio para acercarse.'
    assert len(attempts) == 3 and all(b - a >= 1.99 for a, b in zip(attempts, attempts[1:]))
    assert rig[2].state()['curiosity_count'] == 0
    rig[3].advance(.2)
    sample(rig, 50, box=NEAR)
    rig[3].advance(.2)
    state = sample(rig, 51, box=NEAR)
    assert state['returning'] and rig[1].starts == 1


def test_the_same_stationary_hand_retries_after_a_transient_failure_and_then_uses_one_fixed_goal(rig):
    original = rig[1].approach_hand
    attempts = []
    def transient(*args, **kwargs):
        attempts.append(rig[3].now)
        if len(attempts) == 1:
            raise HomeBehaviorError('El cálculo anterior agotó el tiempo disponible.')
        return original(*args, **kwargs)
    rig[1].approach_hand = transient
    state = confirm(rig)
    assert state['curiosity_status'] == 'retrying' and state['curiosity_attempt_count'] == 1
    assert state['curiosity_blocked_reason']
    for sequence in range(12, 21):
        rig[3].advance(.2)
        state = distant(rig, sequence)
        assert len(attempts) == 1 and state['curiosity_blocked_reason']
    rig[3].advance(.2)
    state = distant(rig, 21)
    assert state['curious'] and state['curiosity_attempt_count'] == 2
    assert state['curiosity_count'] == 1 and state['curiosity_blocked_reason'] is None
    assert state['curiosity_retry_in_s'] == 0. and state['interest_target_px'] == [930., 480.]
    for sequence in range(22, 50):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert len(attempts) == 2 and len(rig[1].approaches) == 1


def test_a_manual_stop_during_retry_cancels_the_pending_approach_and_does_not_rearm(rig):
    attempts = []
    def blocked(*args, **kwargs):
        attempts.append(1)
        raise HomeBehaviorError('No hay espacio.')
    rig[1].approach_hand = blocked
    confirm(rig)
    rig[1].robot.update(stop_reason='button_stop', resume_blocked=True)
    rig[3].advance(.2)
    state = distant(rig, 12)
    assert not state['enabled'] and state['curiosity_status'] == 'disabled'
    for sequence in range(13, 35):
        rig[3].advance(.2)
        distant(rig, sequence)
    assert len(attempts) == 1 and rig[1].stops == 0


def test_manual_cancel_during_interest_planning_invalidates_the_late_authorization(rig):
    entered, release = threading.Event(), threading.Event()
    def delayed(*args, authorize, **kwargs):
        entered.set()
        assert release.wait(2.)
        if not authorize():
            raise HomeBehaviorError('Cancelado.')
        rig[1].starts += 1
        return {'target_px': [930., 480.]}
    rig[1].approach_hand = delayed
    distant(rig, 1)
    for sequence in range(2, 11):
        rig[3].advance(.2)
        distant(rig, sequence)
    rig[3].advance(.2)
    worker = threading.Thread(target=lambda: distant(rig, 11))
    worker.start()
    try:
        assert entered.wait(1.)
        rig[2].cancel('D48')
    finally:
        release.set()
        worker.join(2.)
    assert not rig[2].state()['enabled'] and rig[2].state()['message'] == 'D48'
    assert rig[1].starts == 0
