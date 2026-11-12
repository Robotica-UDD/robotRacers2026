"""Curiosidad: contador, destino fijo y cancelación, sin cámara ni motores."""
from simulation.tests.test_webcam_home_ui import PAGE, run


def test_far_hand_counter_and_curiosity_states_are_visible_without_extra_controls():
    run(r"""
homeState.enabled=true;homeState.phase='observing_hand';
homeState.far_hand_seconds=1.7;homeState.curiosity_wait_seconds=2;
renderHomeUI();
assert.equal($('home-status').textContent,'Observando una mano · 1,7 / 2 s');
assert.equal($('home-cancel').hidden,true);
homeState.phase='curious';homeState.curious=true;
homeState.message='Me acerco al punto que elegí.';
renderHomeUI();
assert.equal($('home-status').textContent,'Acercándose por curiosidad');
assert.equal($('home-message').textContent,'Me acerco al punto que elegí.');
assert.equal($('home-cancel').hidden,false);
assert.equal($('home-cancel').textContent,'Detener acercamiento');
assert.equal($('home-distance').disabled,true);
homeState.phase='interest_reached';homeState.curious=false;
renderHomeUI();
assert.equal($('home-status').textContent,'Punto de interés alcanzado');
assert.equal($('home-cancel').hidden,true);
""")
    html = PAGE.read_text()
    assert 'Una mano lejana durante 2 s despierta curiosidad.' in html
    assert 'Elige un punto a unos 10 cm y mantiene el destino aunque la mano desaparezca.' in html
    assert 'Configuración probada: avance 240 mm/s · giro máximo 90°/s.' in html


def test_observation_counter_is_bounded_and_has_a_valid_two_second_default():
    run(r"""
homeState.enabled=true;homeState.phase='observing_hand';
homeState.far_hand_seconds=9;homeState.curiosity_wait_seconds=2;
renderHomeUI();
assert.equal($('home-status').textContent,'Observando una mano · 2,0 / 2 s');
homeState.far_hand_seconds=NaN;homeState.curiosity_wait_seconds=Infinity;
renderHomeUI();
assert.equal($('home-status').textContent,'Observando una mano · 0,0 / 2 s');
""")


def test_detector_labels_use_the_active_model_and_default_to_mediapipe():
    run(r"""
homeState.detector={status:'ready'};
renderHomeUI();
assert.equal($('home-model').textContent,'MediaPipe · detección de manos lista');
homeState.detector.model=' MediaPipe Hand Landmarker ';
renderHomeUI();
assert.equal($('home-model').textContent,'MediaPipe Hand Landmarker · detección de manos lista');
for(const [status,label] of Object.entries({loading:'cargando detección de manos…',missing_model:'falta instalar el modelo de manos',error:'detector no disponible',closed:'detección detenida'})){
  homeState.detector.status=status;
  renderHomeUI();
  assert.equal($('home-model').textContent,`MediaPipe Hand Landmarker · ${label}`);
}
homeState.detector={status:'ready',model:' '};
renderHomeUI();
assert.equal($('home-model').textContent,'MediaPipe · detección de manos lista');
homeAvailable=false;renderHomeUI();
assert.equal($('home-model').textContent,'Esperando el servicio de casa y manos.');
""")
    assert 'YOLO ·' not in PAGE.read_text()


def test_interest_marker_stays_fixed_when_the_hand_moves_or_disappears():
    run(r"""
homeState.enabled=true;homeState.phase='curious';homeState.curious=true;
homeState.interest_target_px=[850,240];
assert.equal(drawInterestOverlay(2),true);
const marker=$('navigation-interest');
assert.equal(marker.attributes.transform,'translate(850 240)');
assert.equal($('navigation-interest-circle').attributes.r,'5');
assert.equal('hidden' in marker.attributes,false);
homeState.detector.hands=[{bbox_px:[950,150,1050,250]}];
drawInterestOverlay(2);
assert.equal(marker.attributes.transform,'translate(850 240)');
homeState.detector.hands=[];
drawInterestOverlay(2);
assert.equal(marker.attributes.transform,'translate(850 240)');
assert.equal('hidden' in marker.attributes,false);
homeState.curious=false;homeState.phase='interest_reached';
assert.equal(drawInterestOverlay(2),true);
""", extra=('drawInterestOverlay',))
    assert '>INTERÉS</text>' in PAGE.read_text()


def test_interest_marker_requires_matching_camera_and_a_valid_in_image_point():
    run(r"""
homeState.enabled=true;homeState.phase='curious';homeState.curious=true;
homeState.interest_target_px=[850,240];
assert.equal(drawInterestOverlay(1),true);
state.camera_id='other';assert.equal(drawInterestOverlay(1),false);
state.camera_id='usb';state.frame_height=960;assert.equal(drawInterestOverlay(1),false);
state.frame_height=720;
for(const point of [[NaN,200],[-1,200],[1280,200],[100,720],null]){
  homeState.interest_target_px=point;
  assert.equal(drawInterestOverlay(1),false);
  assert.equal('hidden' in $('navigation-interest').attributes,true);
}
homeState.interest_target_px=[850,240];homeState.source=null;
assert.equal(drawInterestOverlay(1),false);
homeState.source={camera_id:'usb',frame_width:1280,frame_height:720};
homeAvailable=false;assert.equal(drawInterestOverlay(1),false);
""", extra=('drawInterestOverlay',))


def test_near_hand_return_has_priority_and_hides_the_interest_marker():
    run(r"""
homeState.enabled=true;homeState.curious=true;homeState.returning=true;
homeState.phase='returning';homeState.interest_target_px=[850,240];
renderHomeUI();
assert.equal($('home-status').textContent,'Volviendo a casa');
assert.equal($('home-cancel').textContent,'Detener regreso a casa');
assert.equal(drawInterestOverlay(1),false);
homeState.returning=false;homeState.curious=false;homeState.phase='disabled';
assert.equal(drawInterestOverlay(1),false);
""", extra=('drawInterestOverlay',))


def test_curiosity_cancel_invalidates_pending_navigation_even_if_hand_and_link_are_lost():
    run(r"""
const requests=[];
api=async(path,options)=>{
  requests.push([path,JSON.parse(options.body),options.headers]);
  return {...homeState,enabled:false,curious:false,phase:'disabled'};
};
homeState.enabled=true;homeState.curious=true;homeState.phase='waiting_connection';
homeState.returning=false;homeState.detector.hands=[];
robotState.connected=false;navigating=false;
renderHomeUI();
assert.equal($('home-cancel').hidden,false);
assert.equal($('home-cancel').disabled,false);
await cancelHomeReturn();
assert.equal(requests.length,1);
assert.equal(requests[0][0],'/api/home/settings');
assert.deepEqual(requests[0][1],{enabled:false});
assert.equal(requests[0][2]['X-Requested-With'],'XMLHttpRequest');
assert.equal(homeState.curious,false);
""")
