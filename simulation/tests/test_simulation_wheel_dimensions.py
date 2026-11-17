"""Replacement wheel geometry and SI conversions, without running Webots."""
import json
import math
from pathlib import Path

import pytest

from simulation.adapters.ros2_bridge import RosBridgeCore
from simulation.service.config import load_configuration
from simulation.tools import build_world
from simulation.tools.build_assets import wheel


ROOT=Path(__file__).resolve().parents[2]


def test_user_wheel_dimensions_preserve_counts_mass_and_other_unmeasured_dynamics():
    robot,_=load_configuration()
    assert robot['wheel_radius_m']==.0165
    assert robot['wheel_width_m']==.007
    assert robot['track_width_m']==.163
    assert robot['track_width_m']+robot['wheel_width_m']==pytest.approx(.170)
    assert robot['encoder_counts_per_revolution']==59.94
    assert robot['mass_kg']==.18
    assert robot['wheel_mass_kg']==.012
    assert robot['friction']==.8 and robot['skid_friction']==.12
    assert '2026-11-17' in robot['provenance']['wheel_dimensions']
    assert 'approximately 170 mm' in robot['provenance']['track_width_m']
    # A 130 mm PCB still fits between the two nominal inner wheel faces.
    clearance=(robot['track_width_m']-robot['wheel_width_m']-robot['body_width_m'])/2
    assert clearance==pytest.approx(.013)


def test_wheel_mesh_has_33_by_7_envelope_without_former_wide_tread():
    robot,_=load_configuration()
    geometry=wheel(robot)
    assert max(math.hypot(x,z) for x,y,z in geometry.vertices)==pytest.approx(.0165,abs=1e-9)
    assert min(y for x,y,z in geometry.vertices)==pytest.approx(-.0035,abs=1e-9)
    assert max(y for x,y,z in geometry.vertices)==pytest.approx(.0035,abs=1e-9)
    assert 'Wheel_tire_33x7' in geometry.parts
    assert 'Silicone_tire_23x40' not in geometry.parts


def test_ros_defaults_use_same_distance_per_count_and_track_as_simulation():
    robot,_=load_configuration()
    core=RosBridgeCore()
    assert core.track==robot['track_width_m']
    assert core.count_distance==pytest.approx(math.pi*.033/59.94)
    # One full measured revolution must travel the new circumference.
    assert core.count_distance*59.94==pytest.approx(math.pi*.033)


def test_proto_only_generation_updates_wheels_without_overwriting_user_world(tmp_path,monkeypatch):
    sim=tmp_path/'simulation'
    for directory in ['config','assets/robot','assets/markers','webots/worlds']:
        (sim/directory).mkdir(parents=True,exist_ok=True)
    robot,_=load_configuration()
    (sim/'config/robot.json').write_text(json.dumps(robot))
    (sim/'assets/manifest.json').write_text(json.dumps({'wheel_radius_m':.0165,'wheel_width_m':.007}))
    for name in ['body','wheel']:
        (sim/f'assets/robot/{name}.inc.wbt').write_text('Group { children [] }')
    world=sim/'webots/worlds/robotracers.wbt'
    world.write_bytes(b'user world, camera and pose must remain untouched')
    image=sim/'assets/markers/tag_0.png';image.write_bytes(b'unchanged marker')
    monkeypatch.setattr(build_world,'SIM',sim)
    monkeypatch.setattr(build_world,'marker_texture',lambda *_: pytest.fail('Do not rebuild marker textures'))
    build_world.generate(proto_only=True)
    assert world.read_bytes()==b'user world, camera and pose must remain untouched'
    assert image.read_bytes()==b'unchanged marker'
    proto=(sim/'webots/protos/RobotRacers.proto').read_text()
    assert 'anchor 0 0.0815 0.0165' in proto
    assert 'anchor 0 -0.0815 0.0165' in proto
    assert 'Cylinder { radius 0.0165 height 0.007 subdivision 32 }' in proto
    assert proto.count('Transform { scale 1.0 1.0 1.0')==2
    assert 'translation .01315 0 0.005' in proto
