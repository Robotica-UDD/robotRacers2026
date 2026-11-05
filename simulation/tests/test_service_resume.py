"""Restaurar una sesión conserva intenciones sin restaurar una pose como sensor."""
import json
import time
import pytest
from simulation.service.runtime import RobotService
from simulation.tests.test_service_core import telemetry


def checkpoint_payload(service):
    payload=service.state()
    payload["status"]="paused"
    payload["settings"].update(scale=1.5,speed=1.5,mass_kg=.21)
    payload["mission"]["goals"]=[{"id":"A","x":-.9,"y":.3},{"id":"B","x":.9,"y":.6},
        {"id":"C","x":-.45,"y":0.},{"id":"D","x":.3,"y":0.}]
    payload["mission"]["active_goal"]="C"
    payload["pose"]={"x":99.,"y":-99.,"yaw":2.8,"source":"persisted"}
    return payload


@pytest.mark.parametrize("saved_status",["paused","running"])
def test_resume_configuration_is_read_only_and_pose_does_not_localize(tmp_path,monkeypatch,saved_status):
    defaults=RobotService(tmp_path/"defaults")
    payload=checkpoint_payload(defaults);payload["status"]=saved_status
    source=tmp_path/"public-state.json";source.write_text(json.dumps(payload));before=source.read_bytes()
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(source))
    runtime=tmp_path/"restored";service=RobotService(runtime)
    assert service.settings==payload["settings"]
    assert [g["id"] for g in service.goals]==[g["id"] for g in payload["mission"]["goals"]]
    for restored,saved in zip(service.goals,payload["mission"]["goals"]):
        assert restored["x"]==pytest.approx(saved["x"])
        assert restored["y"]==pytest.approx(saved["y"])
    assert service.arena.scale==1.5
    assert service.status=="disconnected" and not service.estimator.initialized
    assert service.navigation.waypoints==[] and service.scene_pending is None
    assert not runtime.exists() and source.read_bytes()==before
    now=time.monotonic();service.receive(telemetry(now),now)
    assert service.status=="paused" and not service.estimator.initialized
    assert service.state()["mission"]["active_goal"]=="C"
    assert service.state()["mission"]["progress"]==0.
    assert service.sent_command.v==service.sent_command.w==0.
    with pytest.raises(ValueError,match="cámara"):service.mission_start()
    assert service.estimator.correct((-.1288,0.,.2),now,now)
    assert service.estimator.x[0]==pytest.approx(-.1288)
    assert service.estimator.x[1]==pytest.approx(0.)
    assert service.status=="paused" and source.read_bytes()==before
    service.mission_start()
    assert service.status=="running" and service.resume_active_goal is None


@pytest.mark.parametrize("action",["goals","reset"])
def test_resume_active_goal_is_display_only_and_cleared_by_new_intent(tmp_path,monkeypatch,action):
    payload=checkpoint_payload(RobotService(tmp_path/"defaults"))
    source=tmp_path/"state.json";source.write_text(json.dumps(payload))
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(source))
    service=RobotService(tmp_path/"runtime");now=time.monotonic();service.receive(telemetry(now),now)
    assert service.display_active_goal()=="C" and not service.navigation.waypoints
    if action=="goals":service.change_goals([{"id":"B","x":.9,"y":.4}])
    else:service.scene_command("reset")
    assert service.resume_active_goal is None and service.display_active_goal() is None


@pytest.mark.parametrize("invalid",["mass","goal","duplicate","unknown","json"])
def test_resume_invalid_document_does_not_apply_partial_settings(tmp_path,monkeypatch,invalid):
    defaults=RobotService(tmp_path/"defaults");payload=checkpoint_payload(defaults)
    if invalid=="mass":payload["settings"]["mass_kg"]=-1.
    elif invalid=="goal":payload["mission"]["goals"][0]["x"]=float("nan")
    elif invalid=="duplicate":payload["mission"]["goals"][1]["id"]="A"
    elif invalid=="unknown":payload["settings"]["unknown"]=1.
    source=tmp_path/"state.json";source.write_text("{" if invalid=="json" else json.dumps(payload))
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(source))
    service=RobotService(tmp_path/"restored")
    assert service.settings==defaults.settings and service.goals==defaults.goals
    assert service.resume_warning and not service.resume_paused
    assert not service.estimator.initialized and service.scene_pending is None


def test_resume_missing_file_keeps_defaults(tmp_path,monkeypatch):
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(tmp_path/"missing.json"))
    service=RobotService(tmp_path/"runtime")
    assert service.status=="disconnected" and service.settings["scale"]==1.
    assert service.resume_warning and not (tmp_path/"runtime").exists()


@pytest.mark.parametrize("driver_restart",[False,True])
def test_resume_pause_and_active_goal_survive_telemetry_disconnect(tmp_path,monkeypatch,driver_restart):
    payload=checkpoint_payload(RobotService(tmp_path/"defaults"));payload["mission"]["active_goal"]="A"
    source=tmp_path/"state.json";source.write_text(json.dumps(payload))
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(source))
    service=RobotService(tmp_path/"runtime");now=time.monotonic()
    service.receive(telemetry(now,seq=100),now)
    assert service.status=="paused" and service.display_active_goal()=="A" and not service.resume_paused
    service.tick(now+.301)
    assert service.status=="disconnected" and service.resume_paused
    assert service.sent_command.mode=="stop" and service.resume_active_goal=="A"
    # Varios ticks desconectados no deben borrar la intención de pausa.
    service.tick(now+.35)
    recovered=now+(1.1 if driver_restart else .36)
    service.receive(telemetry(recovered,seq=1 if driver_restart else 101),recovered)
    assert service.status=="paused" and service.state(recovered)["mission"]["active_goal"]=="A"
    assert not service.resume_paused and not service.navigation.waypoints
    assert not service.estimator.initialized


@pytest.mark.parametrize("action",["stop","reset"])
def test_resume_stop_or_reset_clear_reconnection_pause_intention(tmp_path,monkeypatch,action):
    payload=checkpoint_payload(RobotService(tmp_path/"defaults"))
    source=tmp_path/"state.json";source.write_text(json.dumps(payload))
    monkeypatch.setenv("ROBOTRACERS_RESUME_STATE",str(source))
    service=RobotService(tmp_path/"runtime");now=time.monotonic();service.receive(telemetry(now),now)
    service.tick(now+.31)
    assert service.resume_paused
    if action=="stop":service.mission_stop()
    else:service.scene_command("reset")
    assert not service.resume_paused
