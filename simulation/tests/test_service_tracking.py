"""El seguimiento gráfico acompaña su imagen y nunca realimenta el control."""
import asyncio
import json
import threading
import time
import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient
from simulation.service.app import create_app
from simulation.service.runtime import RobotService
from simulation.service.tracking import camera_tracking
from simulation.service.vision import CameraVision
from simulation.tests.test_service_core import make_camera_scene


def sample_service(tmp_path):
    path,frame=make_camera_scene(tmp_path)
    service=RobotService(tmp_path)
    outcome=service.vision.process(frame)
    service.camera_sample=(frame,outcome,service.scene_seq)
    return service,frame,outcome


def test_tracking_box_uses_actual_tag_and_estimated_calibration_without_annotating_pixels(tmp_path):
    service,frame,outcome=sample_service(tmp_path)
    original=frame.bgra.copy()
    tracking=camera_tracking(frame,outcome,service.robot_config,service.goals,"B",time.monotonic())
    assert tracking["tracking"]=="visible"
    assert tracking["seq"]==frame.sequence and tracking["sim_time"]==frame.sim_time
    assert np.allclose(tracking["marker_corners"],outcome.marker_corners)
    x,y,_=outcome.measurement
    expected=outcome.projection.pixels([[x,y,service.robot_config["marker_height_m"]]])[0]
    assert np.allclose(tracking["robot_center"],expected)
    box=tracking["robot_box"]
    assert box["x"]<expected[0]<box["x"]+box["width"]
    assert box["y"]<expected[1]<box["y"]+box["height"]
    # La caja cubre el robot entero, incluidas las ruedas de 230 mm de ancho.
    assert max(box["width"],box["height"])>80
    assert tracking["target"]["id"]=="B" and len(tracking["goals"])==4
    assert np.array_equal(original,frame.bgra)


def test_tracking_jpeg_header_stays_paired_through_cache_and_goals_edit_without_ground_truth(tmp_path):
    service,frame,outcome=sample_service(tmp_path);service.navigation.active_goal="B"
    with TestClient(create_app(service,background=False)) as client:
        first=client.get("/api/camera.jpg");header=first.headers["X-RobotRacers-Tracking"]
        before=json.loads(header)
        assert first.status_code==200 and first.headers["content-type"]=="image/jpeg"
        assert before["seq"]==frame.sequence and before["tracking"]=="visible"
        assert len(header)<5000 and header.isascii()
        expected=cv2.imencode(".jpg",frame.bgra,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()
        assert first.content==expected
        service.change_goals([{"id":"B","x":.6,"y":.1}]);service.navigation.active_goal="B"
        edited=client.get("/api/camera.jpg");after=json.loads(edited.headers["X-RobotRacers-Tracking"])
        assert edited.content==first.content and after["seq"]==before["seq"]
        assert after["target"]["pixel"]!=before["target"]["pixel"]
        assert after["robot_center"]==before["robot_center"]
        # Ni evaluación ni una pose filtrada falsa deben mover la caja observada.
        service.receive({"type":"evaluation","sim_time":1.,"x":99.,"y":-99.,"yaw":2.},time.monotonic())
        service.estimator.correct((30.,40.,1.),time.monotonic(),time.monotonic())
        independent=json.loads(client.get("/api/camera.jpg").headers["X-RobotRacers-Tracking"])
        assert independent["robot_box"]==before["robot_box"]
        assert independent["robot_center"]==before["robot_center"]


def test_tracking_missing_or_stale_tag_has_no_box_or_invented_pose(tmp_path):
    service,frame,outcome=sample_service(tmp_path)
    path,missing=make_camera_scene(tmp_path,include_robot=False);missing.sequence=4
    lost=service.vision.process(missing)
    assert lost.calibrated and not lost.visible
    metadata=camera_tracking(missing,lost,service.robot_config,service.goals,None,time.monotonic())
    assert metadata["tracking"]=="lost"
    assert metadata["robot_box"] is metadata["robot_center"] is metadata["marker_corners"] is None
    assert len(metadata["goals"])==4 and metadata["target"] is None
    stale=camera_tracking(frame,outcome,service.robot_config,service.goals,None,frame.capture_stamp+1.)
    assert stale["tracking"]=="lost" and stale["robot_box"] is None
    assert stale["robot_center"] is None and stale["marker_corners"] is None
    assert np.isfinite(np.array([g["pixel"] for g in stale["goals"]])).all()


def test_tracking_camera_loop_does_not_mix_inflight_frame_with_previous_outcome(tmp_path):
    async def scenario():
        service,first,outcome1=sample_service(tmp_path)
        _,second=make_camera_scene(tmp_path,robot_pose=(-.2,.25,-1.))
        second.sequence=4;second.sim_time=2.
        outcome2=CameraVision(tmp_path/"camera_calibration.json").process(second)
        frames=iter([first,second]);processing=threading.Event();release=threading.Event()
        class Reader:
            def read(self):return next(frames,None)
        def detector(frame):
            if frame.sequence==4:processing.set();release.wait(2);return outcome2
            return outcome1
        service.reader=Reader();service.vision.process=detector
        task=asyncio.create_task(service.camera_loop())
        try:
            for _ in range(200):
                if processing.is_set():break
                await asyncio.sleep(.001)
            assert processing.is_set() and service.last_frame.sequence==4
            jpeg,tracking=service.camera_packet()
            assert tracking["seq"]==2
            assert jpeg==cv2.imencode(".jpg",first.bgra,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()
            release.set()
            for _ in range(200):
                if service.camera_sample[0].sequence==4:break
                await asyncio.sleep(.001)
            service.jpeg_stamp=0.
            jpeg,tracking=service.camera_packet()
            assert tracking["seq"]==4 and tracking["sim_time"]==2.
            assert np.allclose(tracking["marker_corners"],outcome2.marker_corners)
            assert jpeg==cv2.imencode(".jpg",second.bgra,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()
        finally:
            release.set();task.cancel();await asyncio.gather(task,return_exceptions=True)
    asyncio.run(scenario())


def test_tracking_reset_does_not_serve_cached_old_world(tmp_path):
    service,frame,outcome=sample_service(tmp_path)
    assert service.camera_packet() is not None
    service.scene_command("reset")
    assert service.camera_packet() is None
    with TestClient(create_app(service,background=False)) as client:
        assert client.get("/api/camera.jpg").status_code==503
