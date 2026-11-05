import asyncio
import json
import math
from pathlib import Path
import struct
import time
import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient
from simulation.service.app import create_app
from simulation.service.config import load_configuration
from simulation.service.geometry import Arena,Point
from simulation.service.estimator import PoseEstimator,wrap
from simulation.service.navigation import Navigation
from simulation.service.runtime import RobotService,DatagramReceiver
from simulation.service.vision import HEADER,CameraReader,CameraVision,Frame


def telemetry(now=None,**changes):
    return {"type":"telemetry","seq":1,"sim_time":1.,"stamp":time.monotonic() if now is None else now,
        "left_count":0,"right_count":0,"left_velocity":0.,"right_velocity":0.,"left_pwm":0.,"right_pwm":0.,
        "battery_voltage":7.4,"reflectance":[],**changes}


def test_service_astar_avoids_diagonal_through_walls():
    _,config=load_configuration();arena=Arena(config)
    route=arena.route(arena.nodes["A"],arena.nodes["B"])
    assert route==[arena.nodes[key] for key in ["A","L","R","B"]]
    same=arena.route(Point(-.6,.45),Point(-.6,.1))
    assert len(same)==2 and same[0].x==-.6 and same[0].y==pytest.approx(.45) and same[1].y==pytest.approx(.1)
    assert arena.projection(Point(1.,.3))[0]==Point(.6,.3)


def test_service_encoder_prediction_and_vision_rejection():
    robot,_=load_configuration();estimator=PoseEstimator(robot);now=time.monotonic()
    estimator.predict(0,0,now)
    assert not estimator.initialized
    assert estimator.correct((0,0,0),now,now)
    estimator.predict(10,10,now+.1)
    assert estimator.x[0]==pytest.approx(10*estimator.count_distance)
    assert estimator.x[1]==0
    assert not estimator.correct((50,50,1),now+.1,now+.1)
    assert estimator.rejected_vision==1
    assert not estimator.correct((0,0,0),now-1,now)


def test_service_ground_truth_cannot_localize_or_steer(tmp_path):
    service=RobotService(tmp_path);now=time.monotonic()
    service.receive(telemetry(now),now)
    service.receive({"type":"evaluation","sim_time":1,"x":999,"y":-999,"yaw":2,"contacts":4},now)
    assert not service.estimator.initialized
    with pytest.raises(ValueError):service.mission_start()
    service.estimator.correct((.1,.2,.3),now,now)
    before=service.estimator.x.copy();covariance=service.estimator.P.copy()
    service.record_estimate(now)
    service.receive({"type":"evaluation","sim_time":1,"x":300,"y":400,"yaw":-1,"contacts":1},now)
    assert np.array_equal(service.estimator.x,before)
    assert np.array_equal(service.estimator.P,covariance)
    assert service.state(now)["metrics"]["position_error_m"]>400


def test_service_shared_frame_seqlock_and_resolution(tmp_path):
    file=tmp_path/"camera.frame";reader=CameraReader(file)
    def write(width,height,seq):
        file.write_bytes(HEADER.pack(b"RRFRAME1",width,height,4,0,seq,2.,time.monotonic())+bytes([10,20,30,255])*(width*height))
    write(16,16,1);assert reader.read() is None
    write(16,16,2);frame=reader.read();assert frame.sequence==2 and frame.bgra.shape==(16,16,4)
    assert reader.read() is None
    write(32,16,4);frame=reader.read();assert frame.bgra.shape==(16,32,4)
    reader.close()


def make_camera_scene(tmp_path,robot_pose=(.21,-.18,.47),include_robot=True):
    width,height=1280,960;camera_height=2.6
    K=np.array([[1100.,0,width/2],[0,1100.,height/2],[0,0,1.]])
    R=np.diag([1.,-1.,-1.]);rvec=cv2.Rodrigues(R)[0];tvec=np.array([0.,0.,camera_height])
    image=np.full((height,width),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    def paint(identifier,world):
        nonlocal image
        marker=cv2.aruco.generateImageMarker(dictionary,identifier,160)
        target=cv2.projectPoints(np.asarray(world,np.float64),rvec,tvec,K,None)[0].reshape(4,2).astype(np.float32)
        source=np.array([[0,0],[159,0],[159,159],[0,159]],np.float32)
        H=cv2.getPerspectiveTransform(source,target)
        rendered=cv2.warpPerspective(marker,H,(width,height),flags=cv2.INTER_NEAREST,borderValue=255)
        image=np.minimum(image,rendered)
    references=[];size=.048
    for identifier,(x,y) in enumerate([(-1.05,.75),(1.05,.75),(-1.05,-.75),(1.05,-.75)],start=1):
        world=[[x-size/2,y+size/2,0],[x+size/2,y+size/2,0],[x+size/2,y-size/2,0],[x-size/2,y-size/2,0]]
        paint(identifier,world);references.append({"id":identifier,"corners_world":world})
    x,y,yaw=robot_pose;offset=np.array([-.02685,0.,0.]);tag_size=.04;tag_height=.023
    Rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
    local=np.array([[tag_size/2,tag_size/2,0],[tag_size/2,-tag_size/2,0],[-tag_size/2,-tag_size/2,0],[-tag_size/2,tag_size/2,0]])
    world=(Rz@(local+offset).T).T+np.array([x,y,tag_height])
    if include_robot:paint(0,world)
    calibration={"width":width,"height":height,"camera_matrix":K.tolist(),"dist_coeffs":[0]*5,
        "camera_position":[0,0,camera_height],"camera_target":[0,0,0],"marker_size_m":tag_size,
        "marker_height_m":tag_height,"marker_offset_m":offset[:2].tolist(),"arena_scale":1.,"reference_tags":references}
    path=tmp_path/"camera_calibration.json";path.write_text(json.dumps(calibration))
    bgra=cv2.cvtColor(image,cv2.COLOR_GRAY2BGRA)
    return path,Frame(2,1.,time.monotonic(),bgra)


def test_service_detects_real_pixels_and_height_offset(tmp_path):
    path,frame=make_camera_scene(tmp_path)
    result=CameraVision(path).process(frame)
    assert result.calibrated,result.message
    assert result.visible,result.message
    x,y,yaw=result.measurement
    assert x==pytest.approx(.21,abs=.006)
    assert y==pytest.approx(-.18,abs=.006)
    assert wrap(yaw-.47)==pytest.approx(0,abs=.055)
    assert result.markers==5


def test_service_no_robot_pixels_no_pose(tmp_path):
    path,frame=make_camera_scene(tmp_path,include_robot=False)
    result=CameraVision(path).process(frame)
    assert result.calibrated and not result.visible and result.measurement is None


def test_service_api_settings_goals_and_atomic_reset(tmp_path):
    service=RobotService(tmp_path)
    with TestClient(create_app(service,background=False)) as client:
        assert client.get("/api/config").status_code==200
        assert client.post("/api/mission/start").status_code>=400
        now=time.monotonic();service.receive(telemetry(now),now)
        service.estimator.correct((-.6,.55,-math.pi/2),now,now)
        assert client.post("/api/mission/start").json()["status"]=="running"
        assert client.post("/api/settings",json={"speed":.5}).json()["settings"]["speed"]==.5
        assert client.post("/api/settings",json={"mass_kg":.22}).status_code==409
        assert client.put("/api/goals",json={"goals":[{"id":"A","x":1,"y":1}]}).status_code==409
        assert client.post("/api/mission/pause").json()["status"]=="paused"
        assert client.post("/api/mission/stop").json()["status"]=="stopped"
        state=client.put("/api/goals",json={"goals":[{"id":"A","x":1,"y":1}]}).json()
        assert state["mission"]["goals"]==[{"id":"A","x":.6,"y":.55}]
        assert client.post("/api/mission/reset").status_code==200
        command=json.loads((tmp_path/"scene_command.json").read_text())
        assert command["action"]=="reset" and command["start"]["x"]==-.6
        assert not list(tmp_path.glob("*.tmp"))
        assert client.get("/api/camera.jpg").status_code==503
        with client.websocket_connect("/api/ws") as socket:
            assert "mission" in socket.receive_json()


def test_service_old_vision_and_old_telemetry_brake(tmp_path):
    service=RobotService(tmp_path);now=time.monotonic()
    service.receive(telemetry(now),now);service.estimator.correct((-.6,.55,-math.pi/2),now,now)
    service.mission_start();service.receive(telemetry(now+.30,seq=2,sim_time=1.3),now+.30)
    service.tick(now+.51)
    assert service.status=="paused" and service.sent_command.mode=="pause"
    service.tick(now+.61)
    assert service.status=="disconnected" and service.sent_command.mode=="stop"


def test_service_geometry_controller_completes_all_goals_without_shortcuts():
    robot,config=load_configuration();arena=Arena(config)
    navigator=Navigation(robot["track_width_m"])
    goals=[{"id":key,**arena.nodes[key].json()} for key in config["mission"]]
    pose={"x":-.6,"y":.55,"yaw":-math.pi/2};navigator.plan(arena,pose,goals,0.)
    left=right=0.;now=0.
    for _ in range(24000):
        command=navigator.tick(pose,now,.30,left,right)
        left=command.v-command.w*robot["track_width_m"]/2
        right=command.v+command.w*robot["track_width_m"]/2
        pose["x"]+=command.v*math.cos(pose["yaw"])*.01
        pose["y"]+=command.v*math.sin(pose["yaw"])*.01
        pose["yaw"]=wrap(pose["yaw"]+command.w*.01);now+=.01
        assert Point(pose["x"],pose["y"]).distance(arena.projection(Point(pose["x"],pose["y"]))[0])<.10
        if navigator.finished:break
    assert navigator.finished
    assert navigator.reached_goals==config["mission"]
    assert navigator.public(goals,pose)["progress"]==1.


def test_service_udp_protocol_virtual_driver(tmp_path,monkeypatch):
    async def scenario():
        loop=asyncio.get_running_loop();packets=asyncio.Queue()
        class Driver(asyncio.DatagramProtocol):
            def datagram_received(self,data,address):packets.put_nowait(json.loads(data))
        driver,_=await loop.create_datagram_endpoint(Driver,local_addr=("127.0.0.1",0))
        monkeypatch.setenv("ROBOTRACERS_COMMAND_PORT",str(driver.get_extra_info("sockname")[1]))
        monkeypatch.setenv("ROBOTRACERS_TELEMETRY_PORT","0")
        service=RobotService(tmp_path);await service.start()
        target=service.transport.get_extra_info("sockname")
        driver.sendto(json.dumps(telemetry()).encode(),target)
        await asyncio.sleep(.035)
        assert service.connected() and not service.estimator.initialized
        packets_seen=[]
        while not packets.empty():packets_seen.append(await packets.get())
        assert any(packet["mode"]=="idle" for packet in packets_seen)
        driver.sendto(b'{"type":"telemetry","seq":2,"stamp":NaN}',target)
        await asyncio.sleep(.01)
        assert service.invalid_packets==1
        await service.close();driver.close()
    asyncio.run(scenario())


def test_service_reset_discards_inflight_old_camera_frame(tmp_path):
    import threading
    from simulation.service.vision import VisionResult
    async def scenario():
        service=RobotService(tmp_path)
        stamp=time.monotonic()
        (tmp_path/"camera.frame").write_bytes(HEADER.pack(b"RRFRAME1",16,16,4,0,2,1.,stamp)+bytes([255])*16*16*4)
        processing=threading.Event();release=threading.Event()
        def delayed(frame):
            processing.set();release.wait(2)
            return VisionResult(True,(.6,-.55,1.),frame.capture_stamp,1.,1.,5,True,0.)
        service.vision.process=delayed
        task=asyncio.create_task(service.camera_loop())
        try:
            for _ in range(100):
                if processing.is_set():break
                await asyncio.sleep(.002)
            assert processing.is_set()
            service.scene_command("reset")
            service.receive({"type":"scene_ack","seq":service.scene_seq},time.monotonic())
            release.set();await asyncio.sleep(.03)
            assert not service.estimator.initialized
            assert service.accepted_vision==0
        finally:
            release.set();task.cancel();await asyncio.gather(task,return_exceptions=True);service.reader.close()
    asyncio.run(scenario())


def test_service_rejects_delayed_telemetry_and_empty_scene_ack(tmp_path):
    service=RobotService(tmp_path);now=time.monotonic()
    service.receive(telemetry(now-.4),now)
    assert not service.connected(now)
    service.estimator.correct((.1,.2,.3),now,now)
    service.receive({"type":"scene_ack"},now)
    assert service.estimator.initialized


def test_service_scene_sequence_survives_service_restart(tmp_path):
    first=RobotService(tmp_path);first.scene_command("reset")
    previous=first.scene_seq
    second=RobotService(tmp_path);second.scene_command("configure")
    command=json.loads((tmp_path/"scene_command.json").read_text())
    assert command["seq"]>previous


def test_service_goal_is_not_accepted_before_quiet_or_after_60mm_coast():
    robot,config=load_configuration();arena=Arena(config)
    navigator=Navigation(robot["track_width_m"])
    pose={"x":.6,"y":-.3,"yaw":-math.pi/2}
    navigator.plan(arena,pose,[{"id":"D","x":.6,"y":-.55}],0.)
    pose={**pose,"y":-.541}
    navigator.tick(pose,1.,.3,.1,.1)
    assert navigator.stage=="settle" and navigator.reached_goals==[]
    # Reproducción del fallo observado: el robot sobrepasa 60 mm después de la orden de freno.
    pose={**pose,"y":-.61}
    navigator.tick(pose,1.5,.3,0.,0.)
    navigator.tick(pose,2.2,.3,0.,0.)
    assert navigator.reached_goals==[] and not navigator.finished
    assert navigator.correcting and navigator.drive_sign==-1 and navigator.stage=="drive"
    command=navigator.tick(pose,2.21,.3,0.,0.)
    assert command.v<0 and abs(command.w)<.01
    # La corrección vuelve al punto; se acredita solo después del nuevo frenado y quietud.
    pose={**pose,"y":-.549}
    navigator.tick(pose,2.3,.3,-.02,-.02)
    assert navigator.reached_goals==[]
    navigator.tick(pose,2.4,.3,0.,0.)
    navigator.tick(pose,3.5,.3,0.,0.)
    assert navigator.finished and navigator.reached_goals==["D"]
    assert navigator.goal_measurements[0]["error_m"]<.002


def test_service_terminal_command_anticipates_stop_and_heading_never_flips():
    robot,config=load_configuration();arena=Arena(config)
    navigator=Navigation(robot["track_width_m"])
    pose={"x":.6,"y":-.3,"yaw":-math.pi/2}
    navigator.plan(arena,pose,[{"id":"D","x":.6,"y":-.55}],0.)
    for n in range(100):navigator.tick(pose,n*.01,.3,.1,.1)
    pose={**pose,"x":.603,"y":-.52}
    for n in range(100,200):command=navigator.tick(pose,n*.01,.3,.04,.04)
    assert command.v<=1.4*.03+.001
    assert abs(command.w)<.15
    pose={**pose,"y":-.551}
    command=navigator.tick(pose,2.01,.3,.03,.03)
    assert command.v==command.w==0 and navigator.stage=="settle"
    assert navigator.reached_goals==[]


def test_service_command_includes_capture_timestamp_for_latency(tmp_path):
    service=RobotService(tmp_path);now=time.monotonic()
    service.estimator.correct((0.,0.,0.),now,now)
    class Sender:
        def sendto(self,data,destination):self.message=json.loads(data)
    service.transport=Sender()
    from simulation.service.navigation import Command
    service.send(Command(.1,.2,"run"))
    assert service.transport.message["vision_stamp"]==now


def test_service_real_mission_corner_longitudinal_deadzone_brakes_instead_of_stalling():
    robot,config=load_configuration();arena=Arena(config)
    navigator=Navigation(robot["track_width_m"])
    start={"x":.5953,"y":.0008,"yaw":-3.1362}
    navigator.plan(arena,start,[{"id":"L","x":-.6,"y":0.}],0.)
    assert navigator.stage=="drive"
    # Valores del fallo real de la misión2: error total12.8mm, longitudinal2.6mm.
    pose={"x":-.5974,"y":-.0125,"yaw":3.0022}
    command=navigator.tick(pose,3.,.3,.003,-.003)
    assert command.v==command.w==0 and navigator.stage=="settle"
    assert navigator.reached_goals==[]
    navigator.tick(pose,3.01,.3,0.,0.)
    navigator.tick(pose,4.2,.3,0.,0.)
    assert navigator.finished and navigator.reached_goals==["L"]
    assert navigator.goal_measurements[0]["error_m"]<.018


def test_service_vision_multiscale_combines_ids_and_restores_native_pixels(tmp_path):
    path,frame=make_camera_scene(tmp_path)
    vision=CameraVision(path);detector=vision.detector
    class PartialDetector:
        def detectMarkers(self,pixels):
            corners,ids,rejected=detector.detectMarkers(pixels)
            scale=pixels.shape[0]/frame.bgra.shape[0]
            allowed={0} if scale==1 else {0,1,2} if scale==1.5 else {0,1,2,3,4}
            keep=[index for index,identifier in enumerate(ids.flatten()) if int(identifier) in allowed]
            return [corners[index] for index in keep],ids[keep],rejected
    vision.detector=PartialDetector()
    result=vision.process(frame)
    assert result.visible and result.markers==5 and result.detection_scale==3.
    assert result.measurement[0]==pytest.approx(.21,abs=.006)
    assert result.measurement[1]==pytest.approx(-.18,abs=.006)
    # Una vez calibrada, el marcador nativo basta: no se repiten los escalados de inicialización.
    second=vision.process(frame)
    assert second.visible and second.detection_scale==1.
