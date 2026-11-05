import json
import math
from collections import deque
import numpy as np
import pytest
from simulation.adapters.real_camera import RealCamera,FrameWriter
from simulation.adapters.hardware import Esp32Bridge,SerialLineTransport
from simulation.adapters.ros2_bridge import RosBridgeCore
from simulation.service.vision import CameraReader

class Transport:
    def __init__(self,lines=()):self.lines=deque(lines);self.sent=[];self.closed=False
    def readline(self):return self.lines.popleft() if self.lines else b""
    def write(self,data):self.sent.append(json.loads(data));return len(data)
    def close(self):self.closed=True


def test_service_real_camera_is_opt_in_and_uses_same_shared_frame(tmp_path):
    calls=[]
    class Capture:
        def __init__(self,source):calls.append(source);self.released=False
        def isOpened(self):return True
        def set(self,*args):pass
        def read(self):return True,np.full((16,16,3),[10,20,30],np.uint8)
        def release(self):self.released=True
    times=iter([1.,1.02])
    camera=RealCamera(3,{"width":16,"height":16,"camera_matrix":np.eye(3).tolist()},Capture,lambda:next(times))
    assert not calls
    with pytest.raises(RuntimeError):camera.read()
    camera.open();frame,stamp,elapsed=camera.read()
    assert calls==[3] and stamp==1.02 and elapsed==pytest.approx(.02)
    writer=FrameWriter(tmp_path/"camera.frame");reader=CameraReader(tmp_path/"camera.frame")
    writer.publish(frame,stamp,elapsed);received=reader.read()
    assert received.capture_stamp==stamp and received.bgra[0,0].tolist()==[10,20,30,255]
    old_mapping=reader.mapping
    writer.publish(np.zeros((32,16,4),np.uint8),1.04,.04)
    assert old_mapping[:8]==b"RRFRAME1"
    assert reader.read().bgra.shape==(32,16,4)
    reader.close();writer.close();camera.close()


def test_service_serial_transport_does_not_open_on_construction(monkeypatch):
    import simulation.adapters.hardware as hardware
    imported=[]
    monkeypatch.setattr(hardware.importlib,"import_module",lambda name:imported.append(name))
    transport=SerialLineTransport("/dev/not-a-real-device")
    assert imported==[] and transport.serial is None
    with pytest.raises(RuntimeError):transport.write(b"test")


def test_service_encoder_bridge_signs_clock_wrap_and_device_reset():
    bridge=Esp32Bridge(Transport(),clock=lambda:1.)
    first=bridge.encoder({"type":"encoder","left_count":-10,"right_count":10,"device_micros":4294960000},1.1)
    assert first["left_count"]==first["right_count"]==10 and first["stamp"]==1.1
    second=bridge.encoder({"type":"encoder","left_count":-11,"right_count":12,"device_micros":10000},1.2)
    assert second["left_count"]==11 and second["right_count"]==12
    assert second["left_velocity"]==pytest.approx(bridge.distance/.017296)
    assert second["right_velocity"]==pytest.approx(2*bridge.distance/.017296)
    assert not second["device_reset"]
    third=bridge.encoder({"type":"encoder","left_count":0,"right_count":0,"device_micros":500},1.3)
    assert third["device_reset"] and third["left_count"]==11 and third["right_count"]==12
    assert bridge.needs_rearm


def test_service_bridge_commands_require_motion_flag_rearm_and_fresh_data():
    transport=Transport();now=[10.]
    bridge=Esp32Bridge(transport,allow_motion=True,clock=lambda:now[0])
    base={"seq":1,"stamp":10.,"v":.2,"w":1.,"mode":"run"}
    assert bridge.command(base) and transport.sent[-1]["mode"]=="stop"
    assert bridge.command({**base,"seq":2,"mode":"idle"})
    assert bridge.command({**base,"seq":3})
    packet=transport.sent[-1]
    assert packet["left_mps"]==pytest.approx(.105) and packet["right_mps"]==pytest.approx(.295)
    assert packet["ttl_ms"]==250
    assert not bridge.command(base)
    assert not bridge.command({**base,"seq":4,"stamp":9.})
    now[0]=10.3;bridge.watchdog()
    assert transport.sent[-1]["mode"]=="stop" and transport.sent[-1]["seq"]>packet["seq"]
    disabled=Esp32Bridge(Transport(),clock=lambda:10.)
    disabled.command({**base,"mode":"idle"});disabled.command({**base,"seq":2})
    assert disabled.transport.sent[-1]["mode"]=="stop"


def test_service_bridge_json_lines_handles_fragments_and_diagnostics():
    record={"type":"encoder","left_count":-4,"right_count":4,"device_micros":100}
    encoded=json.dumps(record).encode()+b"\n"
    bridge=Esp32Bridge(Transport([b"diagnostico sin JSON\n"+encoded[:20],encoded[20:]]),clock=lambda:2.)
    assert bridge.read()==[]
    assert bridge.read()[0]["left_count"]==4
    bridge.close();assert bridge.transport.closed


def test_service_ros_frames_are_consistent_without_ros_installed():
    core=RosBridgeCore(clock=lambda:1.)
    state={"connected":True,"sim_time":0.,"pose":{"x":2.,"y":3.,"yaw":math.pi/2,"source":"cámara + encoders"},
        "robot":{"left_speed":1.,"right_speed":1.}}
    core.update(state);state={**state,"sim_time":.1,"pose":{**state["pose"],"y":3.1}}
    output=core.update(state)
    assert output["odom"]["x"]==pytest.approx(.1)
    transform=output["map_to_odom"]
    assert transform["x"]==pytest.approx(2.) and transform["y"]==pytest.approx(3.)
    assert transform["yaw"]==pytest.approx(math.pi/2)
    assert core.velocity(.3,.4)
    assert core.command(False)["mode"]=="stop"
    assert core.command(True)["v"]==.3
    assert not core.velocity(float("nan"),0)
    assert core.command(True)["mode"]=="stop"


def test_service_ros_uses_encoder_counts_instead_of_integrating_reported_speed():
    core=RosBridgeCore(clock=lambda:1.)
    state={"connected":True,"sim_time":0.,"pose":{"x":0.,"y":0.,"yaw":0.,"source":"cámara + encoders"},
        "robot":{"left_speed":9.,"right_speed":9.,"left_count":0,"right_count":0}}
    core.update(state)
    state={**state,"sim_time":.1,"robot":{**state["robot"],"left_count":10,"right_count":10}}
    result=core.update(state)
    assert result["odom"]["x"]==pytest.approx(10*core.count_distance)
    assert result["odom"]["x"]<.02
