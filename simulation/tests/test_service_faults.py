"""Fallos integrados en cámara compartida, UDP y generación de consignas, sin hardware."""
import asyncio
import json
import time
import pytest
from simulation.adapters.real_camera import FrameWriter
from simulation.service.runtime import RobotService
from simulation.tests.test_service_core import make_camera_scene,telemetry

@pytest.mark.parametrize("fault",["vision","telemetry","stop"])
def test_service_integrated_faults_brake_and_never_resume_automatically(tmp_path,monkeypatch,fault):
    async def scenario():
        loop=asyncio.get_running_loop();commands=[]
        class Driver(asyncio.DatagramProtocol):
            def datagram_received(self,data,address):commands.append((time.monotonic(),json.loads(data)))
        driver,_=await loop.create_datagram_endpoint(Driver,local_addr=("127.0.0.1",0))
        monkeypatch.setenv("ROBOTRACERS_COMMAND_PORT",str(driver.get_extra_info("sockname")[1]))
        monkeypatch.setenv("ROBOTRACERS_TELEMETRY_PORT","0")
        _,frame=make_camera_scene(tmp_path,robot_pose=(-.6,.55,-1.5707963267948966))
        writer=FrameWriter(tmp_path/"camera.frame")
        service=RobotService(tmp_path);await service.start()
        address=service.transport.get_extra_info("sockname")
        camera_enabled=[True];telemetry_enabled=[True]
        async def camera_producer():
            while True:
                if camera_enabled[0]:writer.publish(frame.bgra,time.monotonic(),1.)
                await asyncio.sleep(.02)
        async def encoder_producer():
            seq=0
            while True:
                if telemetry_enabled[0]:
                    seq+=1
                    driver.sendto(json.dumps(telemetry(seq=seq,sim_time=seq*.01)).encode(),address)
                await asyncio.sleep(.01)
        producers=[asyncio.create_task(camera_producer()),asyncio.create_task(encoder_producer())]
        try:
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and not (service.connected() and service.visual_ready(time.monotonic())):
                await asyncio.sleep(.01)
            assert service.connected() and service.visual_ready(time.monotonic())
            service.mission_start();await asyncio.sleep(.06)
            assert any(command["mode"]=="run" for _,command in commands)
            changed=time.monotonic()
            if fault=="vision":camera_enabled[0]=False
            elif fault=="telemetry":telemetry_enabled[0]=False
            else:service.mission_stop()
            deadline=time.monotonic()+1
            expected="paused" if fault=="vision" else "disconnected" if fault=="telemetry" else "stopped"
            while time.monotonic()<deadline and service.status!=expected:await asyncio.sleep(.01)
            await asyncio.sleep(.03)
            assert service.status==expected
            assert commands[-1][1]["mode"] in {"pause","stop"}
            assert time.monotonic()-changed<(.80 if fault=="vision" else .60)
            first_safe=time.monotonic()
            camera_enabled[0]=telemetry_enabled[0]=True
            await asyncio.sleep(.10)
            assert not any(command["mode"]=="run" for stamp,command in commands if stamp>first_safe)
        finally:
            for producer in producers:producer.cancel()
            await asyncio.gather(*producers,return_exceptions=True)
            await service.close();writer.close();driver.close()
    asyncio.run(scenario())
