"""Adaptador ROS 2 optativo. No importa rclpy ni abre sockets al importar el módulo."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import socket
import time
import urllib.request
from simulation.service.estimator import wrap
from simulation.service.vision import CameraReader
from simulation.service.config import load_configuration

class RosBridgeCore:
    """Transformaciones SI y límites de consigna, comprobables sin una instalación ROS."""
    def __init__(self,track_width:float=.19,clock=time.monotonic,wheel_radius:float=.0115,counts_per_turn:float=59.94):
        self.track=track_width;self.clock=clock;self.x=self.y=self.yaw=0.
        self.previous_time=None;self.previous_counts=None;self.latest_velocity=None;self.command_seq=0
        self.count_distance=2*math.pi*wheel_radius/counts_per_turn
    def velocity(self,v:float,w:float):
        if not math.isfinite(v) or not math.isfinite(w):self.latest_velocity=None;return False
        self.latest_velocity=(max(-1.5,min(1.5,v)),max(-12.,min(12.,w)),self.clock());return True
    def command(self,enable_motion:bool,connected:bool=True) -> dict:
        now=self.clock();self.command_seq+=1;v=w=0.;mode="stop"
        if enable_motion and connected and self.latest_velocity is not None and now-self.latest_velocity[2]<=.25:
            v,w,_=self.latest_velocity;mode="run"
        return {"seq":self.command_seq,"stamp":now,"v":v,"w":w,"mode":mode}
    def update(self,state:dict) -> dict:
        now=float(state["sim_time"])
        robot=state["robot"];left=float(robot["left_speed"]);right=float(robot["right_speed"])
        v=(left+right)/2;w=(right-left)/self.track
        counts=(robot.get("left_count"),robot.get("right_count"))
        have_counts=all(isinstance(value,int) for value in counts)
        if self.previous_time is not None:
            dt=now-self.previous_time
            if dt<0:self.x=self.y=self.yaw=0.
            elif dt<=.5 and state["connected"] and not state.get("scene_pending",False):
                distance=v*dt;angle=w*dt
                if have_counts:
                    if self.previous_counts is None:distance=angle=0.
                    else:
                        dl=(counts[0]-self.previous_counts[0])*self.count_distance
                        dr=(counts[1]-self.previous_counts[1])*self.count_distance
                        if max(abs(dl),abs(dr))>1.6*dt+.02:distance=angle=0.
                        else:distance=(dl+dr)/2;angle=(dr-dl)/self.track
                middle=self.yaw+angle/2
                self.x+=distance*math.cos(middle);self.y+=distance*math.sin(middle);self.yaw=wrap(self.yaw+angle)
        self.previous_time=now;self.previous_counts=counts if have_counts else None
        pose=state["pose"];map_to_odom=None
        if pose.get("source")!="sin localización":
            angle=wrap(float(pose["yaw"])-self.yaw);c,s=math.cos(angle),math.sin(angle)
            map_to_odom={"x":float(pose["x"])-(c*self.x-s*self.y),
                "y":float(pose["y"])-(s*self.x+c*self.y),"yaw":angle}
        return {"odom":{"x":self.x,"y":self.y,"yaw":self.yaw},"map_to_odom":map_to_odom,
            "v":v,"w":w,"time":now}


def quaternion(yaw):return (0.,0.,math.sin(yaw/2),math.cos(yaw/2))


def run_ros2(arguments):
    # Las dependencias ROS pertenecen al entorno ROS del usuario, no al servicio en macOS.
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock,ClockType
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist,TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image,CameraInfo
        from tf2_ros import TransformBroadcaster
        from builtin_interfaces.msg import Time
        if arguments.use_sim_time:from rosgraph_msgs.msg import Clock as ClockMessage
    except ImportError as exc:
        raise RuntimeError("Este adaptador requiere un entorno ROS 2 con rclpy y sus mensajes. El servicio local no lo necesita.") from exc

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("robotracers_bridge")
            robot,_=load_configuration();self.core=RosBridgeCore(robot["track_width_m"],wheel_radius=robot["wheel_radius_m"],counts_per_turn=robot["encoder_counts_per_revolution"])
            self.reader=CameraReader(arguments.runtime/"camera.frame")
            self.odom_publisher=self.create_publisher(Odometry,"odom",10)
            self.image_publisher=self.create_publisher(Image,"overhead/image_raw",qos_profile_sensor_data)
            self.info_publisher=self.create_publisher(CameraInfo,"overhead/camera_info",qos_profile_sensor_data)
            self.transforms=TransformBroadcaster(self)
            self.subscription=self.create_subscription(Twist,"cmd_vel",self.on_velocity,1)
            self.clock_publisher=self.create_publisher(ClockMessage,"clock",10) if arguments.use_sim_time else None
            self.udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if arguments.enable_motion else None
            self.timer=self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def on_velocity(self,message):self.core.velocity(float(message.linear.x),float(message.angular.z))
        def stamp(self,seconds:float|None=None):
            if seconds is None:return self.get_clock().now().to_msg()
            message=Time();message.sec=int(seconds);message.nanosec=int((seconds-int(seconds))*1e9);return message
        def capture_stamp(self,frame):
            if arguments.use_sim_time:return self.stamp(frame.sim_time)
            age=max(0.,time.monotonic()-frame.capture_stamp)
            nanoseconds=max(0,self.get_clock().now().nanoseconds-int(age*1e9))
            stamp=Time();stamp.sec=nanoseconds//1_000_000_000;stamp.nanosec=nanoseconds%1_000_000_000
            return stamp
        def transform(self,parent,child,pose,stamp):
            message=TransformStamped();message.header.stamp=stamp;message.header.frame_id=parent;message.child_frame_id=child
            message.transform.translation.x=pose["x"];message.transform.translation.y=pose["y"]
            q=quaternion(pose["yaw"])
            message.transform.rotation.x,message.transform.rotation.y,message.transform.rotation.z,message.transform.rotation.w=q
            return message
        def tick(self):
            state=None
            try:
                with urllib.request.urlopen(arguments.service+"/api/state",timeout=.12) as response:state=json.load(response)
                values=self.core.update(state);stamp=self.stamp(values["time"] if arguments.use_sim_time else None)
                if self.clock_publisher:
                    clock=ClockMessage();clock.clock=stamp;self.clock_publisher.publish(clock)
                if state["connected"]:
                    message=Odometry();message.header.stamp=stamp;message.header.frame_id="odom";message.child_frame_id="base_link"
                    odom=values["odom"];message.pose.pose.position.x=odom["x"];message.pose.pose.position.y=odom["y"]
                    q=quaternion(odom["yaw"])
                    message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w=q
                    message.twist.twist.linear.x=values["v"];message.twist.twist.angular.z=values["w"]
                    # Covarianza provisional: no representa una calibración de hardware.
                    message.pose.covariance[0]=message.pose.covariance[7]=.0025
                    message.pose.covariance[35]=.01
                    self.odom_publisher.publish(message)
                    transforms=[self.transform("odom","base_link",odom,stamp)]
                    if values["map_to_odom"] is not None:transforms.append(self.transform("map","odom",values["map_to_odom"],stamp))
                    self.transforms.sendTransform(transforms)
                frame=self.reader.read()
                if frame is not None:
                    image=Image();image.header.frame_id="overhead_camera_optical_frame"
                    image.header.stamp=self.capture_stamp(frame)
                    image.height,image.width=frame.bgra.shape[:2];image.encoding="bgra8";image.is_bigendian=0
                    image.step=image.width*4;image.data=frame.bgra.tobytes();self.image_publisher.publish(image)
                    try:
                        config=json.loads((arguments.runtime/"camera_calibration.json").read_text())
                        if (config["width"],config["height"])==(image.width,image.height):
                            info=CameraInfo();info.header=image.header;info.width=image.width;info.height=image.height
                            info.distortion_model="plumb_bob";info.d=list(config.get("dist_coeffs",[0.]*5))
                            info.k=[float(value) for row in config["camera_matrix"] for value in row]
                            info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                            K=config["camera_matrix"];info.p=[K[0][0],0.,K[0][2],0.,0.,K[1][1],K[1][2],0.,0.,0.,1.,0.]
                            self.info_publisher.publish(info)
                    except (OSError,ValueError,KeyError):pass
            except (OSError,ValueError,KeyError,TypeError):pass
            if self.udp is not None:
                safe=bool(state and state.get("connected") and state.get("vision",{}).get("visible"))
                command=self.core.command(True,safe)
                self.udp.sendto(json.dumps(command,allow_nan=False).encode(),("127.0.0.1",arguments.command_port))
        def close(self):
            if self.udp:
                self.udp.sendto(json.dumps(self.core.command(False)).encode(),("127.0.0.1",arguments.command_port));self.udp.close()
            self.reader.close();self.destroy_node()

    rclpy.init();node=BridgeNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description="Publicación ROS 2 opcional de RobotRacers; lectura por defecto.")
    parser.add_argument("--runtime",required=True,type=Path)
    parser.add_argument("--service",default="http://127.0.0.1:8000")
    parser.add_argument("--use-sim-time",action="store_true")
    parser.add_argument("--enable-motion",action="store_true",help="Asume propiedad EXCLUSIVA de las consignas; consulta la documentación.")
    parser.add_argument("--command-port",type=int,default=9212)
    arguments=parser.parse_args()
    try:run_ros2(arguments)
    except RuntimeError as exc:raise SystemExit(str(exc))

if __name__=="__main__":main()
