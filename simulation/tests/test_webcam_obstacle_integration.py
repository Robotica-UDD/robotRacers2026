"""Recorridos con obstáculos y robot cinemático; sin cámaras ni transporte real."""
import math
import time


from simulation.webcam.navigation import Navigation, INT32_MAX
from simulation.webcam.obstacle_planner import point_clearance_mm
from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, ClosedLoopRobot, wait_for


class WideCamera(FakeCamera):
    """Conserva los ejes de la planta ideal y amplía el campo a600 píxeles/m."""
    def pose(self, x=640., y=480., heading=0., scale=2000.):
        return super().pose(x=472. + (x - 640.) * .3,
                            y=480. + (y - 480.) * .3, heading=heading, scale=600.)


POLYGON = [[628., 444.], [652., 444.], [652., 516.], [628., 516.]]


def prepare(tmp_path=None, closed_loop=False):
    camera = WideCamera()
    robot = ClosedLoopRobot(camera) if closed_loop else FakeRobot()
    robot.update(max_velocity_mm_s=INT32_MAX)
    nav = Navigation(camera, robot, tmp_path / 'navigation-state.json' if tmp_path else None)
    nav.CONTROL_INTERVAL = .002
    nav.SETTLE_SECONDS = .01
    nav.WAYPOINT_SETTLE_SECONDS = .005
    nav.SPEED_RISE_MM_S2 = 6000.
    nav.SPEED_FALL_MM_S2 = 9000.
    nav.TURN_CONFIRM_SECONDS = .025
    nav.calibrate([472.,480.], [532.,480.], 1280,960,'usb-camera')
    nav.select_target(808.,480.,1280,960,'usb-camera')
    return camera, robot, nav


def install(nav, polygon=POLYGON):
    return nav.set_obstacles([polygon], 1280,960,'usb-camera',nav.state()['route_revision'])


def test_two_user_destinations_keep_their_order_while_both_legs_avoid_the_obstacle():
    camera, robot, nav = prepare(closed_loop=True)
    passed = []
    try:
        nav.append_target(472.,480.,1280,960,'usb-camera',nav.state()['route_revision'])
        install(nav)
        original_sync = nav._sync_target_locked
        def record_progress():
            original_sync()
            if nav._active:
                passed.append((nav._waypoint_index,robot.x,robot.y))
        nav._sync_target_locked = record_progress
        nav.start(nav.state()['route_revision'])
        wait_for(lambda:not nav.state()['active'], timeout=15.)
        state=nav.state()
        assert state['phase']=='arrived', (state['phase'],state['planning_status'],state['planning_message'])
        assert state['waypoint_index']==state['waypoint_count']==2
        assert [index for index,_,_ in passed] == [1,2]
        assert math.hypot(passed[0][1]-.56,passed[0][2]) <= .04
        assert math.hypot(robot.x,robot.y) <= .04
        assert robot.starts==1 and robot.drives[-1]==(0,0)
        corners=[[460.,468.],[484.,468.],[484.,492.],[460.,492.]]
        clearances=[point_clearance_mm(corners,nav._calibration,[472+x*600,480-y*600],[POLYGON])
                    for x,y,_,_,_ in robot.trace]
        assert min(clearances)>180.
        assert max(abs(y) for _,y,_,_,_ in robot.trace)>.25
    finally:
        nav.close()
