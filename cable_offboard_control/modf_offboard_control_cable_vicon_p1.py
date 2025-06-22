import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleAttitude,
    VehicleOdometry, VehicleRatesSetpoint
)
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import numpy as np
import time
import traceback

from george_params import quad_params
from load_ctrl.LoadController import LoadController
# from Traj.PolyTrajPhase1_2 import PolyTrajPhase1_2 as PolyTrajPhase2
# from Traj.modf_PolyTrajPhase_1 import PolyTrajPhase1 as PolyTrajPhase1
from Traj.total_phase_traj import PolyTrajPhase1
from Traj.PolyTraj_hold import HoldTrajectory

# for time measuring
from line_profiler import LineProfiler
import atexit, csv


def quat2mat(q):
   x, y, z, w = q
   xx, yy, zz, ww = x*x, y*y, z*z, w*w
   xy,xz,yz = x*y, x*z, y*z
   wx, wy, wz = w*x, w*y, w*z
   R = np.array([
       [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy)],
       [2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx)],
       [2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]
   ])
   return R

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.create_subscription(PoseStamped, '/vicon/ball/ball', self.load_pose_callback, 10)

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.load_pose = None
        self.prev_load_position = None
        self.prev_load_time = None
        self.vL_estimate = np.zeros(3)
        self.velocity_filter_alpha = 0.8
        self._last_time = self.get_clock().now().nanoseconds / 1e9

        self.control_frequency = quad_params['dt']
        self.h0 = 10.3052
        self.vy0 = 0.5
        self.vx0 = 0.5
        self.target_offset = 10.0014
        self.controller_counter = 0

        #self.trajectory = PolyTrajPhase1(h0=self.h0, vy0=self.vy0, vx0=self.vx0, target_x=self.target_offset, target_y=self.target_offset)
        self.trajectory = PolyTrajPhase1(t_final = 6)
#        self.trajectory = HoldTrajectory()
        load_params = {
           'mQ': quad_params['mass'],
           'mL': 0.050,
           'l': 0.3,
           'g': 9.80665,

           'JQ': np.diag([quad_params['Ixx'], quad_params['Iyy'], quad_params['Izz']]),
           'kx': np.diag([10, 10, 10]),     # Z方向减小一点避免过冲，x增大确保横向回轨
           'kv': np.diag([4, 8, 5]),     # 阻尼大幅增强，防止震荡（尤其是 Y）
           'kI': np.diag([0, 0, 0]),          # 积分增强，加快稳态误差修正

           'kq': 2,                       # 缆绳方向刚度保持
           'kw': 0.5,                       # 缆绳角速度略增强

           'kR': 8.0,                      # 姿态刚度略降低，避免角度抖动
           'kOmega': 0.7                   # 姿态速度反馈略降，抑制 M_z 震荡
       }




        self.load_controller = LoadController(load_params, logging=False)
        initState = self.trajectory.update(0)
        self.initialReferencePosition = initState['x']
        self.t_0 = self.get_clock().now().nanoseconds / 1e9
        self.initial_waypoint = self.get_initiliazation_pos()
        self.get_logger().info("Load controller initialized.")

        self.state = [0] * 13
        self.reached_initial_waypoint = False
        self.mission_accomplished = False
        self.landing_altitude = 0.35
        self.timer = self.create_timer(self.control_frequency, self.timer_callback)

    def load_pose_callback(self, msg):
        self.load_pose = msg
        current_time = self.get_clock().now().nanoseconds / 1e9
        position = np.array([msg.pose.position.x, msg.pose.position.y, -msg.pose.position.z])

        if self.prev_load_position is not None and self.prev_load_time is not None:
            dt = current_time - self.prev_load_time
            if dt > 0:
                raw_v = (position - self.prev_load_position) / dt
                self.vL_estimate = self.velocity_filter_alpha * self.vL_estimate + (1 - self.velocity_filter_alpha) * raw_v

        self.prev_load_position = position
        self.prev_load_time = current_time

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_attitude_callback(self, msg):
        pass  # Not used

    def vehicle_odometry_callback(self, msg):
        self.state[0] = msg.position[0]
        self.state[1] = -msg.position[1]
        self.state[2] = -msg.position[2]
        self.state[3] = msg.velocity[0]
        self.state[4] = -msg.velocity[1]
        self.state[5] = -msg.velocity[2]
        self.state[6] = msg.q[0]
        self.state[7] = msg.q[1]
        self.state[8] = -msg.q[2]
        self.state[9] = -msg.q[3]
        self.state[10] = msg.angular_velocity[0]
        self.state[11] = -msg.angular_velocity[1]
        self.state[12] = -msg.angular_velocity[2]

    def publish_offboard_control_heartbeat_signal(self, pos=False, vel=False, body_rate=False):
        msg = OffboardControlMode()
        msg.position = pos
        msg.velocity = vel
        msg.body_rate = body_rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
#        self.get_logger().info("heart beating..")

    def get_initiliazation_pos(self):
        return self.initialReferencePosition.reshape(3,).tolist()

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching into offboard mode")
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def move_to_initial_waypoint(self, sensitivity=1.0):
        self.publish_offboard_control_heartbeat_signal(pos=True)
        pos_curr = np.array(self.state[0:3])
        dis = np.linalg.norm(pos_curr - np.array(self.initial_waypoint))
        if dis >= sensitivity:
            msg = TrajectorySetpoint()
            msg.position = [self.initial_waypoint[0], -self.initial_waypoint[1], -self.initial_waypoint[2]]
            msg.yaw = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(msg)
            self.reached_initial_waypoint = False
        else:
            self.get_logger().info("Reached initial position. Starting trajectory tracking...")
            self.reached_initial_waypoint = True
            self.t_0 = self.get_clock().now().nanoseconds / 1e9
    @profile
    def load_tracking_control(self):
        # t0_flag = time.monotonic()
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._last_time
        self._last_time = now
        # print(f"Callback interval dt = {dt*1000:.1f} ms")
       
        thrust_max = quad_params['thrust_max']
        self.publish_offboard_control_heartbeat_signal(body_rate=True)

        t = (self.get_clock().now().nanoseconds / 1e9) - self.t_0
        if t > self.trajectory.t_final:
            self.mission_accomplished = True
            self.land()
            return t
#        t0_traj = time.monotonic()
        flat_output = self.trajectory.update(t)
#        t1_traj = time.monotonic()
#        self.get_logger().info(f"Trajectory update time: {(t1_traj - t0_traj)*1000:.6f} ms")
        xQ = np.array(self.state[0:3])
        vQ = np.array(self.state[3:6])
        quat = self.state[6:10]
        # t0_flag = time.monotonic()
        R = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        # t1_flag = time.monotonic()
        # self.get_logger().info(f"Flag time: {(t1_flag - t0_flag)*1000:.6f} ms")
        # R = quat2mat(quat)
        Omega = np.array(self.state[10:13])

        if self.load_pose is not None:
#        if True:
            xL = np.array([
                self.load_pose.pose.position.x,
                self.load_pose.pose.position.y,
                -self.load_pose.pose.position.z
            ])
            vL = self.vL_estimate
        else:
            xL = xQ + 0.3 * R @ np.array([0, 0, -1])
            vL = vQ

        state = {
            'xQ': xQ,
            'vQ': vQ,
            'R': R,
            'Omega': Omega,
            'xL': xL,
            'vL': vL,
            'w': np.zeros(3)
        }

#        t0_load = time.monotonic()
        out = self.load_controller.update(self.control_frequency, state, flat_output, t=t)
#        t1_load = time.monotonic()
        t0_flag = time.monotonic()
#        self.get_logger().info(f"controller update time: {(t1_load - t1_load)*1000:.6f} ms")
        self.controller_counter +=1
        f = out['f']
        Omega_cmd = out['Omega_cmd']

        msg = VehicleRatesSetpoint()
        msg.roll = Omega_cmd[0]
        msg.pitch = -Omega_cmd[1]
        msg.yaw = -Omega_cmd[2]
        msg.thrust_body = [0.0, 0.0, -f / thrust_max]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_rates_setpoint_publisher.publish(msg)
        t1_flag = time.monotonic()
        # self.get_logger().info(f"Flag time: {(t1_flag - t0_flag)*1000:.6f} ms")
        return t

    def timer_callback(self) -> None:
        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info("Arm and offboard Signal sent, Moving to initial point")
            self.offboard_setpoint_counter +=1
        # self.get_logger().info(f"vehicle_status.nav_state = {self.vehicle_status.nav_state} | "
        #     f"NAVIGATION_STATE_OFFBOARD = {VehicleStatus.NAVIGATION_STATE_OFFBOARD}") 
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_offboard_control_heartbeat_signal(pos=True)
            if self.reached_initial_waypoint:
                self.load_tracking_control()
                # self.get_logger().info("Reach init point")
            else:
                self.move_to_initial_waypoint()

        if self.offboard_setpoint_counter < 20:
            self.publish_offboard_control_heartbeat_signal(pos=True)
            sp = TrajectorySetpoint()
            sp.position = [
                self.initial_waypoint[0], -self.initial_waypoint[1], -self.initial_waypoint[2]
            ]
            sp.yaw = 0.0
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(sp)
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("An error occurred:", e)
        traceback.print_exc()
