#!/usr/bin/env python3
import math
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from fanuc_msgs.msg import RobotStatusExt
from fanuc_msgs.srv import SetBoolIO
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor

class J2Mover(Node):
    def __init__(self):
        super().__init__('joint_j2_mover')
        self.joint_names = []
        self.initial_positions = []
        self.in_motion = False
        self.sent = False
        self.traj_ready = False
        self.traj = JointTrajectory()

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.robot_in_motion_sub = self.create_subscription(
             RobotStatusExt, "/fanuc_gpio_controller/robot_status_ext", self.check_robot_in_motion, 10
        )

        self.publisher_ = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.timer = self.create_timer(1.0, self.control_loop)

        self.set_io_client = self.create_client(SetBoolIO, '/fanuc_gpio_controller/set_bool_io')
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.io_request = SetBoolIO.Request()

    def check_robot_in_motion(self, msg: RobotStatusExt):
         self.in_motion = msg.in_motion
         return self.in_motion
    
    def control_loop(self):
        if not self.initial_positions:
            self.get_logger().info("Waiting for initial joint states...", throttle_duration_sec=5.0)
            return
        elif not self.traj_ready:
            self.create_trajectory()
        elif not self.in_motion and self.traj_ready and not self.sent:
            self.send_traj()
        elif self.in_motion:
            self.sent=False
            self.get_logger().info("robot still in motion")
            return

    def set_io_signal(self, index, value):
        self.io_request.io_type.type = 'DO'
        self.io_request.index = index
        self.io_request.value = value
        self.future = self.set_io_client.call_async(self.io_request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Result: %r' % self.future.result())
        else:
            self.get_logger().error('Service call failed %r' % (self.future.exception(),))

    def joint_state_callback(self, msg: JointState):
        self.joint_names = list(msg.name)
        self.initial_positions = list(msg.position)
        self.get_logger().info(f"Joint names: {self.joint_names}", throttle_duration_sec=1.0)
        self.get_logger().info(f"Initial positions: {self.initial_positions}", throttle_duration_sec=1.0)

    def create_trajectory(self): 
        #traj = JointTrajectory()
        self.traj.joint_names = self.joint_names
        j_index = 0
        duration = 10.0
        steps = 200
        start_pos = self.initial_positions.copy()

        amplitude = math.radians(15.0)  # 15 degrees amplitude


        for i in range(steps + 1):
            t = i * duration / steps
            phase = (2.0 * math.pi / duration) * t
            point = JointTrajectoryPoint()
            point.positions = start_pos.copy()

            # Sine wave trajectory
            angle = amplitude * (1 - math.cos(phase))
            point.positions[j_index] = start_pos[j_index] + angle

            # Calculate velocity (derivative of sine wave)
            velocity = amplitude * (2 * math.pi / duration) * math.sin(phase)
            point.velocities = [0.0] * len(point.positions)  # Initialize all velocities to 0
            point.velocities[j_index] = velocity  # Set velocity

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            self.traj.points.append(point)

        self.traj_ready = True
        self.get_logger().info("Created trajectory.")


    def send_traj(self):
        self.publisher_.publish(self.traj)
        self.get_logger().info("Published trajectory.")
        self.sent=True
        #self.timer.cancel()

    # Delay IO signal to wait for RMIInstance
    #self.io_timer = self.create_timer(2.0, self.delayed_io_signal)
        
    #def delayed_io_signal(self):
    #        self.set_io_signal(1, True)  # Set DO[1] to True
    #        self.io_timer.cancel()  # Cancel the timer after one execution

def main(args=None):
    rclpy.init(args=args)
    node = J2Mover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()