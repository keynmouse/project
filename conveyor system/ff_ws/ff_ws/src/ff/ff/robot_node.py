# test/test_robot_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, CompressedImage
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import Header
from geometry_msgs.msg import Twist 
import time
import getkey
import math
import cv2
import numpy as np



j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150


th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset

# author : karl.kwon (mrthinks@gmail.com)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J0 to J2
def solv2(r1, r2, r3):
  d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
  d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

  s1 = math.acos(d1 / r1)
  s2 = math.acos(d2 / r2)

  return s1, s2

# author : karl.kwon (mrthinks@gmail.com)
# x, y, z : relational position from J0 (joint 0)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J2 to J3
# sr1 : angle between z-axis to J0->J1
# sr2 : angle between J0->J1 to J1->J2
# sr3 : angle between J1->J2 to J2->J3 (maybe always parallel)
def solv_robot_arm2(x, y, z, r1, r2, r3):
  z = z + r3 - j1_z_offset

  Rt = math.sqrt(x**2 + y**2 + z**2)
  Rxy = math.sqrt(x**2 + y**2)
  St = math.asin(z / Rt)
#   Sxy = math.acos(x / Rxy)
  Sxy = math.atan2(y, x)

  s1, s2 = solv2(r1, r2, Rt)

  sr1 = math.pi/2 - (s1 + St)
  sr2 = s1 + s2
  sr2_ = sr1 + sr2
  sr3 = math.pi - sr2_

  return Sxy, sr1, sr2, sr3, St, Rt


usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

w: forward
s: backward
d: turn right
a: turn left

<space>: stop

INIT : (1)

CTRL-C to quit
"""

joint_angle_delta = 0.1  # radian



class TestRobotNode(Node):
    def __init__(self):
        super().__init__('test_robot_node')
        
        # Publishers
        self.robot_state_pub = self.create_publisher(JointState, '/robot/joint_states', 10)
        self.robot_status_pub = self.create_publisher(String, '/robot/status', 10)
        
        # Subscribers
        self.robot_cmd_sub = self.create_subscription(String, '/robot/command', 
                                                    self.robot_cmd_callback, 10)
        self.robot_joint_sub = self.create_subscription(JointState, '/robot/joint_command', 
                                                      self.robot_joint_cmd_callback, 10)
        
        self.robot_camera_sub = self.create_subscription(
            CompressedImage,
            '/robot/camera/image_raw/compressed',
            self.robot_camera_callback,
            10
        )

        self.robot_cam_image = None
        
        # robot 
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        # Twist is geometry_msgs for linear and angular velocity 
        self.move_cmd = Twist() 
        self.trajectory_msg = JointTrajectory()
        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(100, 0, 0, r1, r2, r3)
        
        self.trajectory_msg.header = Header()
#        self.trajectory_msg.header.stamp = current_time.to_msg()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        self.trajectory_msg.points = [point]

        self.joint_pub.publish(self.trajectory_msg)
        
        # Timers
        self.create_timer(0.1, self.publish_robot_state)    # 10Hz
        self.create_timer(0.5, self.publish_robot_status)   # 2Hz
        
        # Test state
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_status = "IDLE"
        
        self.get_logger().info('Robot node started')

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0
    
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return
        self.gripper_action_client.send_goal_async(goal)

    def robot_camera_callback(self, msg):
        """Handle compressed robot camera feed"""
        try:
            # CompressedImage를 numpy array로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            # numpy array를 이미지로 디코딩
            self.robot_cam_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
   
        except Exception as e:
            self.get_logger().error(f'Failed to process robot camera image: {str(e)}')


    def robot_schedule(self):
        box = None
        self.go_forward()
        self.detect_and_grip()
        self.move_to_conveyor()
        self.go_backward()
        self.take_basket_to(box)

    
    def go_forward(self):
        while(self.is_far()):
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 0.
            self.cmd_vel.publish(self.move_cmd)

    def is_far(self):
        # TODO
        return True


    def detect_and_grip(self, items):
        # BGR에서 RGB로 변환
        cv_image = cv2.cvtColor(self.robot_cam_image, cv2.COLOR_BGR2RGB)
        
        # Run detection
        detections, annotated_frame = self.yolo.detect_image(cv_image)

        # Print detections
        for det in detections:
            if det['class_name'] in items:
                items.pop(det['class_name'])
                x1, y1, x2, y2 = det['bbox']
                x = (x1+x2)/2
                y = (y1+y2)/2
                x_index, y_index = self.check_grid(x,y)
                self.move_to_conveyor(x_index, y_index)
            print(f"Detected {det['class_name']} with confidence {det['confidence']:.2f}")

    def check_grid(self, x, y):
        vanishing_point = (640, 0) # 소실점 좌표
        grid_bottom_width = 100 # 그리드 바닥 너비
        grid_top_width = 50 # 그리드 상단 너비
        grid_bottom_y = 720 # 그리드 바닥 y좌표
        grid_top_y = 0 # 그리드 상단 y좌표

        # y 범위 체크
        if y < grid_top_y or y > grid_bottom_y:
            return -1, -1

        # 현재 y위치에서의 그리드 너비 계산
        t = (y - grid_top_y) / (grid_bottom_y - grid_top_y)
        current_width = grid_top_width + (grid_bottom_width - grid_top_width) * t
        left_bound = vanishing_point[0] - current_width/2
        right_bound = vanishing_point[0] + current_width/2

        # x 범위 체크
        if x < left_bound or x > right_bound:
            return -1, -1

        # 그리드 인덱스 계산
        normalized_x = (x - left_bound) / current_width
        normalized_y = (y - grid_top_y) / (grid_bottom_y - grid_top_y)

        grid_x = min(2, int(normalized_x * 3))
        grid_y = min(2, int(normalized_y * 3))

        return grid_x, grid_y

    def move_to_conveyor(self, x_index, y_index):
        # 포즈 저장
        index = x_index * 3 + y_index


        # 012
        # 345
        # 678

        poses = {
            0 : [],
            1 : [],
            3 : [],
            3 : [],
            4 : [],
            5 : [],
            6 : [],
            7 : [],
            8 : [],
        }

        self.send_gripper_goal(-0.015)
        # poses[index]로 이동
        self.send_gripper_goal(0.01)
        # 원래 위치로 돌아가기

    def go_backward(self):
        while(self.is_far()):
            self.move_cmd.linear.x = -0.1
            self.move_cmd.angular.z = 0.
            self.cmd_vel.publish(self.move_cmd)

    def take_basket_to(self, box):
        pass


    def robot_cmd_callback(self, msg):
        self.get_logger().info(f'Received robot command: {msg.data}')
        if msg.data == "START":
            self.robot_status = "RUNNING"
            self.robot_schedule

        elif msg.data == "STOP":
            self.robot_status = "STOPPED"

    def robot_joint_cmd_callback(self, msg):
        self.get_logger().info(f'Received joint command: {msg.position}')

    def publish_robot_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Simulate joint movement
        self.joint_positions = [
            (p + 0.01) % 6.28 for p in self.joint_positions
        ]
        msg.position = self.joint_positions
        
        self.robot_state_pub.publish(msg)

    def publish_robot_status(self):
        msg = String()
        msg.data = self.robot_status
        self.robot_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()