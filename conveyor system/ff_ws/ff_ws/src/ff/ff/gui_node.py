import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool, Int32
from sensor_msgs.msg import Image, JointState, CompressedImage
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge
import sys
import cv2
import numpy as np
from datetime import datetime
import json
import os
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import smtplib
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtCore import Qt, QTimer, QThreadPool, QRunnable
from PyQt5.QtGui import QImage, QPixmap
from ff.login import LoginDialog
from ff.yolo_detector import YOLODetector


# 에러 코드
# 0: 에러 없음
# 1: 컨베이어 연결 에러
# 2: USB 연결 에러

class RobotControlUINode(Node):
    def __init__(self):
        super().__init__('robot_control_ui_node')


        # YOLO 초기화
        self.yolo = YOLODetector(
            model_path='./models/yolov8n.pt',  # Use YOLOv8 nano model
            conf_threshold=0.5
        )

        # 에러 정리코드
        self.error_status = {
            -1 : "기타",
            0 : "에러 없음",
            1 : "컨베이어 연결 에러",
            2 : "USB 연결 에러",
            3 : "",
        }

        # 상태 변수들
        self.current_task = None
        self.work_start_time = None
        self.is_collecting_data = False
        self.data_collection_dir = './data/'
        self.error_logs = []
        self.__all_layouts = []

        # 메일 설정 (네이버)
        self.__from_mail = ''
        self.__to_mail = ''
        self.__id = ''
        self.__passworld = ''
        
        # Publishers
        self.robot_cmd_pub = self.create_publisher(String, '/robot/command', 10)
        self.robot_joint_pub = self.create_publisher(JointState, '/robot/joint_command', 10)
        self.conveyor_cmd_pub = self.create_publisher(Twist, '/conveyor/command', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.robot_state_callback,
            10
        )
        self.robot_status_sub = self.create_subscription(
            String,
            '/robot/status',
            self.robot_status_callback,
            10
        )
        self.conveyor_state_sub = self.create_subscription(
            String,
            '/conveyor/state',
            self.conveyor_state_callback,
            10
        )
        self.robot_camera_sub = self.create_subscription(
            CompressedImage,
            '/robot/camera/image_raw/compressed',
            self.robot_camera_callback,
            10
        )
        self.global_camera_sub = self.create_subscription(
            Image,
            '/global/camera/image_raw',
            self.global_camera_callback,
            10
        )
        self.conveyor_error_sub = self.create_subscription(Int32, '/conveyor/error', 
                                                       self.conveyor_error_callback, 10)
        
        # self.conveyor_serial_data = self.create_subscription(String, '/conveyor/serial_data', 
        #                                                self.conveyor_error_callback, 10)
        # Initialize UI
        self.app = QApplication(sys.argv)
        self.init_ui()
        self.bridge = CvBridge()
        
        # Timer for status updates
        self.timer = self.create_timer(0.1, self.update_status)
        self.create_timer(1.0, self.update_work_time)

    def robot_state_callback(self, msg):
        """Handle robot joint states"""
        joint_positions = msg.position
        # Update UI with joint positions
        self.update_joint_display(joint_positions)

    def robot_status_callback(self, msg):
        """Handle robot status updates"""
        status = msg.data
        self.status_label.setText(f"Robot Status: {status}")

    def conveyor_state_callback(self, msg):
        """Handle conveyor state updates"""
        state = msg.data
        self.speed_label.setText(f"Conveyor State: {state}")

    def robot_camera_callback(self, msg):
        """Handle compressed robot camera feed"""
        try:
            # CompressedImage를 numpy array로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            # numpy array를 이미지로 디코딩
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # BGR에서 RGB로 변환
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # # Run detection
            # detections, annotated_frame = self.yolo.detect_image(cv_image)

            # # Print detections
            # for det in detections:
            #     print(f"Detected {det['class_name']} with confidence {det['confidence']:.2f}")

            # yolo 사용 안 함
            annotated_frame = cv_image
                
            self.update_robot_camera_display(annotated_frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process robot camera image: {str(e)}')

    def global_camera_callback(self, msg):
        """Handle global camera feed"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.update_global_camera_display(cv_image)
        except Exception as e:
            self.get_logger().error(f'Failed to process global camera image: {str(e)}')

    def send_mail(self, to, subject, message):
        # 네이버 SMTP 서버 설정
        smtp_server = smtplib.SMTP('smtp.naver.com', 587)
        smtp_server.ehlo()
        smtp_server.starttls()

        # 네이버 이메일 계정 로그인
        smtp_server.login(self.__id, self.__passworld)

        # 이메일 메시지 구성
        msg = MIMEMultipart()
        msg['Subject'] = subject
        msg.attach(MIMEText(message))

        # 이메일 메시지 전송
        msg['From'] = self.__from_mail
        msg['To'] = to
        smtp_server.sendmail(msg['From'], msg['To'], msg.as_string())

        smtp_server.quit()

    def conveyor_error_callback(self, msg):
        if msg.data == 0:   # 에러 아님
            return
            
        self.get_logger().error(f'Error on conveyor: error-code-{msg.data}')
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] Error on conveyor: error-code-{self.error_status[msg.data]}"
        self.log_text.append(log_entry)
        

        if self.__to_mail == '':
            return
        self.send_mail(self.__to_mail, 
                       f'Error on conveyor: error-code-{msg.data}', 
                       f'Error on conveyor: error-code-{msg.data}')

    def send_robot_command(self, command):
        """Send command to robot"""
        msg = String()
        msg.data = command
        self.robot_cmd_pub.publish(msg)

    def send_conveyor_command(self, speed):
        """Send command to conveyor"""
        msg = Twist()
        msg.linear.x = float(speed)  # Use linear.x for speed
        self.conveyor_cmd_pub.publish(msg)

    def emergency_stop(self):
        """비상 정지 기능"""
        msg = Bool()
        msg.data = True
        self.emergency_pub.publish(msg)
        self.emergency_button.setEnabled(False)
        self.resume_button.setEnabled(True)
        for layout in self.__all_layouts:
            layout.setEnabled(False)
        self.log_text.append("Emergency stop activated")

    def resume_operation(self):
        """Resume operation after emergency stop"""
        msg = Bool()
        msg.data = False
        self.emergency_pub.publish(msg)
        self.emergency_button.setEnabled(True)
        self.resume_button.setEnabled(False)
        for layout in self.__all_layouts:
            layout.setEnabled(True)
        self.log_text.append("Resumed")
    
    def init_ui(self):
        """Initialize the user interface"""
        self.window = QMainWindow()
        self.window.setWindowTitle('Robot Control System')
        self.window.setGeometry(100, 100, 1600, 900)
        self.window.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                border: 2px solid #cccccc;
                border-radius: 6px;
                margin-top: 1ex;
                font-weight: bold;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                padding: 0 5px;
            }
            QPushButton {
                padding: 8px 15px;
                border-radius: 4px;
                border: none;
                background-color: #2196F3;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:disabled {
                background-color: #BDBDBD;
            }
            QLineEdit {
                padding: 8px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: white;
            }
            QLabel {
                color: #333333;
                font-size: 14px;
            }
        """)
    
        # Create central widget and layout
        central_widget = QWidget()
        self.window.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
    
        # Left panel for camera feeds
        left_panel = QVBoxLayout()
        left_panel.setSpacing(15)
    
        # Camera feeds in separate group boxes
        robot_camera_group = QGroupBox("Robot Camera Feed")
        robot_camera_layout = QVBoxLayout()
        # Robot camera label setup
        self.robot_camera_label = QLabel()
        self.robot_camera_label.setMinimumSize(800, 400)
        self.robot_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_camera_label.setStyleSheet("""
            QLabel {
                background-color: #1e1e1e;
                border-radius: 8px;
                padding: 10px;
            }
        """)
        robot_camera_layout.addWidget(self.robot_camera_label)
        robot_camera_group.setLayout(robot_camera_layout)
    
        global_camera_group = QGroupBox("Global Camera Feed")
        global_camera_layout = QVBoxLayout()
        # Global camera label setup
        self.global_camera_label = QLabel()
        self.global_camera_label.setMinimumSize(800, 400)
        self.global_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.global_camera_label.setStyleSheet("""
            QLabel {
                background-color: #1e1e1e;
                border-radius: 8px;
                padding: 10px;
            }
        """)
        global_camera_layout.addWidget(self.global_camera_label)
        global_camera_group.setLayout(global_camera_layout)
    
        left_panel.addWidget(robot_camera_group)
        left_panel.addWidget(global_camera_group)
    
        # Right panel for controls
        right_panel = QVBoxLayout()
        right_panel.setSpacing(15)
    
        # Status group
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel("Robot Status: IDLE")
        self.status_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                background-color: #e8f5e9;
                border-radius: 4px;
            }
        """)
        
        self.speed_label = QLabel("Conveyor State: Unkown")
        self.speed_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                background-color: #e3f2fd;
                border-radius: 4px;
            }
        """)
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.speed_label)
        status_group.setLayout(status_layout)
        right_panel.addWidget(status_group)
    
        # Emergency controls
        emergency_group = QGroupBox("Emergency Controls")
        emergency_layout = QHBoxLayout()
        
        self.emergency_button = QPushButton("EMERGENCY STOP")

        self.emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                font-size: 16px;
                padding: 15px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        self.emergency_button.clicked.connect(self.emergency_stop)
        
        self.resume_button = QPushButton("Resume Operation")

        self.resume_button.setStyleSheet("""
            QPushButton {
                background-color: #4caf50;
                font-size: 16px;
                padding: 15px;
            }
            QPushButton:hover {
                background-color: #388e3c;
            }
        """)
        self.resume_button.clicked.connect(self.resume_operation)
        self.resume_button.setEnabled(False)
        
        emergency_layout.addWidget(self.emergency_button)
        emergency_layout.addWidget(self.resume_button)
        emergency_group.setLayout(emergency_layout)
        right_panel.addWidget(emergency_group)
    
        # Conveyor controls
        conveyor_group = QGroupBox("Conveyor Controls")
        conveyor_layout = QVBoxLayout()
    
        # Speed input with label in horizontal layout
        speed_input_layout = QHBoxLayout()
        speed_label = QLabel("Speed:")
        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Enter speed (0-100)")
        speed_input_layout.addWidget(speed_label)
        speed_input_layout.addWidget(self.speed_input)
        conveyor_layout.addLayout(speed_input_layout)
    
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.forward_button = QPushButton("Forward")
        self.__all_layouts.append(self.forward_button)
        self.forward_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        
        self.reverse_button = QPushButton("Reverse")
        self.__all_layouts.append(self.reverse_button)
        self.reverse_button.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
            }
            QPushButton:hover {
                background-color: #f57c00;
            }
        """)
        
        self.stop_button = QPushButton("Stop")
        self.__all_layouts.append(self.stop_button)
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #757575;
            }
            QPushButton:hover {
                background-color: #616161;
            }
        """)
    
        button_layout.addWidget(self.forward_button)
        button_layout.addWidget(self.reverse_button)
        button_layout.addWidget(self.stop_button)
        conveyor_layout.addLayout(button_layout)
    
        self.forward_button.clicked.connect(
            lambda: self.send_conveyor_command(float(self.speed_input.text() or "0")))
        self.reverse_button.clicked.connect(
            lambda: self.send_conveyor_command(-float(self.speed_input.text() or "0")))
        self.stop_button.clicked.connect(
            lambda: self.send_conveyor_command(-1))
    
        conveyor_group.setLayout(conveyor_layout)
        right_panel.addWidget(conveyor_group)
    
        # Add stretch to push everything up
        right_panel.addStretch()
    
        # Add panels to main layout
        main_layout.addLayout(left_panel, stretch=2)
        main_layout.addLayout(right_panel, stretch=1)
    
        self.window.show()

        # 작업 선택 그룹
        task_group = QGroupBox("Task Selection")
        task_layout = QVBoxLayout()

        self.task_combo = QComboBox()
        self.__all_layouts.append(self.task_combo)
        self.task_combo.addItems(['Pick and Place', 'Assembly', 'Inspection', 'Packaging'])
        self.task_combo.currentTextChanged.connect(self.task_selected)

        self.work_time_label = QLabel("작업 시간: 00:00:00")
        self.work_time_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                padding: 10px;
                background-color: #fff3e0;
                border-radius: 4px;
            }
        """)

        task_layout.addWidget(self.task_combo)
        task_layout.addWidget(self.work_time_label)
        task_group.setLayout(task_layout)
        right_panel.addWidget(task_group)

        # 데이터 수집 그룹 추가
        data_collection_group = QGroupBox("Data Collection")
        data_layout = QVBoxLayout()

        self.collection_button = QPushButton("Start Collection")
        self.__all_layouts.append(self.collection_button)
        
        self.collection_button.clicked.connect(self.toggle_data_collection)
        data_layout.addWidget(self.collection_button)

        data_collection_group.setLayout(data_layout)
        right_panel.addWidget(data_collection_group)

        # 오류 로그 그룹 추가
        log_group = QGroupBox("Error Logs")
        log_layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

        log_group.setLayout(log_layout)
        right_panel.addWidget(log_group)

    def task_selected(self, task_name):
        """작업 선택 시 호출"""
        self.current_task = task_name
        self.work_start_time = datetime.now()
        self.send_robot_command(f"START_TASK:{task_name}")
        self.log_text.append(f"Task started: {task_name}")

    def update_work_time(self):
        """작업 시간 업데이트"""
        if self.work_start_time and self.current_task:
            elapsed = datetime.now() - self.work_start_time
            hours = int(elapsed.total_seconds() // 3600)
            minutes = int((elapsed.total_seconds() % 3600) // 60)
            seconds = int(elapsed.total_seconds() % 60)
            self.work_time_label.setText(f"작업 시간: {hours:02d}:{minutes:02d}:{seconds:02d}")

    def toggle_data_collection(self):
        """데이터 수집 시작/중지"""
        self.is_collecting_data = not self.is_collecting_data
        
        if self.is_collecting_data:
            self.collection_button.setText("Stop Collection")
            self.log_text.append("Data collection started")
            
            # 타이머 생성 및 시작
            if not hasattr(self, 'collection_timer'):
                self.collection_timer = QTimer()
                self.collection_timer.timeout.connect(self.save_image)
            
            # 1초(1000ms)마다 이미지 저장
            self.collection_timer.start(1000)
        else:
            self.collection_button.setText("Start Collection")
            self.log_text.append("Data collection stopped")
            
            # 타이머 중지
            if hasattr(self, 'collection_timer'):
                self.collection_timer.stop()
    
    def save_image(self):
        """주기적으로 호출되어 이미지를 저장하는 함수"""
        current_pixmap = self.robot_camera_label.pixmap()
        if current_pixmap:
            current_image = current_pixmap.toImage()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"{self.data_collection_dir}/image_{timestamp}.png"
            print(filepath)
            if current_image.save(filepath):
                print(f"이미지 저장 성공: {filepath}")
            else:
                print("이미지 저장 실패")

    def handle_error(self, error_msg):
        """오류 처리"""
        self.emergency_stop()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] ERROR: {error_msg}"
        self.log_text.append(log_entry)
        self.error_logs.append(log_entry)

    def update_robot_camera_display(self, cv_image):
        """로봇 카메라 디스플레이 업데이트"""
        try:
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # 레이블 크기 가져오기
            label_size = self.robot_camera_label.size()

            # 이미지를 레이블 크기에 맞게 스케일링하되 비율 유지
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(
                label_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.robot_camera_label.setPixmap(scaled_pixmap)
            self.robot_camera_label.setAlignment(Qt.AlignCenter)
        except Exception as e:
            self.get_logger().error(f'Failed to update robot camera display: {str(e)}')

    def update_global_camera_display(self, cv_image):
        """글로벌 카메라 디스플레이 업데이트"""
        try:
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # 레이블 크기 가져오기
            label_size = self.global_camera_label.size()

            # 이미지를 레이블 크기에 맞게 스케일링하되 비율 유지
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(
                label_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.global_camera_label.setPixmap(scaled_pixmap)
            self.global_camera_label.setAlignment(Qt.AlignCenter)
        except Exception as e:
            self.get_logger().error(f'Failed to update global camera display: {str(e)}')

    def update_joint_display(self, joint_positions):
        """Update joint position display"""
        # Implementation depends on your UI design
        pass

    def update_status(self):
        """Regular status updates"""
        pass

###################################################

def login_process():
    # 로그인 처리
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    login = LoginDialog()
    if login.exec_() != QDialog.Accepted:
        print("[INFO] Login Failed!")
        sys.exit()
    else:
        print("[INFO] Login Success!")

def main(args=None):
    login_process()
    rclpy.init(args=args)
    node = RobotControlUINode()
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.app.processEvents()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()