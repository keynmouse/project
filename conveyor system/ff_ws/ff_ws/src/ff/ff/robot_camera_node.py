# test/test_camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from ff.yolo_detector import YOLODetector
import json

class RobotCameraNode(Node):
    def __init__(self):
        super().__init__('robot_camera_node')

        # YOLO 초기화
        self.yolo = YOLODetector(
            model_path='./models/best.pt',  # Use YOLOv8 nano model
            conf_threshold=0.5
        )
        # 파라미터 선언 (기본값: 0)
        self.declare_parameter('cam_index', 0)
        # 파라미터 값 가져오기
        self.cam_index = self.get_parameter('cam_index').value
        self.get_logger().info(f'Cam Index: {self.cam_index}')

        # 최적화
        self.last_publish_time = self.get_clock().now()
        self.publish_interval = 0.1  # 1초 간격으로 publish


        # Publishers
        self.compressed_pub = self.create_publisher(CompressedImage, '/robot/camera/image_raw/compressed', 10)
        
        # Service Server
        self.detection_service = self.create_service(
            Trigger,
            '/get_detections',
            self.get_detections_callback
        )


        self.latest_frame = None


        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        # ls -l /dev/video* 의 연속된 2개의 숫자중 앞선 숫자가 카메라 인덱스
        self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다!')
            return
            
        # Set camera properties
        self.cam_width = 1280
        self.cam_height = 720
        # 카메라 해상도 이슈로 추가
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_height)
        # def print_camera_properties(self):
        #     resolutions = [
        #         (640, 480),
        #         (1280, 720),
        #         (1920, 1080)
        #     ]

        #     for width, height in resolutions:
        #         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        #         actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        #         actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        #         self.get_logger().info(f'Testing {width}x{height} -> Got: {actual_width}x{actual_height}')

        # print_camera_properties(self)

        # # 설정된 해상도 확인
        # actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        # actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        # self.get_logger().info(f'Requested resolution: {self.cam_width}x{self.cam_height}')
        # self.get_logger().info(f'Actual resolution: {actual_width}x{actual_height}')

        # Timer
        self.create_timer(0.033, self.publish_camera_images) # 30Hz
        
        self.get_logger().info('Robot Camera node started')


    def get_detections_callback(self, request, response):
        """
        서비스 콜백 함수: 현재 프레임의 디텍션 결과를 JSON 문자열로 반환
        """

        # 이미지 크기 설정
        IMAGE_WIDTH = 1280
        IMAGE_HEIGHT = 720

        normalized_grid_start=(0.1125, 0.075)
        normalized_grid_end=(0.9142, 0.975)
        
        self.get_logger().info('Detection service called')
        
        if self.latest_frame is None:
            response.success = False
            response.message = "No frame available"
            return response
        
        IMAGE_HEIGHT, IMAGE_WIDTH = self.latest_frame.shape[:2]

        grid_start = (int(normalized_grid_start[0] * IMAGE_WIDTH), 
                 int(normalized_grid_start[1] * IMAGE_HEIGHT))
        grid_end = (int(normalized_grid_end[0] * IMAGE_WIDTH), 
               int(normalized_grid_end[1] * IMAGE_HEIGHT))
        
        # 그리드 크기 계산
        grid_width = grid_end[0] - grid_start[0]
        grid_height = grid_end[1] - grid_start[1]
        cell_width = grid_width // 3
        cell_height = grid_height // 2

        # 원점 표시 - 그리드의 좌측 상단
        origin_x = grid_start[0]
        origin_y = grid_start[1]

        print("detection start")

        # Run detection on latest frame
        detections, _ = self.yolo.detect_image(self.latest_frame)


        bbox_coordinates = [[item["bbox"], item["class_name"]] for item in detections]

        yolo_detections = []
        for bbox in bbox_coordinates:
            x1, y1, x2, y2 = bbox[0]
            x_center = (x1 + x2) / 2 / IMAGE_WIDTH
            y_center = (y1 + y2) / 2 / IMAGE_HEIGHT
            width = (x2 - x1) / IMAGE_WIDTH
            height = (y2 - y1) / IMAGE_HEIGHT
            yolo_detections.append([x_center, y_center, width, height, bbox[1]])
 
        # Convert detections to required format
        result = []
        # YOLO 감지 결과 표시
        for detection in yolo_detections:
            # YOLO 좌표를 이미지 좌표로 변환
            x_center = int(detection[0] * IMAGE_WIDTH)
            y_center = int(detection[1] * IMAGE_HEIGHT)
            width = int(detection[2] * IMAGE_WIDTH)
            height = int(detection[3] * IMAGE_HEIGHT)

            # 그리드 단위로 변환된 좌표 계산
            grid_unit_x = (x_center - origin_x) / cell_width
            grid_unit_y = (y_center - origin_y) / cell_height

            # 좌표 텍스트 표시
            if grid_unit_x >= 0 and grid_unit_y >= 0:
                result.append({
                    "position" : (grid_unit_x, grid_unit_y),
                    "color" : detection[4],
                })

        print("Json start")
        # Convert to JSON string
        json_result = json.dumps(result)
        
        response.success = True
        response.message = json_result
        return response


    def create_test_image(self, text):
        image = np.zeros((self.cam_height, self.cam_width, 3), dtype=np.uint8)

        # Add text and timestamp
        cv2.putText(image, text, (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(image, time.strftime("%H:%M:%S"), (50, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        
        return image

    def get_camera_image(self, text):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('카메라에서 프레임을 읽을 수 없습니다!')
            return np.zeros((self.cam_height, self.cam_width, 3), dtype=np.uint8)
        self.latest_frame = frame
        # Add text and timestamp
        cv2.putText(frame, text, (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (50, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Run detection
        _, frame = self.yolo.detect_image(frame)

        # Print detections
        # for det in detections:
        #     print(f"Detected {det['class_name']} with confidence {det['confidence']:.2f}")
        

        return frame

    def publish_camera_images(self):
        current_time = self.get_clock().now()
        # 마지막 publish 후 publish_interval 시간이 지났는지 확인
        if (current_time - self.last_publish_time).nanoseconds / 1e9 < self.publish_interval:
            return

        robot_image = self.get_camera_image("Robot Camera")
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = current_time.to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', robot_image)[1]).tobytes()
        self.compressed_pub.publish(compressed_msg)

        self.last_publish_time = current_time


    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = RobotCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()