# test/test_camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time


# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

Type = 'DICT_5X5_100'

d = np.array([[-0.00746391,  0.06038115, -0.0001621,   0.00045486, -0.04346805]])
k = np.array([[663.53906823,   0.,         620.95123764], 
                [  0.,         664.13958877, 353.79669005],
                [  0.,           0.,           1.        ]])



def get_matrix(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[Type])
    arucoParams = cv2.aruco.DetectorParameters()
    
    # Create detector object
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    
    # Use detector instead of cv2.aruco.detectMarkers
    corners1, ids1, rejected = detector.detectMarkers(gray)
    
    if ids1 is not None:
        # 새로운 방식의 pose estimation
        objPoints = np.array([[0, 0, 0], [0.105, 0, 0], [0.105, 0.105, 0], [0, 0.105, 0]], dtype=np.float32)
        rvecs1 = []
        tvecs1 = []
        
        for corner in corners1:
            retval, rvec, tvec = cv2.solvePnP(objPoints, corner, k, d)
            rvecs1.append(rvec)
            tvecs1.append(tvec)
        
        rvecs1 = np.array(rvecs1)
        tvecs1 = np.array(tvecs1)
        R, _ = cv2.Rodrigues(rvecs1[0])
        return ids1, rvecs1, R, tvecs1
    else:
        return None, None, None, None


unit_x = np.array([[1.0], [0.0], [0.0]])
unit_y = np.array([[0.0], [1.0], [0.0]])
unit_z = np.array([[0.0], [0.0], [1.0]])

class GlobalCameraNode(Node):
    def __init__(self):
        super().__init__('global_camera_node')
        
        # Publishers
        self.robot_camera_pub = self.create_publisher(Image, '/robot/camera/image_raw', 10)
        self.global_camera_pub = self.create_publisher(Image, '/global/camera/image_raw', 10)

        # 파라미터 선언 (기본값: 0)
        self.declare_parameter('cam_index', 0)
        # 파라미터 값 가져오기
        self.cam_index = self.get_parameter('cam_index').value
        self.get_logger().info(f'Cam Index: {self.cam_index}')
        
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
        
        self.get_logger().info('Global Camera node started')



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
        
        # Add text and timestamp
        cv2.putText(frame, text, (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (50, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (self.cam_width,self.cam_height), 1, (self.cam_width,self.cam_height))
        frame = cv2.undistort(frame, k, d, None, newcameramtx)
        
        ids, rvecs, R, tvecs = get_matrix(frame)
        if ids is not None:
            for revc, tvec, id in zip(rvecs, tvecs, ids):
                
                cv2.drawFrameAxes(frame, k, d, revc, tvec, 0.1)
                # 마커의 3D 점을 2D 이미지 평면으로 프로젝션
                point_3d = np.float32([[0, 0, 0]]).reshape(-1, 3)
                point_2d, _ = cv2.projectPoints(point_3d, revc, tvec, k, d)

                # id 텍스트 위치 계산 (마커 중심점)
                center_x = int(point_2d[0][0][0])
                center_y = int(point_2d[0][0][1])

                # id 텍스트 그리기
                cv2.putText(frame, f"ID: {id[0]}", (center_x - 20, center_y - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.get_logger().info(f'id: {id}, tvec : {tvec}')

            x_T = np.dot(R, unit_x)
            y_T = np.dot(R, unit_y)
            z_T = np.dot(R, unit_z)

            axis_size = 100
            axis_center = (self.cam_width//2, self.cam_height//2)
            cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*x_T[0]), int(axis_center[1] + axis_size*x_T[1])), (0, 0, 255), 2)
            cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*y_T[0]), int(axis_center[1] + axis_size*y_T[1])), (0, 255, 0), 2)
            cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*z_T[0]), int(axis_center[1] + axis_size*z_T[1])), (255, 0, 0), 2)

        return frame

    def publish_camera_images(self):
        # Create Camera images topic
        global_image = self.get_camera_image("Global Camera")
        
        # Convert to ROS messages
        global_msg = self.bridge.cv2_to_imgmsg(global_image, "bgr8")
        
        # Publish
        self.global_camera_pub.publish(global_msg)


    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = GlobalCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()