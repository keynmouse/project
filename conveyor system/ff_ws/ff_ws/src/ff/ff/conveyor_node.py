# test/test_conveyor_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool
import serial
import time

# 에러 코드
# 0: 에러 없음
# 1: 컨베이어 연결 에러
# 2: USB 연결 에러

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('test_conveyor_node')
        
        # Publishers
        # self.conveyor_state_pub = self.create_publisher(Twist, '/conveyor/state', 10)
        self.error_code = self.create_publisher(Int32, '/conveyor/error', 10)

        self.publisher = self.create_publisher(String, '/conveyor/state', 10)
        
        # 시리얼 포트 설정
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',  # 아두이노 포트
            baudrate=115200,        # 보드레이트
            timeout=0.001
        )
        
        # Subscribers
        self.conveyor_cmd_sub = self.create_subscription(Twist, '/conveyor/command', 
                                                       self.conveyor_cmd_callback, 10)
        self.emergency_pub = self.create_subscription(Bool, '/emergency_stop', 
                                                   self.emergency_observer_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.publish_conveyor_state) # 10Hz
        
        # Test state
        self.conveyor_speed = 0.0
        self.errro_code = 0

        self.get_logger().info('Conveyor node started')

    def conveyor_cmd_callback(self, msg):
        self.get_logger().info(f'Received conveyor command - speed: {msg.linear.x}')
        self.conveyor_speed = msg.linear.x
        self.conveyor_controller(self.conveyor_speed)

    def emergency_observer_callback(self, msg):
        print(msg.data)
        if msg.data:
            self.conveyor_controller(-1.0)
        else:
            pass
            
        
    def conveyor_controller(self, speed):
        # if speed > 0:
        #     # 양의 방향
        #     self.publish_error_state(0)
        #     pass
        # elif speed < 0:
        #     # 음의 방향
            
        #     pass
        # else: 
        #     # 정지
        #     self.publish_error_state(2)
        #     pass
        try:
            command = str(speed)
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().info(f'명령어 전송: {command}')
        except Exception as e:
            self.get_logger().error(f'명령어 전송 오류: {str(e)}')

    def publish_conveyor_state(self):
        
        try: 
            if self.serial_port.in_waiting:
                try:
                    # 시리얼 데이터 읽기
                    serial_data = self.serial_port.readline().decode('utf-8').strip()[-1]

                    # 메시지 생성 및 발행
                    msg = String()

                    if serial_data == '_':
                        state = 'Conveyor_Run'
                    elif serial_data == '.':
                        state = 'Conveyor_Ready'
                    elif serial_data == 's':
                        state = 'Starting'

                    msg.data = state
                    self.publisher.publish(msg)

                    # 로그 출력
                    # self.get_logger().info(f'수신된 데이터: {serial_data}, 상태: {state}')

                except Exception as e:
                    self.get_logger().error(f'데이터 읽기 오류: {str(e)}')
                    # 1: 컨베이어 연결 에러
                    self.publish_error_state(1)

        except Exception as e:
            self.get_logger().error(f'USB 연결 오류: {str(e)}')
            # 2: USB 연결 에러
            self.publish_error_state(2)



    def publish_error_state(self, debug=0):

        msg = Int32()
        msg.data = self.error_handler() if debug == 0 else debug
        self.error_code.publish(msg)

    def error_handler(self):
        self.errro_code = 0
        return self.errro_code

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    
    try:
        while rclpy.ok():
            # 콜백 한 번 처리
            rclpy.spin_once(node)
            # 테스트용
            # node.publish_error_state(1)
            # node.publish_error_state()
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()