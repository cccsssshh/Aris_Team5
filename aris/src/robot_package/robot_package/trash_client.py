import rclpy as rp
from rclpy.node import Node
from interface_package.srv import Check
from std_msgs.msg import Bool
from xarm.wrapper import XArmAPI
from robot_package.scripts import RobotMain
import time

class trash_client(Node):
    
    def __init__(self):
        super().__init__('trash_client')
    
        self.client = self.create_client(Check, 'get_coord')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = Check.Request()
        self.timer_period = 5
        self.timer = self.create_timer(self.timer_period, self.send_request)
        self.future = None
        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)
        self.robot_active = False
        self.robot_status_subscriber = self.create_subscription(
            Bool,
            'robot_to_kiosk_status',
            self.robot_status_callback,
            10
        )
        
    def robot_status_callback(self, msg):
        self.robot_active = msg.data
     
    def send_request(self):    
        if not self.robot_active:
            self.request.signal = 1
            self.future = self.client.call_async(self.request)
            self.get_logger().info('Request sent')
    
    def check_response(self):
        if self.future and self.future.done():
            if self.robot_active:
                self.get_logger().info(f'self.robot_active : {self.robot_active}')
                self.get_logger().info('RobotToKiosk 노드가 활성 상태입니다. 요청을 처리하지 않습니다.')
            else:
                try:
                    response = self.future.result()
                    self.get_logger().info(f'Result: x={response.x}, y={response.y}')
                    self.robot_main.trash(response.x, response.y)
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
            self.future = None

def main(args=None):
    rp.init(args=args)
    coordinate_client = trash_client()  # 클라이언트 노드 생성 
    
    while rp.ok():
        rp.spin_once(coordinate_client, timeout_sec=1.0)
        coordinate_client.check_response()
        time.sleep(1)  # 적절한 주기로 대기
        
    coordinate_client.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
