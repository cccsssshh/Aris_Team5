import rclpy
from rclpy.node import Node
from robot_service.msg import RobotStatusInfo
from robot_package.scripts import RobotMain
from xarm.wrapper import XArmAPI

class RobotToAdminiSub(Node):

    def __init__(self):
        super().__init__('RobotToAdminiSub')
        self.subscription = self.create_subscription(
            RobotStatusInfo,
            'RobotStatusInfo',  # Publisher 노드에서 사용한 주제 이름
            self.callback,
            10  # 큐의 크기
        )
        self.subscription  # 구독자 객체를 유지하기 위해 인스턴스 변수로 할당

    def callback(self, msg):
        joints = (f'J1: {msg.j1:.2f}, J2: {msg.j2:.2f}, J3: {msg.j3:.2f}, '
                  f'J4: {msg.j4:.2f}, J5: {msg.j5:.2f}, J6: {msg.j6:.2f}')
        temperatures = (f'T1: {msg.temperature1}, T2: {msg.temperature2}, '
                        f'T3: {msg.temperature3}, T4: {msg.temperature4}, '
                        f'T5: {msg.temperature5}, T6: {msg.temperature6}')
        self.get_logger().info('Subscribing.......!!!!!!!!!!!!!!\n')
        self.get_logger().info(f'\n Joints: {joints}\n Temperatures: {temperatures}')

def main(args=None):
    rclpy.init(args=args)
    admin_to_robot = RobotToAdminiSub()
    rclpy.spin(admin_to_robot)
    admin_to_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
