import rclpy
from rclpy.node import Node
from robot_service.srv import IceRobot

# ================ client ================ #
class KioskToRobot(Node):
    def __init__(self):
        super().__init__('KioskToRobot')
        self.client = self.create_client(IceRobot, 'IceRobot')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = IceRobot.Request()

    def send_request(self, menu, quantity):
        self.request.menu = menu
        self.request.quantity = quantity
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = KioskToRobot()
    
    node.send_request("banana", 1)  # 예시: 바나나 메뉴 1개 요청
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                if response.success:
                    node.get_logger().info('Result of IceRobot: success')
                else:
                    node.get_logger().info('Result of IceRobot: failed')
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()