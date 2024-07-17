import rclpy
from rclpy.node import Node
from robot_service.srv import IceRobot
from robot_package.scripts import RobotMain

# ================ service ================ #
class RobotToKiosk(Node):
    def __init__(self):
        super().__init__('RobotToKiosk')
        self.srv = self.create_service(IceRobot, 'IceRobot', self.ice_robot_callback)

    def ice_robot_callback(self, request, response):
        menu = request.menu.lower()  # 메뉴 이름을 소문자로 변환하여 비교
        
        if menu == "banana":
            for _ in range(request.quantity):
                RobotMain.motion_home()
                # RobotMain.run_banana()  # 바나나 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('banana!!!!!!!!!!!!!')
                
        elif menu == "choco":
            for _ in range(request.quantity):
                # RobotMain.run_choco()  # 초코 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('choco!!!!!!!!!!!!!')

        elif menu == "strawberry":
            for _ in range(request.quantity):
                # RobotMain.run_strawberry()  # 딸기 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('strawberry!!!!!!!!!!!!!')

        response.success = True  # 응답 결과 설정
        self.get_logger().info('IceRobot success!!')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotToKiosk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
