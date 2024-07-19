import rclpy
from rclpy.node import Node
from robot_service.srv import IceRobot
from robot_package.scripts import RobotMain
from xarm.wrapper import XArmAPI 

# ================ service ================ #
class RobotToKiosk(Node):
    def __init__(self):
        super().__init__('RobotToKiosk')
        self.srv = self.create_service(IceRobot, 'IceRobot', self.ice_robot_callback)

        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)

    
    
    def ice_robot_callback(self, request, response):
        menu = request.menu.lower()  # 메뉴 이름을 소문자로 변환하여 비교
    
        response.success = True  # 응답 결과 설정
        self.get_logger().info('IceRobot success!!')
        
        if menu == "banana":
            # for _ in range(request.order_num):
                # self.robot_main.motion_home()
            # self.robot_main.run_banana()  # 바나나 처리 동작을 quantity만큼 반복 실행
            self.get_logger().info('banana!!!!!!!!!!!!!')
                
        elif menu == "초코맛":
            # for _ in range(request.quantity):
                self.robot_main.run_choco()  # 초코 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('choco!!!!!!!!!!!!!')

        elif menu == "딸기맛":
            # for _ in range(request.quantity):
                self.robot_main.run_strawberry()  # 딸기 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('strawberry!!!!!!!!!!!!!')
                
        elif menu == "아포가토":
            # for _ in range(request.quantity):
                self.robot_main.apocato()  # 딸기 처리 동작을 quantity만큼 반복 실행
                self.get_logger().info('strawberry!!!!!!!!!!!!!')

        elif menu == "아포가토음성":
            self.robot_main.apogato_bluetooth()  # 딸기 처리 동작을 quantity만큼 반복 실행
            self.get_logger().info('strawberry!!!!!!!!!!!!!')

        if request.tracking:
            self.robot_main.run_robot_arm_tracking()  # kiosk는 client (요청을 함)  그 다음 robot은 service (요청 받고 작동해서 반응 보내줌)
            self.get_logger().info('tracking!!!!!!!!!!!!!')
        
        if request.half:
            self.robot_main.final_run_half()  # kiosk는 client (요청을 함)  그 다음 robot은 service (요청 받고 작동해서 반응 보내줌)
            self.get_logger().info('half icecream!!!!!!!!!!!!!')
            pass
        
        # response.success = True  # 응답 결과 설정
        # self.get_logger().info('IceRobot success!!')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotToKiosk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
