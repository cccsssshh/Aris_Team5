import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from interface_package.srv import IceRobot
from robot_package.scripts import RobotMain
from xarm.wrapper import XArmAPI
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import time

class RobotToKiosk(LifecycleNode):
    def __init__(self):
        super().__init__('RobotToKiosk')
        self.srv = self.create_service(IceRobot, 'IceRobot', self.ice_robot_callback)
        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)
        self.srv4 = self.create_service(Trigger, "Greet", self.trigger_callback)
        self.status_publisher = self.create_publisher(Bool, 'robot_to_kiosk_status', 10)
        self.timer = None

    def trigger_callback(self, request, response):
        try:
            self.get_logger().info('Received Trigger request')
            self.handle_shaking_call()
            time.sleep(5)
            response.success = True
            response.message = "Trigger handled successfully"
        except Exception as e:
            self.get_logger().error(f'Error in trigger_callback: {e}')
            response.success = False
            response.message = str(e)
        return response

    def handle_shaking_call(self):
        self.handle_shaking()
        self.get_logger().info('Handling shaking...')

    def ice_robot_callback(self, request, response):
        try:
            menu = request.menu.lower()
            shaking = request.shaking.lower()
            half = request.half.lower()
            tracking = request.tracking.lower()
            self.get_logger().info(f'Received request: {request}')

            self.handle_menu(menu, tracking, shaking)

            if half == "half":
                self.handle_half()

            response.success = True
            self.get_logger().info('Service success!!')
        except Exception as e:
            self.get_logger().error(f'Error in ice_robot_callback: {e}')
            response.success = False
        return response

    def handle_tracking(self):
        self.robot_main.run_robot_arm_tracking()
        self.get_logger().info('Tracking started')

    def handle_shaking(self):
        self.robot_main.motion_greet()
        self.get_logger().info('Shaking initiated')

    def handle_half(self):
        self.robot_main.final_run_half()
        self.get_logger().info('Half ice cream process initiated')
        
    def handle_tracking_a(self):
        self.get_logger().info('Tracking tracking_a_banana')
        self.robot_main.tracking_a_banana()
        self.get_logger().info('Tracking tracking_a_banana')
        
    def handle_tracking_b(self):
        self.get_logger().info('Tracking tracking_b_choco')
        self.robot_main.tracking_b_choco()
        self.get_logger().info('Tracking tracking_b_choco')
        
    def handle_tracking_c(self):
        self.get_logger().info('Tracking tracking_c_strawberry')
        self.robot_main.tracking_c_strawberry()
        self.get_logger().info('Tracking tracking_c_strawberry')
        
    def gripper(self):
        try:
            code = self.arm.open_lite6_gripper()
            if code != 0:
                self.get_logger().error(f'Failed to open gripper, error code: {code}')
                return
            
            time.sleep(5)
            
            code = self.arm.stop_lite6_gripper()
            if code != 0:
                self.get_logger().error(f'Failed to stop gripper, error code: {code}')
                return

        except Exception as e:
            self.get_logger().error(f'Error in gripper operation: {e}')

# ========================= lifecycle node ========================= # 
    '''
    manual (terminal)
    setting
    1. ros2 run robot_package RobotToKiosk
    2. ros2 lifecycle set /RobotToKiosk configure

    active
    ros2 lifecycle set /RobotToKiosk activate

    deactive
    ros2 lifecycle set /RobotToKiosk deactivate

    shutdown
    ros2 lifecycle set /RobotToKiosk shutdown
    '''

    def publish_status(self, state):
        msg = Bool()
        if state == 'active':
            msg.data = True
        elif state == 'inactive':
            msg.data = False
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Published status: {msg.data}')

    def on_configure(self, state):
        try:
            self.get_logger().info('Entering CONFIGURE state.')
            self.get_logger().info('RobotToKiosk node is now CONFIGURING.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during CONFIGURE: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        try:
            self.get_logger().info('Entering ACTIVATE state.')
            self.get_logger().info('RobotToKiosk node is now ACTIVE.')
            self.publish_status('active')
            if self.timer:
                self.timer.cancel()
            # self.timer = self.create_timer(1.0, lambda: self.publish_status('active'))
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during ACTIVATE: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        try:
            self.get_logger().info('Entering DEACTIVATE state.')
            self.get_logger().info('RobotToKiosk node is now INACTIVE.')
            self.publish_status('inactive')
            # self.timer = self.create_timer(1.0, lambda: self.publish_status('inactive'))
            if self.timer:
                self.timer.cancel()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during DEACTIVATE: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        try:
            self.get_logger().info('Entering SHUTDOWN state.')
            self.get_logger().info('RobotToKiosk node is shutting down.')
            if self.timer:
                self.timer.cancel()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during SHUTDOWN: {e}')
            return TransitionCallbackReturn.FAILURE

    def handle_menu(self, menu, tracking, shaking):
        actions = {
            "banana": self.robot_main.run_banana,
            "choco": self.robot_main.run_choco,
            "berry": self.robot_main.run_strawberry,
            "affogato": self.robot_main.apocato,
            "아포가토": self.robot_main.apogato_bluetooth
        }
        
        try:
            if menu in actions:
                actions[menu]()
                time.sleep(1)

                if menu == "banana":
                    if tracking == "serving":
                        try:
                            self.handle_tracking_a()
                            time.sleep(1)
                            self.handle_tracking()
                            time.sleep(1)
                            self.robot_main.tracking_home()
                            self.robot_main.speak("아이스크림을 가져가 주세요")
                            time.sleep(3)
                            self.gripper()
                            time.sleep(1)
                            self.robot_main.motion_home()
                            
                        except Exception as e:
                            self.get_logger().error(f'Error during banana tracking/serving: {e}')
                    elif shaking == "hello":
                        self.handle_shaking()
                
                elif menu == "choco":
                    if tracking == "serving":
                        try:
                            self.handle_tracking_b()
                            time.sleep(1)
                            self.handle_tracking()
                            time.sleep(1)
                            self.robot_main.tracking_home()
                            self.robot_main.speak("아이스크림을 가져가 주세요")
                            time.sleep(3)
                            self.gripper()
                            time.sleep(1)
                            self.robot_main.motion_home()

                        except Exception as e:
                            self.get_logger().error(f'Error during choco tracking/serving: {e}')
                    elif shaking == "hello":
                        self.handle_shaking()

                elif menu == "berry":
                    if tracking == "serving":
                        try:
                            self.handle_tracking_c()
                            time.sleep(1)
                            self.handle_tracking()
                            time.sleep(1)
                            self.robot_main.tracking_home()
                            self.robot_main.speak("아이스크림을 가져가 주세요")
                            time.sleep(3)
                            self.gripper()
                            time.sleep(1)
                            self.robot_main.motion_home()
                            
                        except Exception as e:
                            self.get_logger().error(f'Error during berry tracking/serving: {e}')
                    elif shaking == "hello":
                        self.handle_shaking()
                
                self.get_logger().info(f'{menu} processed')

        except Exception as e:
            self.get_logger().error(f'Error in handle_menu: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotToKiosk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
