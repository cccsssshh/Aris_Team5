import rclpy
from rclpy.lifecycle import LifecycleNode, State
from rclpy.node import Node
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
        # Initialize the robot arm and related components
        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)
        self.srv4 = self.create_service(Trigger, "Greet", self.trigger_callback)
        self.response = Trigger.Response()
        self.status_publisher = self.create_publisher(Bool, 'robot_to_kiosk_status', 10)

    def trigger_callback(self, request, response):
        self.get_logger().info('Received Trigger request')
        self.handle_shaking_call()
        time.sleep(5)
        response.success = True
        response.message = "Trigger handled successfully"
        return response

    def handle_shaking_call(self):
        self.handle_shaking()
        self.get_logger().info('Handling shaking...')

    def ice_robot_callback(self, request, response):
        """Handle service requests."""
        menu = request.menu.lower()
        shaking = request.shaking.lower()
        half = request.half.lower()
        tracking = request.tracking.lower()
        self.get_logger().info(f'request : {request}')
        
        self.handle_menu(menu, tracking, shaking)
        
        if half == "half":
            self.handle_half()

        response.success = True
        self.get_logger().info('Service success!!')
        return response

    def handle_tracking(self):
        """Handle tracking actions."""
        self.robot_main.run_robot_arm_tracking()
        self.get_logger().info('Tracking!!!!!!!!!!!!!')

    def handle_shaking(self):
        """Handle shaking actions."""
        self.robot_main.motion_greet()
        self.get_logger().info('Hello!!!!!!!!!!!!!')

    def handle_half(self):
        """Handle half requests."""
        self.robot_main.final_run_half()
        self.get_logger().info('Half icecream!!!!!!!!!!!!!')

    def publish_status(self):
        """Publish the node's status."""
        msg = Bool()
        msg.data = self.get_lifecycle_state().id == State.ACTIVE
        self.status_publisher.publish(msg)
        self.get_logger().info(f'msg : {msg}')

    def on_configure(self, state):
        self.get_logger().info('RobotToKiosk node is now CONFIGURING.')
        return State.TRANSITION_CALLBACK

    def on_activate(self, state):
        self.get_logger().info('RobotToKiosk node is now ACTIVE.')
        self.publish_status()
        self.timer = self.create_timer(1.0, self.publish_status)
        return State.TRANSITION_CALLBACK

    def on_deactivate(self, state):
        self.get_logger().info('RobotToKiosk node is now INACTIVE.')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        return State.TRANSITION_CALLBACK

    def on_shutdown(self, state):
        self.get_logger().info('RobotToKiosk node is shutting down.')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        return State.TRANSITION_CALLBACK

    def handle_menu(self, menu, tracking, shaking):
        """Handle menu-related actions."""
        actions = {
            "banana": self.robot_main.run_banana,
            "choco": self.robot_main.run_choco,
            "berry": self.robot_main.run_strawberry,
            "affogato": self.robot_main.apocato,
            "아포가토": self.robot_main.apogato_bluetooth
        }
        
        if menu in actions:
            actions[menu]()
            time.sleep(1)
            
            if menu == "banana":
                if tracking == "serving":
                    self.robot_main.tracking_a_banana()
                    time.sleep(1)
                    self.robot_main.run_robot_arm_tracking()
                    time.sleep(1)
                    self.robot_main.speak("아이스크림을 가져가 주세요")
                elif shaking == "hello":
                    self.handle_shaking()
            
            elif menu == "choco":
                if tracking == "serving":
                    self.robot_main.tracking_b_choco()
                    time.sleep(1)
                    self.robot_main.run_robot_arm_tracking()
                    time.sleep(1)
                    self.robot_main.speak("아이스크림을 가져가 주세요")
                elif shaking == "hello":
                    self.handle_shaking()

            elif menu == "berry":
                if tracking == "serving":
                    self.robot_main.tracking_c_strawberry()
                    time.sleep(1)
                    self.robot_main.run_robot_arm_tracking()
                    time.sleep(1)
                    self.robot_main.speak("아이스크림을 가져가 주세요")
                    
                elif shaking == "hello":
                    self.handle_shaking()
            
            self.get_logger().info(f'{menu}!!!!!!!!!!!!!')

def main(args=None):
    rclpy.init(args=args)
    node = RobotToKiosk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()