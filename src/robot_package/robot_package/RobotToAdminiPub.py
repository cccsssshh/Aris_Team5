import rclpy
from rclpy.node import Node
from robot_service.msg import RobotStatusInfo
from robot_package.scripts import RobotMain
from xarm.wrapper import XArmAPI


class RobotToAdminiPub(Node):

    def __init__(self):
        super().__init__('RobotToAdminiPub')
        self.publisher_ = self.create_publisher(RobotStatusInfo, 'RobotStatusInfo', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)


    def timer_callback(self):
        joint_angles, temperature = self.robot_main.joint_state_ros()
        
        msg = RobotStatusInfo()
        msg.j1 = joint_angles[0]
        msg.j2 = joint_angles[1]
        msg.j3 = joint_angles[2]
        msg.j4 = joint_angles[3]
        msg.j5 = joint_angles[4]
        msg.j6 = joint_angles[5]
        
        msg.temperature1 = temperature[0]
        msg.temperature2 = temperature[1]
        msg.temperature3 = temperature[2]
        msg.temperature4 = temperature[3]
        msg.temperature5 = temperature[4]
        msg.temperature6 = temperature[5]
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Joints [{msg.j1}, {msg.j2}, {msg.j3}, {msg.j4}, {msg.j5}, {msg.j6}]')
        self.get_logger().info(f'Publishing: Temperatures [{msg.temperature1}, {msg.temperature2}, {msg.temperature3}, {msg.temperature4}, {msg.temperature5}, {msg.temperature6}]')
        self.get_logger().info('Publishing......!!!!!!!!')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publish_robot = RobotToAdminiPub()

    rclpy.spin(publish_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publish_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()