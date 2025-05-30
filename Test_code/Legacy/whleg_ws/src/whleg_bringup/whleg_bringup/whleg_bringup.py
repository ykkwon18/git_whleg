import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class BringupNode(Node):
    def __init__(self):
        super().__init__('whleg_bringup')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # OpenRB-150과 USB 시리얼 연결
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
        self.get_logger().info("✅ OpenRB-150 연결 완료")

    def cmd_vel_callback(self, msg):
        """ `/cmd_vel` 메시지를 받아서 시리얼로 전송 """
        # rad/s
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # 문자열 형태로 전송 (ex. '200.0 -200.0 300.0' )
        data = f"{linear_x} {linear_y} {angular_z}\n"
        self.ser.write(data.encode('utf-8'))
        
        self.get_logger().info(f"📤 전송된 값 - linear_x: {linear_x}")
        self.get_logger().info(f"📤 전송된 값 - linear_y: {linear_y}")
        self.get_logger().info(f"📤 전송된 값 - angular_z: {angular_z}")

def main(args=None):
    rclpy.init(args=args)
    node = BringupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

