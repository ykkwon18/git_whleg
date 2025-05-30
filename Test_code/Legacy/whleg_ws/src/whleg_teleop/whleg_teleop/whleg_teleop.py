import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('whleg_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("키보드 입력 시작: W(전진), S(후진), A(좌회전), D(우회전), X(종료), Q(노드 종료)")
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def run(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'q':
                        break

                    msg = Twist()
                    if key == 'w':
                        msg.linear.x = 200.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                    elif key == 's':
                        msg.linear.x = -200.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                    elif key == 'a':
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = 200.0
                    elif key == 'd':
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = -200.0
                    elif key == 'x':
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                        
                    else:
                        continue

                    self.publisher_.publish(msg)
                    self.get_logger().info(f'퍼블리시된 속도 값: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

