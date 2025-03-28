import sys
import select
import termios
import tty
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
	def __init__(self):
		super().__init__('whleg_teleop')
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		print("키보드 입력 시작: w/s/a/d 이동, q/e 대각선, Ctrl+C 정지, f: 속도 토글 (저속 <-> 고속)", flush=True)
		self.settings = termios.tcgetattr(sys.stdin)

		self.speed_linear = 1.0
		self.speed_angular_degree = 20.0
		self.fast_mode = False  # 토글 상태

		self.run()

	def run(self):
		tty.setraw(sys.stdin.fileno())
		try:
			while rclpy.ok():
				if select.select([sys.stdin], [], [], 0.1)[0]:
					key = sys.stdin.read(1)

					if key == '\x03':  # Ctrl+C
						# 정지 명령 전송
						stop_msg = Twist()
						stop_msg.linear.x = 0.0
						stop_msg.angular.z = 0.0
						self.publisher_.publish(stop_msg)
						print("Ctrl+C 감지됨. 종료합니다.", flush=True)
						break

					if key == 'f':
						self.fast_mode = not self.fast_mode
						if self.fast_mode:
							self.speed_linear *= 2
							self.speed_angular_degree *= 2
						else:
							self.speed_linear /= 2
							self.speed_angular_degree /= 2
						mode = "고속모드!" if self.fast_mode else "저속모드!"
						print(f"[{key}] → linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f} (rad/s)", flush=True)
						continue

					msg = Twist()
					angular_rad = math.radians(self.speed_angular_degree)

					if key == 'w':
						msg.linear.x = self.speed_linear
						msg.angular.z = 0.0
					elif key == 's':
						msg.linear.x = -self.speed_linear
						msg.angular.z = 0.0
					elif key == 'a':
						msg.angular.z = -angular_rad
						msg.linear.x = 0.0
					elif key == 'd':
						msg.angular.z = angular_rad
						msg.linear.x = 0.0
					elif key == 'x':
						msg.linear.x = 0.0
						msg.angular.z = 0.0
					elif key == 'e':
						msg.linear.x = self.speed_linear
						msg.angular.z = -angular_rad
					elif key == 'q':
						msg.linear.x = self.speed_linear
						msg.angular.z = angular_rad
					else:
						continue

					self.publisher_.publish(msg)
					print(f"[{key}] → linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}\r\n", flush=True)
					
		finally:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
	rclpy.init(args=args)
	try:
		TeleopNode()
	except KeyboardInterrupt:
		pass
	rclpy.shutdown()

if __name__ == '__main__':
	main()
