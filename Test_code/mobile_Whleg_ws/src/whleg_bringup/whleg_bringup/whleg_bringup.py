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

		# 여러 OpenRB-150 보드와 연결
		port_list = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3']
		self.serial_ports = []
		self.connected_ports = []
		self.failed_ports = []

		for port in port_list:
			try:
				ser = serial.Serial(port, 57600, timeout=1)
				self.serial_ports.append(ser)
				self.connected_ports.append(port)
			except serial.SerialException as e:
				self.failed_ports.append(port)

		# 결과 요약 출력
		if self.connected_ports:
			self.get_logger().info("✅ 연결된 포트:")
			for port in self.connected_ports:
				self.get_logger().info(f"  - {port}")
		if self.failed_ports:
			self.get_logger().warn("⚠️ 연결 실패한 포트:")
			for port in self.failed_ports:
				self.get_logger().warn(f"  - {port}")

		if not self.serial_ports:
			self.get_logger().fatal("❌ OpenRB-150 보드가 하나도 연결되지 않았습니다. 노드를 종료합니다.")
			rclpy.shutdown()

	def cmd_vel_callback(self, msg):
		""" `/cmd_vel` 메시지를 받아서 모든 시리얼 포트로 전송 """
		linear_x = msg.linear.x
		linear_y = msg.linear.y
		angular_z = msg.angular.z

		data = f"{linear_x} {linear_y} {angular_z}\n"
		encoded_data = data.encode('utf-8')

		for ser in self.serial_ports:
			try:
				ser.write(encoded_data)
			except serial.SerialException as e:
				self.get_logger().error(f"📛 전송 오류: {e}")

		self.get_logger().info(f"📤 전송된 값 - linear_x: {linear_x}")
		self.get_logger().info(f"📤 전송된 값 - linear_y: {linear_y}")
		self.get_logger().info(f"📤 전송된 값 - angular_z: {angular_z}")
		
	def send_stop_command(self):
		"""모든 시리얼 포트에 '0 0 0' 명령 전송"""
		stop_data = "0 0 0\n".encode('utf-8')
		for ser in self.serial_ports:
			try:
				ser.write(stop_data)
			except serial.SerialException as e:
				self.get_logger().error(f"❌ 정지 명령 전송 실패: {e}")
		self.get_logger().info("🛑 정지 명령 '0 0 0' 전송 완료")

def main(args=None):
	rclpy.init(args=args)
	node = BringupNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

