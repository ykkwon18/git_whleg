# 최종 수정일 - 2025.06.13
# 4축 실제 구동을 위한 BringupNode (cmd_vel에서 모드명령 제거)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import serial
import sys
import select
import termios
import tty

class BringupNode(Node):
	def __init__(self):
		super().__init__('whleg_bringup')

		self.create_subscription(Twist, '/cmd_vel', self.send_serial_commands, 10)                    # /cmd_vel 구독
		self.create_subscription(String, '/driving_mode', self.driving_mode_callback, 10)             # /driving_mode 구독
		self.create_subscription(Int32, '/power', self.power_callback, 10)                            # /power 구독 추가

		self.time_publisher = self.create_publisher(Int32, '/time', 10)                               # /time 퍼블리시
		self.create_timer(1.0, self.publish_time)

		port_list = ['/dev/OpenRB150_1', '/dev/OpenRB150_3', '/dev/OpenRB150_5', '/dev/OpenRB150_7']
		self.serial_ports = []
		self.connected_ports = []
		self.failed_ports = []

		for port in port_list:
			try:
				ser = serial.Serial(port, 57600, timeout=1)
				self.serial_ports.append(ser)
				self.connected_ports.append(port)
			except serial.SerialException:
				self.failed_ports.append(port)

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

		self.sent_initial_time = False
		self.current_power = 1
		self.current_mode = "Wheel"  # 기본 모드

	def publish_time(self):
		sec = int(self.get_clock().now().seconds_nanoseconds()[0])
		msg = Int32()
		msg.data = sec
		self.time_publisher.publish(msg)

	def driving_mode_callback(self, msg: String):
		if msg.data in ["Wheel", "Leg"]:
			if msg.data != self.current_mode:  # 실제로 모드가 변경될 때만 전송
				self.current_mode = msg.data
				self.get_logger().info(f"🧭 주행 모드 변경: {self.current_mode}")

				mode_value = 1 if self.current_mode == "Wheel" else 0
				cmd_mode = f"2 {mode_value}\n".encode('utf-8')

				for ser in self.serial_ports:
					try:
						ser.write(cmd_mode)
					except serial.SerialException as e:
						self.get_logger().error(f"📛 모드 전송 오류 (driving_mode_callback): {e}")

				self.get_logger().info(f"📤 모드 전송 (driving_mode_callback): {cmd_mode.decode().strip()}")
			else:
				self.get_logger().info(f"🧭 동일 모드 수신, 전송 생략: {self.current_mode}")

		else:
			self.get_logger().warn(f"⚠️ 잘못된 모드 수신: '{msg.data}'")

	def power_callback(self, msg: Int32):
		if msg.data != self.current_power:
			self.current_power = msg.data
			cmd_power = f"9\n".encode('utf-8')
			for ser in self.serial_ports:
				try:
					ser.write(cmd_power)
				except serial.SerialException as e:
					self.get_logger().error(f"⚡ 전원 전송 오류: {e}")
			self.get_logger().info(f"🔌 전원 상태 전송: {cmd_power.decode().strip()}")

	def send_serial_commands(self, msg: Twist):
		if not self.sent_initial_time:
			sec = int(self.get_clock().now().seconds_nanoseconds()[0])
			time_str = f"{sec:04d}"[-4:]
			cmd_time = f"0 {time_str}\n".encode('utf-8')
			for ser in self.serial_ports:
				try:
					ser.write(cmd_time)
				except serial.SerialException as e:
					self.get_logger().error(f"📛 시간 전송 오류: {e}")
			self.get_logger().info(f"⏱ 초기 시간 전송: {cmd_time.decode().strip()}")
			self.sent_initial_time = True

		linear_x = int(msg.linear.x * 100)
		angular_z = int(msg.angular.z * 100)
		cmd_velocity = f"1 {linear_x:+03d} {angular_z:+03d}\n".encode('utf-8')

		for ser in self.serial_ports:
			try:
				ser.write(cmd_velocity)
			except serial.SerialException as e:
				self.get_logger().error(f"📛 속도 전송 오류: {e}")

		#self.get_logger().info(f"📤 속도 전송: {cmd_velocity.decode().strip()}")

	def send_stop_command(self):
		stop_cmd = "1 +000 +000\n".encode('utf-8')
		for ser in self.serial_ports:
			try:
				ser.write(stop_cmd)
			except serial.SerialException as e:
				self.get_logger().error(f"❌ 정지 명령 전송 실패: {e}")
		self.get_logger().info("🛑 정지 명령 '1 +000 +000' 전송 완료")

# esc 또는 Ctrl+C 감지
def check_esc_pressed():
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setcbreak(fd)
		if select.select([sys.stdin], [], [], 0.1)[0]:
			key = sys.stdin.read(1)
			return key == '\x1b'  # ESC
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return False

def main(args=None):
	rclpy.init(args=args)
	node = BringupNode()

	try:
		while rclpy.ok():
			rclpy.spin_once(node, timeout_sec=0.1)
			if check_esc_pressed():
				node.get_logger().info("⛔ ESC 키 감지됨, 정지 명령 전송")
				node.send_stop_command()
				break
	except KeyboardInterrupt:
		node.get_logger().info("🛑 Ctrl+C 감지됨, 정지 명령 전송")
		node.send_stop_command()
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

