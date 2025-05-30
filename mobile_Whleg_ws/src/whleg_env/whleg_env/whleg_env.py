import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyrealsense2 as rs
import numpy as np
import cv2


class StepDetectionNode(Node):
    def __init__(self):
        super().__init__('whleg_env')

        # Constants
        self.CAMERA_HEIGHT = 0.13
        self.PIXEL_OFFSET = 60
        self.STEP_THRESH = 0.05
        self.ROI_HALF_W = 10
        self.SKIP_FRAMES = 5

        # Publisher
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)

        # RealSense init
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.frame_idx = 0
        self.current_mode = "Wheel"  # 초기 모드

        # 타이머로 루프 실행
        self.create_timer(1.0 / 15.0, self.process_frame)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return

        self.frame_idx += 1
        h, w = depth_frame.get_height(), depth_frame.get_width()
        cx = w // 2
        y_ground = h - 10
        y_front = max(0, y_ground - self.PIXEL_OFFSET)

        if self.frame_idx % self.SKIP_FRAMES == 0:
            intrin = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
            heights = []

            for dx in range(-self.ROI_HALF_W, self.ROI_HALF_W + 1):
                x = cx + dx
                if x < 0 or x >= w:
                    continue
                z = depth_frame.get_distance(x, y_front)
                if z <= 0:
                    continue
                point = rs.rs2_deproject_pixel_to_point(intrin, [x, y_front], z)
                heights.append(self.CAMERA_HEIGHT - point[1])

            object_height = float(np.median(heights)) if heights else 0.0

            # 판단 및 퍼블리시
            new_mode = "Leg" if object_height > self.STEP_THRESH else "Wheel"
            if new_mode != self.current_mode:
                self.current_mode = new_mode
                msg = String()
                msg.data = self.current_mode
                self.mode_pub.publish(msg)
                self.get_logger().info(f"🚩 주행 모드 변경: {self.current_mode} (턱 높이: {object_height:.3f} m)")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 노드 종료 요청됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
