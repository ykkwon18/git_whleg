from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	urdf_file = os.path.join(
		get_package_share_directory('urdf_test'),
		'urdf',
		'urdf.urdf'
	)

	with open(urdf_file, 'r') as infp:
		robot_desc = infp.read()

	return LaunchDescription([
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			parameters=[{'robot_description': robot_desc}],
			output='screen'
		),
		Node(
			package='joint_state_publisher_gui',
			executable='joint_state_publisher_gui',
			name='joint_state_publisher_gui',
			output='screen'
		),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen'
		)
	])
