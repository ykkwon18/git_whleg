This workspace is designed to test ROS2 humble - OpenRB-150 environment.
The nodes send 3-digits data through the serial port.  ( e.g., '100 100 100\n')
Follow the steps below to use the nodes in this workspace.

1. Clone whleg_ws folder into your Home directory.
2. cd to Whleg_ws and colcon build.
3. $source install/setup.bash
4. In the first turminal, execute $ros2 run whleg_bringup whleg_bringup
5. In the second turminal, execute $ros2 run whleg_teleop whleg_teleop
6. In the second turminal, you can control the motor speed using keyboard input.