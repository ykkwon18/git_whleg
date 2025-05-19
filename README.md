# git_whleg
 Wheel-Leg Transformable Mobile Robot

 Used:

 Ubuntu22.04LTS
 ROS2-Humble

 Robotis OpenRB-150
 Robotis XL330-M288-T



-----------------------------------------------
통신 구조 info

주고 받아야 할 데이터
1. cmd_vel  -  모터의 현재 누적 회전량  -> 라즈베리파이에서 현재 위치 추정
2. 디버깅 데이터 (openrb에서 라즈베리로)
3. 모터의 현재 누적 회전량
4. 누적 시간


# 라즈베리 -> openrb150
0: 현재 시간
1: cmd_vel
2: transform   True/False

# openrb150 -> 라즈베리
0: 현재 시간
1: 디버깅 메시지
2: 모터 누적 회전량
