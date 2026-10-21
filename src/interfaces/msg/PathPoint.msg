geometry_msgs/PoseStamped pose
# 목표 종방향 속도 (m/s). 전후진 부호 규칙은 아직 미확정.
# NaN은 속도 미주입, 0을 포함한 유한값은 주입된 속도다. 무한대는 유효한 속도가 아니다.
float64 velocity
# 목표 종방향 가속도 (m/s^2). 부호 규칙은 아직 미확정.
float64 acceleration
# Path.header.stamp부터 해당 점의 계획 예정 시각까지의 시간.
# 시간 계획을 제공하면 첫 점은 0, 이후 점은 엄격히 증가한다. 대기 중에도 시간은 증가한다.
# 모든 점이 0이면 예정 시각 미제공이다. 시간 기반 MPC는 이 경로를 허용하지 않는다.
# pose.header.stamp는 입력 기준 시각이며 이 필드의 미래 예정 시각과 구분한다.
duration time_from_start
