geometry_msgs/PoseStamped pose
# 목표 종방향 속도 (m/s). 전후진 부호 규칙은 아직 미확정.
# NaN은 속도 미주입, 0을 포함한 유한값은 주입된 속도다. 무한대는 유효한 속도가 아니다.
float64 velocity
# 목표 종방향 가속도 (m/s^2). 부호 규칙은 아직 미확정.
float64 acceleration

# Optional trajectory time from Path.header.stamp. All-zero means a spatial profile.
# Timed trajectories start at zero and increase strictly, including stationary waits.
duration time_from_start
