# /local_path와 /local_path/velocity_injected의 공통 타입.
# /local_path의 velocity가 NaN인 point는 속도 미주입 상태다. 0을 포함한 유한값은 주입된 속도다.
# Velocity Planner의 1스테이지는 NaN인 point의 속도와 가속도만 채우고, 기존 값과 경로 형상·점 순서를 유지한다.
# 입력은 map, 출력은 입력 stamp 시점 TF로 변환한 base_link. 각 point pose도 변환한다.
# 경로와 point pose의 stamp는 입력 값을 유지하며 frame_id는 출력 좌표계로 설정한다.
std_msgs/Header header
interfaces/PathPoint[] points

# Optional full spatial reference for lateral tracking. Poses use Path.header frame.
# points remains the longitudinal v/a/time trajectory.
# Both arrays start at the same route station; the timed trajectory may start at ego pose.
# An empty lateral_reference uses points for both lateral and longitudinal tracking.
geometry_msgs/Pose[] lateral_reference
