# /local_path와 /local_path/velocity_injected의 공통 타입.
# /local_path의 velocity와 acceleration은 미주입 값이므로 사용하지 않는다.
# Velocity Planner는 경로 형상과 점 순서를 유지하고 속도와 가속도를 채운다.
# 입력은 map, 출력은 입력 stamp 시점 TF로 변환한 base_link. 각 point pose도 변환한다.
# 경로와 point pose의 stamp는 입력 값을 유지하며 frame_id는 출력 좌표계로 설정한다.
std_msgs/Header header
interfaces/PathPoint[] points
