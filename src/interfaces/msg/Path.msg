# /local_path와 /local_path/velocity_injected의 공통 타입.
# /local_path의 velocity가 NaN인 point는 속도 미주입 상태다. 0을 포함한 유한값은 주입된 속도다.
# Velocity Planner의 1스테이지는 NaN인 point의 속도와 가속도만 채우고, 기존 값과 경로 형상·점 순서를 유지한다.
# 입력은 map, 출력은 base_link. 각 point pose도 변환한다.
# base_link 출력은 변환에 실제 사용한 map→base_link TF를 map_from_path_frame에 동봉한다.
# header.stamp와 time_from_start는 계획 시간축이다. TF 시각으로 대체하지 않는다.
# 경로와 point pose의 stamp는 입력 값을 유지하며 frame_id는 출력 좌표계로 설정한다.
# point.time_from_start는 header.stamp 기준의 계획 예정 시각이다.
# 속도 주입·정지선 처리·좌표 변환은 이 상대 시각을 변경하지 않는다.
# 시간 기반 제어는 최종 velocity_injected의 속도·가속도를 사용한다.
std_msgs/Header header
# map 경로에서는 미사용. base_link 경로에서는 parent=map, child=base_link인 실제 TF.
# 제어기는 이 변환으로 map 위치를 복원하므로 TF 시각을 계획 시각과 같다고 추정하지 않는다.
geometry_msgs/TransformStamped map_from_path_frame
interfaces/PathPoint[] points
