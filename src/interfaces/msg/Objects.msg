# /objects: 동일 입력 점군에서 검출·추적한 객체 목록. 객체가 없으면 빈 배열.
# #BASE_LINK_LEVEL
# 현재 /objects의 production frame은 base_link_level이다.
# 이 frame은 base_link와 차량 뒷차축 중심 원점·yaw를 공유하지만 roll/pitch가
# 제거된 perception용 수평 좌표계이다. +x는 수평 heading 방향, +y는 좌측,
# +z는 중력 반대 방향이며 차체와 함께 기울어지는 기존 base_link의 별칭이 아니다.
# LiDAR ROI·VoxelGrid·DBSCAN·OBB를 같은 수평 frame에서 안정적으로 계산하기 위해
# 사용하며, detector 결과는 그 좌표를 역회전하지 않고 그대로 발행한다.
# 정확한 역회전은 해당 점군 수평화에 쓴 동일 시각의 자세를 요구한다. 최신 차량
# 상태를 다시 사용하면 두 회전이 상쇄되지 않을 수 있어 detector 내부에서 하지 않는다.
# downstream은 header.frame_id를 확인하고 base_link나 map이 필요하면 자신의
# 입력 경계에서 header.stamp에 맞는 명시적인 좌표 변환을 수행해야 한다.
# frame 이름만 바꾸거나 mounting·roll/pitch를 다시 적용하면 좌표 의미가 달라진다.
#
# #FRAME_CONTRACT
# Object 내부의 위치·bbox 방향·유효한 속도와 가속도는 이 header.frame_id를 따른다.
# Object는 frame-generic 구조이므로 실제 좌표 변환 뒤 다른 frame으로 표현할 수도 있다.
# production header.stamp는 입력 /velodyne_points/leveled의 stamp를 유지한다.
# frame을 변환해도 관측 시각을 최신 차량 상태나 계산 완료 시각으로 대체하지 않는다.
# 모든 객체 위치·방향·벡터는 이 header의 좌표계와 시각을 따른다.
std_msgs/Header header
interfaces/Object[] objects
