# #FRAME_CONTRACT
# 검출·추적 객체 하나이며 Object 자체는 특정 좌표계에 고정된 메시지가 아니다.
# 모든 공간 좌표와 관측 시각은 부모 메시지인 Objects.header.frame_id와 stamp를 따른다.
# position과 bbox_pose.position은 같은 bbox 중심이며, bbox_pose.orientation은
# bbox 자체 local 축을 부모 header frame으로 표현하는 단위 quaternion이다.
# 유효한 velocity·acceleration도 부모 header frame의 축 성분이다.
# bbox_size는 bbox 자체 local x/y/z 축의 전체 길이이므로 부모 frame의 축별 extent가 아니다.
# 현재 production perception 출력은 base_link_level이다. 이 frame은 차체 base_link와
# 뒷차축 원점·yaw를 공유하지만 차량 roll/pitch가 제거된 수평 frame이다.
# 향후 다른 모듈이 실제 좌표를 변환한 뒤 같은 메시지 구조를 사용할 수 있으므로
# 각 필드를 base_link로 하드코딩해 해석하면 안 된다. 좌표계 이름만 바꾸면
# 수치의 의미가 달라진다. 다른 frame이 필요하면 모듈 입력 경계에서 header.stamp에
# 맞는 명시적인 위치·방향·벡터 변환을 수행하고 bbox 자체 축 길이는 유지한다.
# 동일 객체를 연속 관측에서 추적하기 위한 ID. 특정 지도 객체, 클래스 또는
# 시뮬레이터 객체 ID와 미리 매핑된 값이 아니며 실행 간 동일성을 보장하지 않는다.
uint32 id
# 분류 이름. 미분류는 unknown.
string label
# bbox의 기하학적 중심 위치 [m], Objects.header.frame_id 기준. bbox_pose.position과 동일하다.
geometry_msgs/Point position
# 3축 속도 [m/s], Objects.header.frame_id의 축 성분. 자차 대비 상대 운동 여부는 미확정이다.
# velocity_available=false이면 이 값을 사용하지 않는다.
geometry_msgs/Vector3 velocity
# 3축 가속도 [m/s^2], Objects.header.frame_id의 축 성분. 자차 대비 상대 운동 여부는 미확정이다.
# acceleration_available=false이면 이 값을 사용하지 않는다.
geometry_msgs/Vector3 acceleration
# 속도·가속도 추정값의 존재 여부. 0이라는 수치만으로 미추정을 표현하지 않는다.
bool velocity_available
bool acceleration_available
# bbox 중심 위치 [m]와 bbox 자체 local 축을 Objects.header.frame_id로 표현하는 단위 quaternion.
# quaternion 순서는 ROS 규약 (x, y, z, w).
geometry_msgs/Pose bbox_pose
# bbox 자체 x·y·z 축의 전체 길이 [m]. 반길이가 아니다.
geometry_msgs/Vector3 bbox_size
