# 검출·추적 객체 하나. 좌표계와 시각은 Objects.header를 따른다.
# map: 지도 원점 기준, +x UTM 격자 동쪽·+y 격자 북쪽·+z 위쪽.
# 동일 객체를 연속 관측에서 추적하기 위한 ID. 특정 지도 객체, 클래스 또는
# 시뮬레이터 객체 ID와 미리 매핑된 값이 아니며 실행 간 동일성을 보장하지 않는다.
uint32 id
# 분류 이름. 미분류는 unknown.
string label
# bbox의 기하학적 중심 위치 [m], map 좌표. bbox_pose.position과 동일하다.
geometry_msgs/Point position
# 지면 기준 절대 3축 속도 [m/s], map 축 성분. 자차 상대속도가 아니다.
# velocity_available=false이면 이 값을 사용하지 않는다.
geometry_msgs/Vector3 velocity
# 지면 기준 절대 3축 가속도 [m/s^2], map 축 성분. 자차 상대가속도가 아니다.
# acceleration_available=false이면 이 값을 사용하지 않는다.
geometry_msgs/Vector3 acceleration
# 속도·가속도 추정값의 존재 여부. 0이라는 수치만으로 미추정을 표현하지 않는다.
bool velocity_available
bool acceleration_available
# bbox 중심 위치 [m]와 bbox 자체 축을 map으로 변환하는 단위 quaternion.
# quaternion 순서는 ROS 규약 (x, y, z, w).
geometry_msgs/Pose bbox_pose
# bbox 자체 x·y·z 축의 전체 길이 [m]. 반길이가 아니다.
geometry_msgs/Vector3 bbox_size
# 미래 bbox 중심·자체 축 pose, map 좌표. prediction_available=false이면 사용하지 않는다.
# [i]의 목표 시각은 Objects.header.stamp + 0.5 * (i + 1)초 (0.5~5.0초)다.
# 0초 bbox는 기존 position·bbox_pose·bbox_size로 표현한다. 크기는 예측 구간 동안 일정하다.
bool prediction_available
geometry_msgs/Pose[10] predicted_bbox_poses
