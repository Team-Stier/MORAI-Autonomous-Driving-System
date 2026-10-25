# /objects: 동일 입력 점군에서 검출·추적한 객체 목록. 객체가 없으면 빈 배열.
# frame_id=map. stamp는 입력 /velodyne_points/leveled의 stamp를 유지한다.
# header는 관측 기준 시각·좌표계이며, 예측 목표 시각은 Object의 예측 필드 정의를 따른다.
std_msgs/Header header
interfaces/Object[] objects
