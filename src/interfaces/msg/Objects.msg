# /objects: 동일 입력 점군에서 검출·추적한 객체 목록. 객체가 없으면 빈 배열.
# frame_id=base_link. stamp는 입력 /velodyne_points/leveled의 stamp를 유지한다.
# 모든 객체 위치·방향·벡터는 이 header의 좌표계와 시각을 따른다.
std_msgs/Header header
interfaces/Object[] objects
