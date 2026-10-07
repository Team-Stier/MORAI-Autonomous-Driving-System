# /objects: 동일 입력 점군에서 검출·추적한 객체 목록. 객체가 없으면 빈 배열.
# frame_id=base_link. stamp는 입력 /velodyne_points/leveled의 stamp를 유지한다.
# 객체 위치·bbox 방향은 base_link, 속도·가속도 벡터는 map 축 성분이다.
std_msgs/Header header
interfaces/Object[] objects
