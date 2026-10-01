std_msgs/Header header

# 현재 위치에 해당하는 구간을 판정할 수 없으면 UNKNOWN. 예: 위치 미수신 또는 구간 미매칭.
uint8 UNKNOWN=0
# 지도에 표시된 구간 종류다. 같은 종류의 여러 위치는 같은 상수를 사용한다.
# GPS 음영 구간.
uint8 GPS_SHADOW=1
# 주변 차량 인지 구간.
uint8 SURROUNDING_VEHICLES=2
# 교차로 구간 진입.
uint8 INTERSECTION_ENTRY=3
# 회전 교차로 진입.
uint8 ROUNDABOUT_ENTRY=4
# 신호등 구간. 지도에 표시된 두 신호등 위치에 공통 적용한다.
uint8 TRAFFIC_LIGHT=5
# 신호등과 동적 장애물이 함께 있는 구간.
uint8 TRAFFIC_LIGHT_DYNAMIC_OBSTACLE=6
# 신호등과 정적 장애물이 함께 있는 구간.
uint8 TRAFFIC_LIGHT_STATIC_OBSTACLE=7

uint8 state
