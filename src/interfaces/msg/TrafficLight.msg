std_msgs/Header header

# 영상에서 신호등이 탐지되지 않거나 점등 조합을 판정할 수 없으면 UNKNOWN.
# OFF는 신호등이 탐지되고 모든 등이 꺼진 것으로 판정된 경우만 사용한다.
# 아래 값은 조합별 enum이며 비트 플래그가 아니다.
uint8 UNKNOWN=0
uint8 OFF=1
uint8 RED=2
uint8 YELLOW=3
uint8 GREEN=4
uint8 LEFT=5
uint8 RED_YELLOW=6
uint8 RED_GREEN=7
uint8 RED_LEFT=8
uint8 YELLOW_GREEN=9
uint8 YELLOW_LEFT=10
uint8 GREEN_LEFT=11
uint8 RED_YELLOW_GREEN=12
uint8 RED_YELLOW_LEFT=13
uint8 RED_GREEN_LEFT=14
uint8 YELLOW_GREEN_LEFT=15
uint8 RED_YELLOW_GREEN_LEFT=16

uint8 state
