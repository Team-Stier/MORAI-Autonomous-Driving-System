std_msgs/Header header

# Core의 초기 저장 섹션. 첫 체크포인트 반지름에 진입하기 전에는 UNKNOWN을 반환·발행한다.
uint8 UNKNOWN=0
# checkpoints.csv의 0-based 행 인덱스 + 1이 섹션 값이다.
# map XY 거리 <= config 반지름이면 Core의 저장 섹션을 해당 값으로 갱신한다.
# 여러 반지름에 속하면 가장 가까운 체크포인트, 거리 동률이면 작은 인덱스를 선택한다.
# 어느 반지름에도 속하지 않으면 저장 섹션을 유지한다.
# Core는 매 process() 호출마다 저장 섹션을 반환하며 Pipeline은 이를 발행한다.
uint8 SECTION_1=1
uint8 SECTION_2=2
uint8 SECTION_3=3
uint8 SECTION_4=4
uint8 SECTION_5=5
uint8 SECTION_6=6
uint8 SECTION_7=7
uint8 SECTION_8=8
uint8 SECTION_9=9
uint8 SECTION_10=10
uint8 SECTION_11=11
uint8 SECTION_12=12
uint8 SECTION_13=13
uint8 SECTION_14=14
uint8 SECTION_15=15
uint8 SECTION_16=16

uint8 state
