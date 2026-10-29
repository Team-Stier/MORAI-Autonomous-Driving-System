# 실제 발행한 /localization/ego의 추정 모드. 센서 수신 여부나 처리 성공/실패 코드가 아니다.
# header.frame_id=map, stamp는 같은 계산 회차의 ego stamp(추정 기준 IMU 시각)와 같다.
# 두 토픽의 전송은 원자적이지 않다. 소비자는 stamp와 입력 만료 계약을 검사한다.
std_msgs/Header header
# 미확정 값. 유효한 추정 모드로 사용하지 않는다.
uint8 UNKNOWN=0
# GPS로 보정하는 추정 Core가 계산한 위치·속도.
uint8 GPS_FUSED=1
# GPS 없이 IMU·차량 상태와 LiDAR 벽 관측을 사용하는 추정 Core가 계산한 위치·속도.
# 물리적인 터널 위치의 판정값은 아니다.
uint8 IMU_LIDAR=2
uint8 mode
