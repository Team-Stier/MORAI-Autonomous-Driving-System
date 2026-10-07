# lidar_leveler

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `pass_through.hpp` Core가 센서 장착 TF로 점군을 `base_link`로 변환한다.

- 구독: `/velodyne_points`, `/vehicle/competition_status`, `/tf_static`
- 발행 연결: `/velodyne_points/leveled`
- 실행: /velodyne_points 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `node.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 `pass_through.hpp`에 매핑한다.
- `registrations/pass_through.yaml`은 필수 입력 `cloud`·`tf_static`과 `process(cloud, tf_static)`의 반환 점군을 연결한다.
- `parameters/pass_through.hpp`의 읽기 전용 로더가 z 범위를 생성자에 주입하고 `validators/pass_through.hpp`가 점군 layout과 센서 장착 TF 존재를 검사한다.
- 공통 실행기는 지속 Core 객체·조립·선택·공통 검사·Policy·계산·최종 발행을 담당한다.
- 점군 출력은 `time_from: cloud`로 입력 stamp·수신 시각을 계승한다. `output.header`가 입력 점군 stamp와 `base_link`를 한 번 주입한다. 출력은 검사하지 않는다.
- 기존 `config/lidar_leveler.yaml`의 실행 설정·z 범위·`stage1` 매핑은 공통 실행기가 읽는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
