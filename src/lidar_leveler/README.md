# lidar_leveler

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/velodyne_points`, `/vehicle/competition_status`, `/tf_static`
- 발행 연결: `/velodyne_points/leveled`
- 실행: /velodyne_points 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/lidar_leveler.yaml`은 UNKNOWN=0과 섹션 1~16을 빈 문자열(`""`)로 둔다. Core 미등록 항목은 NoCore로 종료하며 계산·발행하지 않는다.
- Core 등록은 `include/lidar_leveler/core_set.hpp`의 CoreId·CoreSet TODO에서 수행한다. Core는 `include/lidar_leveler/cores/`에 파일 하나씩 작성하고 실제 메시지를 개별 process 인자로 받는다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
