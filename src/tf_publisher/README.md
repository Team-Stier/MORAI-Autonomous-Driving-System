# tf_publisher

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `ego_tf.hpp` Core를 등록한다.

- 구독: `/localization/ego`
- 발행 연결: `/tf`
- 실행: /localization/ego 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/tf_publisher.yaml`은 UNKNOWN=0과 섹션 1~16을 `ego_tf.hpp`에 매핑한다.
- `include/tf_publisher/core_set.hpp`가 ego를 필수 입력으로 선언하고 지속 Core의 `process(ego)`에 연결한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`/tf_static`은 Core와 독립인 장착 설정으로 시작 시 한 번 latched 발행한다. 장착값은 `spec/0918sensors.json`에서 가져왔으며 camera optical 축 변환을 포함한다. 동적 `/tf`는 ego 콜백에서 `map → base_link`를 발행한다.
