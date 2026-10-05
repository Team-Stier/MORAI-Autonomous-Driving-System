# tf_publisher

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `ego_tf.hpp` Core를 등록한다.

- 구독: `/localization/ego`
- 발행 연결: `/tf`
- 실행: /localization/ego 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/tf_publisher.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 `ego_tf.hpp`에 매핑한다.
- `CoreSet`은 지속 Core 객체를 소유한다. `core_dispatch.hpp`의 case가 ego(0)를 필수 입력으로 선언·확인한 뒤 `process(ego)`를 호출한다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 변수는 입력의 stamp·receipt metadata를 계승한다. 새 TFMessage 반환값은 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 ego stamp와 `map`을 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`/tf_static`은 Core와 독립인 장착 설정으로 시작 시 한 번 latched 발행한다. 장착값은 `spec/0918sensors.json`에서 가져왔으며 camera optical 축 변환을 포함한다. 동적 `/tf`는 ego 콜백에서 `map → base_link`를 발행한다.
