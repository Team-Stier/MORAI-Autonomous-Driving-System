# tf_publisher

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `ego_tf.hpp` Core를 등록한다.

- 구독: `/localization/ego`
- 발행 연결: `/tf`
- 실행: /localization/ego 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `node.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 `ego_tf.hpp`에 매핑한다.
- `registrations/ego_tf.yaml`은 필수 입력 `ego`와 `process(ego)`의 반환 TFMessage를 연결한다.
- 공통 실행기는 지속 Core 객체·조립·선택·검사·Policy·계산·최종 발행을 담당한다.
- 새 TFMessage의 시간 메타데이터는 비운다. 바깥 Header가 없는 TFMessage에는 공통 헤더 주입을 수행하지 않는다. Core가 ego Header로 각 TransformStamped의 stamp와 `map`을 채운다. 출력은 검사하지 않는다.
- 기존 `config/tf_publisher.yaml`의 실행 설정·`stage1` 매핑·`static_transforms`는 공통 실행기가 읽는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`node.yaml`의 `enable_static_transforms`가 공통 정적 TF 기능을 켠다. `/tf_static`은 `static_transforms`의 전체 장착 bundle을 같은 시작 ROS 시각으로 한 번 latched 발행한다. 장착값은 `spec/0918sensors.json`에서 가져왔으며 camera optical 축 변환을 포함한다. 동적 `/tf`는 ego 콜백에서 `map → base_link`를 발행한다.
