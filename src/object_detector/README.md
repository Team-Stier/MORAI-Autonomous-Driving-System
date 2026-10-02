# object_detector

`node_pipeline` 라이브러리를 사용하는 두 Core 순차 실행 패키지다. 두 단계 모두 Core는 아직 등록하지 않았다.

- 구독: `/velodyne_points/leveled`, `/section_annotation`
- 발행 연결: `/objects`
- 실행: /velodyne_points/leveled 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- 처리: `Core1.process(PointCloud2) → Objects → Core2.process(Objects) → Objects`. 중간·최종 메시지는 `interfaces::Objects` 하나씩이다.
- `config/object_detector.yaml`의 `core1`·`core2` 아래에 각각 `section_core_map`·`health_core_map`을 둔다. 두 단계 모두 UNKNOWN=0과 섹션 1~16은 빈 문자열(`""`)로 둔다. 각 Selector는 같은 section과 최종 구독 입력 health를 사용한다. 어느 단계든 Core가 선택되지 않으면 NoCore로 종료하며 계산·발행하지 않는다.
- Core 등록은 `include/object_detector/core_set.hpp`의 FirstCoreId·SecondCoreId·CoreSet TODO에서 수행한다. Core는 `include/object_detector/cores/`에 파일 하나씩 작성한다. CoreSet이 지속 Core를 소유하고, 첫 단계에 구독 점군을 전달한 뒤 반환된 Objects를 두 번째 단계에 전달한다.
- 중간 Objects는 헤더 주입·검사·발행하지 않는다. 최종 Objects는 Pipeline이 입력 점군 stamp와 `frame_id=base_link`를 주입하여 `/objects`로 발행한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
