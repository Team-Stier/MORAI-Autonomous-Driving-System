# object_detector

`node_pipeline` 라이브러리를 사용하는 두 Core 순차 실행 패키지다. stage1에는 `Dbscan`, stage2에는 `ObjectsPassThrough`가 등록되어 있다.

- 구독: `/velodyne_points/leveled`, `/section_annotation`
- 발행 연결: `/objects`
- 실행: /velodyne_points/leveled 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- 처리: `Core1.process(PointCloud2) → Objects → Core2.process(Objects) → Objects`. 중간·최종 메시지는 `interfaces::Objects` 하나씩이다.
- `config/object_detector.yaml`의 `stage1`·`stage2` 아래에 각각 `section_core_map`·`health_core_map`을 둔다. UNKNOWN=0과 섹션 1~16에서 stage1은 `dbscan.hpp`, stage2는 `objects_pass_through.hpp`를 선택한다. 각 Selector는 같은 section과 각 스테이지 현재 입력의 health를 사용한다. 어느 단계든 Core가 선택되지 않으면 NoCore로 종료하며 계산·발행하지 않는다.
- Core 등록은 `include/object_detector/core_set.hpp`의 FirstCoreId·SecondCoreId·CoreSet에서 수행한다. Core는 `include/object_detector/cores/`에 파일 하나씩 작성한다. CoreSet은 기존 Core ID 구조와 Core 객체를 소유한다. `core_dispatch.hpp`에 필수 입력 목록과 단계별 선택 분기·개별 인자 연결을 둔다. Pipeline이 첫 Core에 구독 점군을 전달하고, 반환된 Objects를 두 번째 Core에 전달한다.
- 중간 Objects는 두 번째 스테이지에서 검사·보정·선택하며 헤더 주입·발행하지 않는다. 최종 Objects는 Pipeline이 입력 점군 stamp와 `frame_id=base_link`를 주입하여 `/objects`로 발행한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

노드는 공통 스테이지 루프를 두 번 실행한다. 각 스테이지는 독립된 checker·policy·selector와 `stage1`·`stage2` 설정을 사용한다. CoreSet은 실행을 중계하지 않으며 Pipeline의 case가 소유 객체를 직접 호출한다. 기존 입력의 수정·패스스루 슬롯은 시간 메타데이터를 상속하고, 새 계산 슬롯은 이를 비워 두며 다음 검사에서 그 부재는 정상이다. 스냅샷 context·token은 유지하며 마지막 스테이지 후에만 헤더 주입·발행한다.
