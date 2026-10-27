# tf_publisher

`map`·`base_link`·`base_link_level`·센서 frame의 TF 트리를 발행하는 두 노드다. `launch.sh`가 두 노드를 함께 실행한다.
노드 구성은 `config/tf_publisher/`와 `config/static_tf_publisher/`, 확장점은 `include/tf_publisher/`에 둔다.

| 노드 | 구독 | 발행 | 실행 | Stage |
| --- | --- | --- | --- | --- |
| `tf_publisher_node` | `/localization/ego` | `/tf` (`map` → `base_link`·`base_link_level`, ego stamp) | ego 입력마다 | `transform`: `ego_tf.hpp`의 `process(ego)` |
| `static_tf_publisher_node` | 없음 | `/tf_static` (`base_link` → 센서 frame, stamp 0) | 시작 시 1회 latched | `mounts`: `sensor_mounts.hpp`의 `process()` |

- `ego_tf.hpp`는 ego pose를 `child_frame_id=base_link` 변환으로 옮기고, 같은 위치에서 yaw만 따르는 `base_link_level` 변환을 함께 발행한다. Policy `remove_invalid.hpp`가 값·stamp·frame 오류 입력을 제거하며 ego의 Stale·ReceiveTimeout은 허용 issue다.
- `sensor_mounts.hpp`는 장착 변환 전체를 한 메시지로 만든다. 장착값은 `spec/0918sensors.json`에서 가져왔으며 camera optical 축 변환을 포함한다. 값은 `config/static_tf_publisher/parameters/sensor_mounts.yaml`에 두고 로더가 단위 quaternion을 검사한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
