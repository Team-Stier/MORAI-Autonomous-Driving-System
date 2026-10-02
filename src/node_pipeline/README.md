# node_pipeline

공통 C++14 라이브러리를 제공하는 서비스 패키지 겸 코딩 가이드 템플릿 저장소다.
`templates/`와 `templates_two_core/` 소스는 가이드 전용이며 절대 실행·빌드 대상이 아니다. 각 실행 패키지가 `TODO:` 부분을 구체화해 자신의 소스를 작성한다.
계산 알고리즘과 실제 입출력 메시지 연결은 실행 패키지가 소유한다.

## 처리 계약

1. 입력 callback은 구독 메시지를 원래 타입 그대로 큐에 저장한다. sample ID와 실제 수신 steady 시각을 별도로 기록한다. ROS 구독 큐는 1이고 조립 큐 용량은 config로 지정한다.
2. `periodic` 또는 지정한 토픽의 `on_input` callback이 `process()`를 실행한다. 나머지 callback은 적재만 한다. callback은 직렬 실행한다.
3. Assembler는 누락·오래됨·invalid 여부와 무관하게 항상 스냅샷을 반환한다. 원본 stamp를 새로 만들지 않는다. 예제 조립기는 최신 기준 stamp에 가장 가까운 샘플을 고르며, 동률이면 수신 ID가 큰 것을 고른다. on_input의 기준 메시지는 방금 수신한 sample ID로 고정한다.
4. 하나의 스냅샷과 루프 진입 전에 한 번 읽은 시각으로 health 검사와 policy 처리를 반복한다. 큐 전체는 Assembler만 읽는다. policy는 선택 메시지의 보정 복사본만 수정하고 stamp·수신 시각을 보존한다.
5. Health는 input별 모든 오류를 보관한다. timestamp 나이는 ROS 시각, 수신 경과는 steady 시각으로 검사한다. 누락된 입력의 수신 대기는 노드 시작 시각 기준이다. 시차는 존재하는 각 input 쌍에 대해 검사한다.
6. InputPolicy는 최고 우선순위 오류 하나를 선택하고 switch로 strategy 객체를 호출한다. 음수 priority는 dismiss다. 이미 Missing인 입력은 다시 보정하지 않는다. 동률은 input·issue 순회 순서다. strategy는 데이터를 수정하고 반환값은 없다.
7. AllGreen 또는 dismiss issue만 남으면 policy는 false를 반환하고 노드는 `break;`한다. 보정했으면 true를 반환하여 다음 검사 회차로 간다. 기본 최대 검사 횟수는 10회이며, 마지막 검사에서도 완료를 확인하면 수용한다. 완료를 확인하지 못하면 해당 스냅샷을 폐기한다.
8. CoreSelector는 `section_core_map`으로 선택하고 `health_core_map`으로 덮어쓴다. `when`은 AND 조건이고 가장 높은 priority의 일치 규칙을 적용한다. 일치하지 않으면 section 선택을 유지한다. 최고 priority 동률은 설정 오류다. 최초 구역은 UNKNOWN(0)이다.
9. CoreSet은 `requiredInputs(core_id)`로 Core별 필수 입력의 tuple 인덱스를 선언한다. 라이브러리는 선택 직후 `data.hasInputs(required)`로 모두 존재하는지 검사한다. 누락되면 NoCore로 종료하고 계산·헤더 주입·발행을 생략한다. 추가 입력은 제한하지 않는다. CoreSet은 선택된 지속 Core를 호출한다. 첫 Core에는 구독 메시지를, 후속 Core에는 이전 Core의 반환값을 각각의 인자로 전달한다. 마지막 Core는 목표 발행 메시지 타입으로 payload를 완성해 반환한다. Pipeline은 헤더만 주입하고 출력 검사 없이 발행한다.
10. 실행 중 예외를 잡아 재시도하거나 계속 실행하지 않는다. on_input은 마지막 core 시도 token과 같은 스냅샷의 재실행을 생략한다. periodic은 동일 입력으로도 계산·발행한다.

## Core 입출력 계약과 Strategy 패턴

첫 Core의 입력은 구독 메시지의 실제 타입 그대로다. 후속 Core는 이전 Core가 반환한 중간 변수를 개별 인자로 받는다. `msg1`, `msg2`는 인자 이름의 표기이며 공통 메시지 타입을 새로 정의하지 않는다.

```cpp
geometry_msgs::PoseStamped process(const sensor_msgs::NavSatFix& gps_msg,
               const sensor_msgs::Imu& imu_msg);
```

구독 입력을 `Value`로 변환하거나 Core에 Data·Slot·Sample 래퍼, Health, section, 수신 관리용 메타데이터를 넘기지 않는다.
원래 메시지의 header는 메시지의 일부로 전달된다. policy가 보정해도 메시지 타입은 동일하며 보정한 필드만 바뀐다.

마지막 Core의 `process()`는 목표 발행 메시지 타입을 반환하고 payload를 완성한다.
중간 Core는 다음 Core의 계산에 필요한 하나 이상의 변수를 반환할 수 있다. 개수·타입·순서는 연결된 Core의 계약으로 정하며 두 개로 고정하지 않는다.
여러 반환 변수는 `std::pair`·`std::tuple` 같은 표준 컨테이너로 묶을 수 있다. CoreSet이 값을 풀어 다음 Core의 개별 인자로 전달한다.
별도 Output·PoseResult 같은 계산 결과 래퍼나 성공/실패·준비 상태 코드를 만들지 않는다.
Core는 출력 메시지의 발행 헤더를 설정하지 않는다. Pipeline의 HeaderInjector가 stamp·frame_id 등 헤더만 주입한다.
HeaderInjector와 IO는 payload를 변경하거나 출력 검사를 하지 않는다. IO는 헤더 주입이 끝난 메시지를 발행한다.

Core 하나당 파일 하나를 둔다. 그 파일에는 필요한 include, 해당 Core 클래스의 `process()`와 내부 계산 상태만 둔다.
계산 상태는 다음 호출까지 유지되며 선택이 바뀌어도 자동 reset하거나 객체를 재생성하지 않는다.
CoreSet·입력 검사·policy·헤더 주입는 Core 파일 밖에 둔다. 필수 입력 선언과 존재 여부 검사는 Core 파일 밖에 둔다.

| Strategy 패턴 역할 | 객체 | 책임 |
| --- | --- | --- |
| Context | PipelineNode | 공통 실행 순서와 helper 소유 |
| 선택 | CoreSelector | section·health config로 core ID 결정 |
| 소유·호출 연결 | CoreSet | 지속 Core 소유, 실제 메시지를 개별 인자로 전달 |
| 계산 Strategy | 각 Core | `process(개별 인자...) → 계산 반환값`; 마지막 Core는 목표 발행 메시지 타입 반환 |

## 두 Core 순차 템플릿

[`templates_two_core/`](templates_two_core/)는 `Core1.process(msg1, msg2, ...) → (data1, ..., dataN) → Core2.process(data1, ..., dataN) → result 발행` 구조다.
Core1은 구독 메시지를 개별 인자로 받고 하나 이상의 중간 변수를 반환할 수 있다. Core2는 이 반환값을 개별 인자로 받으며, 중간 변수의 개수·타입·순서는 두 Core의 계약으로 정한다.
현재 예제는 같은 실제 메시지 타입의 두 값을 `std::pair<Message, Message>`로 묶어 반환한다. 다른 반환 개수·타입은 해당 Core의 반환 타입과 CoreSet의 인자 연결에 반영한다.
CoreSet이 지속 Core를 소유하고, 두 Selector가 선택한 Core ID로 각 단계의 호출을 연결한다. 각 Selector는 같은 section과 최종 구독 입력 health를 사용한다. 어느 단계든 선택되지 않으면 계산·헤더 주입·발행을 생략한다.
첫 단계의 필수 구독 입력을 Pipeline이 검사한다. 각 Core는 계산을 수행하고 마지막 Core가 최종 result의 payload를 완성한다. 최종 result의 헤더 주입과 발행은 Pipeline이 담당한다. 중간 변수는 검사·헤더 주입·발행하지 않는다.
ROS 설정은 `templates_two_core/config/template.yaml`에 둔다. 공통 실행·health·policy 설정은 최상위에 두고, `core1`과 `core2` 아래에 각각 `section_core_map`·`health_core_map`을 지정한다. 매핑 값은 각 단계에 등록한 Core 파일 이름이다.

## 공개 라이브러리와 노드별 코드

| 파일 | 제공 범위 |
| --- | --- |
| `include/node_pipeline/pipeline.hpp` | PipelineNode, 실행 설정·trigger·health·선택·처리 결과 공통 타입 |
| `include/node_pipeline/components.hpp` | 입력별 HealthLimits, 필수 입력 검사, InputBuffer〈Id, Access, Ts...〉, SnapshotAssembler, HealthChecker, 공통 수치 검사, ClampUpper/ClampLower/RemoveInput, InputPolicy, CoreSelector |
| `include/node_pipeline/ros.hpp` | ROS config 로더·검증, 이름→ID 해석, RosIO〈발행 메시지 타입〉 |
| `templates/types.hpp` | 가이드용 메시지와 입력·발행 메시지 타입 |
| `templates/scalar_core.hpp` | ScalarCore 클래스 하나와 계산 상태 |
| `templates/core_set.hpp` | 템플릿 Core 소유·메시지 인자 연결 |
| `templates/template_components.hpp` | 메시지 필드 Access·Validator, 라이브러리 타입 별칭·객체 조합 |
| `templates/ros_node.cpp` | TODO: 실제 타입·토픽을 구체화하는 config 호출, ROS HeaderInjector·callback 연결 |
| `templates/standalone.cpp` | ROS 없는 공개 API 사용 가이드 |
| `config/template.yaml` | 가이드 템플릿 설정 |

실행 노드는 다음 `TODO:` 부분을 구체화한다.

| 위치 | 구체화할 부분 |
| --- | --- |
| `templates/types.hpp` | 실제 구독·발행 메시지 타입 |
| `templates/template_components.hpp` | 입력 ID·타입 pack·Access·Validator·policy·selector |
| `templates/core_set.hpp` | Core 소유·필수 입력 인덱스·개별 process 인자 |
| `templates/scalar_core.hpp` | Core별 알고리즘과 지속 상태 |
| `templates/ros_node.cpp` | 토픽·callback 연결·발행 header 주입 기준 |
| `config/template.yaml` | 실행 방식·한도·policy 우선순위·Core 매핑 |

공통 라이브러리 소스는 구체화 대상이 아니다.
큐·조립·공통 health·strategy·매핑 탐색·config 파싱은 라이브러리 구현을 사용한다.
`InputBuffer<Id, Access, GpsMsg, ImuMsg, CloudMsg>`처럼 서로 다른 메시지 타입도 지정할 수 있다.
입력 ID는 tuple 순서와 대응하며 `std::integral_constant<Id, 입력ID>`로 구독 callback에 연결한다.
ROS 공유 포인터는 `retainMessage()`로 원본과 소유권을 유지한다. 보정은 copy-on-write로 원본과 같은 타입의 복사본에 적용한다.

## Config

`config/template.yaml`을 node private namespace에 로드한다.
`trigger_input`은 on_input일 때만 읽으며 등록된 upstream의 절대 토픽 이름이다.
periodic 모드에서는 frequency_hz로 timer를 생성한다.
시간 한도·큐 용량·검사 한도·policy priority·section 매핑·health 조건과 선택 priority는 config로 지정한다.
보정 priority와 core 선택 priority는 별도다.

```yaml
section_core_map:
  "0": primary
health_core_map:
  - when:
      - {input: primary, issue: Stale}
      - {input: primary, issue: ReceiveTimeout}
    priority: 110
    core: auxiliary
```

section_core_map의 빈 문자열(`""`)은 Core 미등록을 뜻하며 해당 섹션의 기본 Core를 선택하지 않는다.
등록한 input·core 이름만 사용할 수 있다. when에 적지 않은 input/issue는 제한하지 않는다.
구역 상수는 `src/interfaces/msg/SectionAnnotation.msg`를 따른다. 알고리즘 이름과 매핑은 각 실행 패키지가 정의한다.

## 템플릿 실행 시퀀스

라이브러리 내부 순회와 필드 검사는 공개 메서드 호출 안에 포함한다. 아래는 ROS 가이드 템플릿의 메서드 연결이다.

```mermaid
sequenceDiagram
    participant M as main
    participant U as primary·auxiliary 토픽
    participant SE as /section_annotation
    participant T as ros::SteadyTimer
    participant CB as callbacks: RosCallbacks
    participant N as node: PipelineNode
    participant B as InputBuffer
    participant A as SnapshotAssembler
    participant H as HealthChecker
    participant P as InputPolicy
    participant S as CoreSelector
    participant C as CoreSet
    participant K as ScalarCore
    participant MB as RosHeaderInjector
    participant IO as RosIO
    participant O as 출력 토픽

    M->>M: node_pipeline::loadRosConfig(nh, params, topics, inputs, cores, selector, policy)
    alt config 형식·타입·이름·한도 오류
        M-->>M: throw std::invalid_argument
    else 정상 설정
        M-->>M: settings
        M->>N: PipelineNode(execution, buffer, assembler, checker, policy, selector, cores, builder, io)
    end
    loop ROS callback 직렬 실행
        alt section 수신
            SE->>CB: onSection(message)
            CB->>N: onSection(Section)
            N->>B: storeRawSection(section)
        else input 수신
            U->>CB: onPrimary(message) 또는 onAuxiliary(message)
            CB->>CB: retainMessage(message)
            CB->>N: onInput(input_tag, message)
            N->>IO: readClocks()
            IO-->>N: receive_clocks
            N->>B: pushInput(input_tag, message, receive_clocks)
            B-->>N: sample
            opt OnInput이며 지정 input
                N->>N: process(trigger)
            end
        else timer tick
            T->>CB: onPeriodic(event)
            CB->>N: onPeriodic()
            N->>N: process(trigger)
        end
        opt process 호출됨
            N->>A: assemble(buffer, trigger)
            A-->>N: snapshot
            alt OnInput이고 snapshot.is_new=false
                N->>N: finish(Skipped, checks, reason)
                N->>IO: report(report)
            else 새 스냅샷 또는 Periodic
                N->>IO: readClocks()
                IO-->>N: health_clocks
                loop checks가 max_health_checks 미만
                    N->>H: healthCheck(data, context, health_clocks)
                    H-->>N: health
                    N->>IO: reportHealth(health, checks)
                    N->>P: apply(data, health)
                    P-->>N: modified
                    opt modified=false
                        Note over N: settled=true, break
                    end
                end
                alt settled=false
                    N->>N: finish(Rejected, checks, health check limit)
                    N->>IO: report(report)
                else settled=true
                    N->>S: select(health.section, health)
                    S-->>N: selection
                    alt selection.selected=false
                        N->>N: finish(NoCore, checks, reason)
                        N->>IO: report(report)
                    else selection.selected=true
                        N->>C: requiredInputs(selection.id)
                        C-->>N: required
                        N->>N: data.hasInputs(required)
                        alt 필수 입력 중 Missing 있음
                            N->>N: finish(NoCore, checks, required core input missing)
                            N->>IO: report(report)
                        else 필수 입력 모두 존재
                        N->>A: markAttempted(snapshot.token)
                        N->>C: process(selection.id, data)
                        alt id=Primary
                            C->>K: primary.process(data.get〈0〉().message())
                        else id=Auxiliary
                            C->>K: auxiliary.process(data.get〈1〉().message())
                        end
                        K-->>C: message
                        C-->>N: message
                        N->>MB: injectHeader(message, context)
                        MB-->>N: void
                        N->>IO: publish(message)
                        IO->>O: publisher.publish(message)
                        N->>N: finish(Published, checks, empty reason)
                        N->>IO: report(report)
                        end
                    end
                end
            end
        end
    end
```

## 라이브러리 원형 — 예제 상세 시퀀스

라이브러리화 이전 예제의 상세 시퀀스를 보존한다. 아래는 당시 구현이며, 현재 공개 API와 실행 계약은 앞의 템플릿 시퀀스를 따른다.

```mermaid
sequenceDiagram
    participant M as main(): ros_node.cpp
    participant R as nh/private_nh: ros::NodeHandle
    participant L as limits: HealthLimits
    participant U as /node_pipeline_example/primary 또는 auxiliary
    participant SE as /section_annotation
    participant T as timer: ros::SteadyTimer
    participant CB as callbacks: RosCallbacks
    participant N as node: PipelineNode
    participant MB as message_builder: RosMessageBuilder
    participant I as io: RosIO
    participant B as buffer: InputBuffer
    participant A as assembler: SnapshotAssembler
    participant H as checker: HealthChecker
    participant P as policy: InputPolicy
    participant UP as clamp_upper_: ClampUpper
    participant LO as clamp_lower_: ClampLower
    participant RM as remove_input_: RemoveInput
    participant S as selector: CoreSelector
    participant C as cores: CoreSet
    participant K as primary/auxiliary: ScalarCore
    participant O as /node_pipeline_example/output

    M->>R: param()/getParam()
    R-->>M: 실행 모드, 한도, priority, section_core_map, health_core_map
    loop duration 설정마다
        M->>M: readDurationNs(private_nh, key, default_sec)
        alt 비유한·0 이하·int64 ns 범위 초과 또는 1ns 미만
            M-->>M: invalid_argument
        else 유효
            M-->>M: duration_ns
        end
    end
    M->>R: resolveName(primary_topic) / resolveName(auxiliary_topic)
    R-->>M: primary_topic, auxiliary_topic
    alt mode가 periodic/on_input 밖
        M-->>M: invalid_argument
    end
    opt mode=on_input
        M->>R: param(trigger_input, default=primary_topic)
        R-->>M: trigger_topic
        alt trigger_topic이 비어 있거나 절대 이름이 아님
            M-->>M: invalid_argument
        else 절대 토픽 이름
            M->>R: resolveName(trigger_topic)
            R-->>M: trigger_topic
            alt trigger_topic=primary_topic
                Note over M: trigger_input=Input::Primary
            else trigger_topic=auxiliary_topic
                Note over M: trigger_input=Input::Auxiliary
            else 등록된 upstream 토픽이 아님
                M-->>M: invalid_argument
            end
        end
    end
    Note over M: Periodic에서는 trigger_input 설정을 읽거나 검사하지 않음
    opt queue_capacity 또는 max_health_checks가 0 이하
        M-->>M: invalid_argument
    end
    M->>L: validate()
    alt 비유한 bound·lower greater than upper·빈 frame·시간 한도 0 이하
        L-->>M: invalid_argument
    else 정상
        L-->>M: void
    end
    opt section_core_map 설정 있음
        alt mapping.getType()이 TypeStruct가 아님
            break std::invalid_argument
                M->>M: throw std::invalid_argument("invalid section_core_map")
                M->>M: ROS_FATAL_STREAM(error.what())
                Note over M: main return 1
            end
        else TypeStruct
            Note over M: selector.section_core_map.clear()
            loop mapping의 각 entry
                M->>M: std::stoi(entry->first, &parsed)
                alt 숫자로 해석할 수 없음
                    break std::invalid_argument
                        M-->>M: throw std::invalid_argument
                        M->>M: ROS_FATAL_STREAM(error.what())
                        Note over M: main return 1
                    end
                else int 표현 범위 초과
                    break std::out_of_range
                        M-->>M: throw std::out_of_range
                        M->>M: ROS_FATAL_STREAM(error.what())
                        Note over M: main return 1
                    end
                else 정수 변환 성공
                    M-->>M: section
                    Note over M: parsed=해석한 문자 수
                    alt parsed가 key 길이와 다름 또는 section이 0..255 밖 또는 value가 TypeString이 아님
                        break std::invalid_argument
                            M->>M: throw std::invalid_argument("invalid section mapping entry")
                            M->>M: ROS_FATAL_STREAM(error.what())
                            Note over M: main return 1
                        end
                    else entry 형식 유효
                        Note over M: core=entry 문자열 값
                        alt core가 primary와 auxiliary 모두 아님
                            break std::invalid_argument
                                M->>M: throw std::invalid_argument("unknown core")
                                M->>M: ROS_FATAL_STREAM(error.what())
                                Note over M: main return 1
                            end
                        else 등록된 core
                            Note over M: selector.section_core_map[section]=CoreId
                        end
                    end
                end
            end
        end
    end
    alt UNKNOWN 매핑 없음 또는 Periodic 주기가 비유한·0 이하·ROS Duration 표현 범위 밖
        M-->>M: invalid_argument
    else 유효 설정
        M->>R: getParam(health_core_map, health_mapping)
    R-->>M: bool
    opt health_core_map 설정 있음
        loop 각 규칙과 when 조건
            alt 필수 필드·타입·이름 오류 또는 when 비어 있음
                M-->>M: throw std::invalid_argument
            else 유효
                Note over M: selector.health_core_map에 검증된 rule 추가
            end
        end
    end
    M->>R: advertise(output_topic, 1)
        R-->>M: RosIO.publisher
        M->>N: PipelineNode(config, buffer, assembler, checker, policy, selector, cores, message_builder, io)
        alt max_health_checks=0·invalid mode·OnInput trigger가 등록 input 아님
            N-->>M: invalid_argument
        else 정상 생성
            N-->>M: node
            Note over N: helper는 value member, core 상태 유지<br/>InputBuffer(capacity=0)도 생성 실패
            Note over M,CB: callbacks=RosCallbacks{node}
            M->>R: subscribe(primary_topic/auxiliary_topic/section, 1, callbacks)
            opt Periodic
                M->>R: createSteadyTimer(period, onPeriodic, callbacks)
                R-->>M: timer
            end
            M->>R: ros::spin()
            Note over CB,T: 동일 callback queue에서 직렬 처리
        end
    end
    opt 위 초기화/연결에서 std::exception
        M->>M: ROS_FATAL_STREAM(error.what())
        Note over M,T: 지역 ROS handle 소멸, main return 1<br/>정상 spin 종료는 return 0
    end

    loop 초기화 성공 후 ros::spin()의 직렬 callback 처리
        alt section callback
            SE->>CB: onSection(msg)
            CB->>N: onSection(Section(true, state, stamp, frame))
            N->>B: storeRawSection(section)
            Note over B,N: 최신 raw section 1개 보관 후 callback 종료<br/>section 자체는 process trigger가 아님
        else primary 또는 auxiliary callback
            U->>CB: onPrimary(msg) / onAuxiliary(msg)
            CB->>CB: decodeInput(*msg)
            CB->>N: onInput(input, value)
            N->>I: readClocks()
            I-->>N: Clocks(ros_ns, steady_ns)
            N->>B: pushInput(input, value, clocks)
            alt 등록되지 않은 input
                B-->>N: invalid_argument
                N->>N: finish(Exception, 0, reason)
                N->>I: report(report)
                Note over N: 현재 callback 종료·process 호출 없음
            else 등록된 input
                Note over B: 원본·stamp·receipt_steady_ns·sample_id 저장<br/>값 검사 없음
                opt queue.size greater than capacity
                    Note over B: pop_front(): 가장 먼저 수신한 sample 1개 폐기<br/>해당 input의 evictions 증가 (예제 규칙)
                end
                B-->>N: Sample(id, stamp_ns)
                alt OnInput이고 input=trigger_input
                    Note over N: 이 callback 안에서 아래 공통 process를 호출<br/>trigger는 방금 수신한 input/id/stamp
                else Periodic 또는 보조 input
                    N-->>CB: Queued
                    Note over N: 큐 적재 후 callback 종료·process 호출 없음
                end
            end
        else periodic callback
            T->>CB: onPeriodic(event)
            CB->>N: onPeriodic()
            alt mode가 Periodic이 아님
                N->>N: finish(Skipped, 0, periodic trigger disabled)
                N->>I: report(report)
                Note over N: 현재 callback 종료·process 호출 없음
            else Periodic
                Note over N: 아래 공통 process를 호출<br/>trigger는 기본 생성한 Periodic Trigger
            end
        end
        opt 현재 callback이 process를 호출하는 경로
            N->>N: process(trigger)
            N->>A: assemble(buffer, trigger)
            A->>B: readView()
            B-->>A: raw queues, evictions, raw section 복사
            alt OnInput
                loop 지정 input queue의 각 sample
                    opt sample.id=trigger.sample_id
                        Note over A: reference=해당 sample
                    end
                end
            else Periodic
                loop 모든 queue의 모든 sample
                    opt reference 없음 또는 더 큰 stamp 또는 같은 stamp에서 더 큰 id
                        Note over A: reference=해당 sample
                    end
                end
            end
            loop 각 input queue
                alt OnInput의 지정 input
                    Note over A: reference 그대로 배치 (없으면 Missing)
                else 다른 input 또는 Periodic
                    loop queue의 각 sample
                        opt stamp 거리가 작음 또는 같은 거리에서 더 큰 id
                            Note over A: 가장 가까운 sample 배치<br/>빈 queue는 Missing·나이/값으로 제외하지 않음
                        end
                    end
                end
                Note over A: token에 선택 id 저장 (Missing=0)
            end
            Note over A: context.stamp_ns/frame은 reference sample에서 보존
            A-->>N: Snapshot(data, context, token, is_new)
            alt OnInput이고 snapshot.is_new=false
                N->>N: finish(Skipped, 0, bundle already attempted)
                N->>I: report(report)
                Note over N: 중복 bundle은 process return
            else 새 callback bundle 또는 Periodic
                Note over N,H: snapshot을 1회 조립한 뒤<br/>checks=0, 작업용 data를 계속 사용
                N->>I: readClocks()
                I-->>N: health_clocks
                loop checks가 max_health_checks 미만이고 settled=false
                    Note over N: checks += 1
                    N->>H: healthCheck(data, context, health_clocks)
                    Note over H: 매 회차 새 Health: inputs=[]<br/>section=UNKNOWN, 이전 해결 오류를 current에 누적하지 않음
                    loop 각 input
                        alt 현재 working slot 없음
                            Note over H: current issues += Missing
                        else slot 있음
                            H->>H: checkValues(slot.sample, health_clocks)
                            opt stamp가 0 이하
                                Note over H: issues += InvalidStamp
                            end
                            opt stamp가 ros_now보다 미래
                                Note over H: issues += FutureStamp
                            end
                            opt frame이 limits.frame과 다름
                                Note over H: issues += FrameMismatch
                            end
                            alt value=NaN
                                Note over H: issues += NaN
                            else value=Inf
                                Note over H: issues += Inf
                            else value greater than upper
                                Note over H: issues += OutRangePositive
                            else value less than lower
                                Note over H: issues += OutRangeNegative
                            end
                            H-->>H: current issue vector
                            opt stamp greater than 0이고 ros age가 stale_ns 초과
                                H->>H: exceeds(ros_ns, stamp_ns, stale_ns)
                                Note over H: current issues += Stale
                            end
                        end
                        alt 현재 input 없음
                            Note over H: timeout 기준=started_steady_ns
                        else 현재 input 있음
                            Note over H: timeout 기준=slot.sample.receipt_steady_ns
                        end
                        H->>H: exceeds(steady_ns, 기준, receive_timeout_ns)
                        opt true
                            Note over H: current issues += ReceiveTimeout
                        end
                    end
                    loop i=0부터 inputs.size() 미만
                        loop j=i+1부터 inputs.size() 미만
                            opt 두 input이 존재하고 절대 stamp 차이가 max_skew_ns 초과
                                loop input=i, j
                                    H->>H: std::find(issues.begin(), issues.end(), Kind::Skew)
                                    H-->>H: iterator
                                    opt iterator=issues.end()
                                        Note over H: issues에 Skew 추가
                                    end
                                end
                            end
                        end
                    end
                    alt raw section 없음
                        Note over H: section=UNKNOWN
                    else section frame이 map 아님·stamp 0 이하·미래 stamp
                        Note over H: invalid_section=true, section=UNKNOWN (예제 규칙)
                    else section header 유효
                        Note over H: section=raw.state<br/>해당 enum의 config 매핑 여부는 Selector가 판정
                    end
                    H-->>N: Health(current issues, section, evictions)
                    N->>I: reportHealth(health, checks)
                    Note over I: current issues·invalid section·누적 eviction을 동기 진단
                    N->>P: apply(data, health)
                    P->>P: node_pipeline::highestPriorityIssue(health, priority, issue)
                    loop 모든 input의 모든 issue
                        opt priority(input, kind)가 현재 best_rank보다 큼
                            Note over P: issue=이 오류, best_rank=rank<br/>rank 음수는 dismiss, 동률이면 이전 선택 유지
                        end
                    end
                    P-->>P: 처리할 issue 있음 / 없음
                    alt 처리할 issue 없음 (AllGreen 또는 dismiss issue만 남음)
                        P-->>N: false
                        Note over N: settled=true<br/>break
                    else issue.kind=OutRangePositive
                        P->>UP: operator()(data, issue)
                        Note over UP: data.inputs[index(issue.input)].sample.message.value=upper
                        UP-->>P: void
                        P-->>N: true
                    else issue.kind=OutRangeNegative
                        P->>LO: operator()(data, issue)
                        Note over LO: data.inputs[index(issue.input)].sample.message.value=lower
                        LO-->>P: void
                        P-->>N: true
                    else InvalidStamp/FutureStamp/FrameMismatch/NaN/Inf
                        P->>RM: operator()(data, issue)
                        Note over RM: data.inputs[index(issue.input)].present=false
                        RM-->>P: void
                        P-->>N: true
                    else 그 외 issue에 nonnegative rank를 설정함
                        P->>RM: operator()(data, issue)
                        Note over RM: data.inputs[index(issue.input)].present=false
                        RM-->>P: void
                        P-->>N: true
                    end
                end
                alt settled=false
                    N->>N: finish(Rejected, checks, health check limit)
                    N->>I: report(report)
                else settled=true
                        N->>S: select(health.section, health)
                        S->>S: section_core_map.find(section)
                        S-->>S: base
                        opt base != section_core_map.end()
                            Note over S: selection=Selection(true, base.core)
                        end
                        loop health_core_map의 각 rule
                            break rule.when이 비어 있음
                                S-->>N: throw std::invalid_argument(empty health when)
                            end
                            S->>S: std::all_of(rule.when.begin(), rule.when.end(), predicate)
                            loop when의 condition (첫 불일치에서 종료)
                                S->>S: health.inputs.at(index(condition.input))
                                S->>S: std::find(issues.begin(), issues.end(), condition.kind)
                                S-->>S: iterator
                            end
                            S-->>S: matches
                            opt matches=true
                                alt best 없음 또는 rule.priority > best.priority
                                    Note over S: best=rule, tied=false
                                else rule.priority=best.priority
                                    Note over S: tied=true
                                end
                            end
                        end
                        break tied=true
                            S-->>N: throw std::invalid_argument(ambiguous health_core_map priority)
                        end
                        opt best 있음
                            Note over S: selection=Selection(true, best.core)
                        end
                        S-->>N: selection
                        alt selected=false
                            N->>N: finish(NoCore, checks, no compatible core)
                            N->>I: report(report)
                        else selected=true
                            N->>A: markAttempted(snapshot.token)
                            N->>C: process(selection.id, data)
                            alt id=Primary
                                C->>K: primary.process(data.inputs[index(Input::Primary)].sample.message)
                            else id=Auxiliary
                                C->>K: auxiliary.process(data.inputs[index(Input::Auxiliary)].sample.message)
                            end
                            activate K
                            Note over K: 선택된 input의 계산값만 복사하고 updates_ 증가<br/>health를 받지 않음·계산 상태는 다음 frame에도 유지
                            K-->>C: output
                            deactivate K
                            C-->>N: output
                                    N->>MB: build(output, context)
                                    Note over MB: context.stamp_ns/frame으로 header 구성<br/>output.value를 vector.x에 설정
                                    MB-->>N: message
                                    N->>I: publish(message)
                                    I->>O: publisher.publish(message)
                                    I-->>N: void
                                    N->>N: finish(Published, checks, empty reason)
                                    N->>I: report(report)
                        end
                end
            end
        end
    end
```

## 라이브러리 사용

실행 패키지는 `node_pipeline`에 의존하고 공개 헤더를 include한다. `ros.hpp` 사용 시 roscpp 의존성을 추가한다.
가이드 템플릿을 실행 파일 target이나 bringup에 등록하지 않는다. 실행 패키지에서 작성한 소스만 빌드한다.
