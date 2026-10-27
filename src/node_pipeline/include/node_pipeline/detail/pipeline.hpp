#pragma once
// 입력 적재·최초 조립·Stage 루프·최종 발행을 소유하는 실행기다. ROS에 의존하지 않는다.
#include <node_pipeline/detail/stages.hpp>

namespace node_pipeline {
namespace detail {

// Stage를 0부터 n-1까지 순서대로 실행한다. fn이 false를 반환하면 이후 Stage를 실행하지 않는다.
template<class Tuple, class Fn, std::size_t... I> bool forEachStageImpl(Tuple& stages, Fn& fn, std::index_sequence<I...>) {
  bool running = true;
  using Expand = int[];
  (void)Expand{0, (running = running && fn(std::integral_constant<std::size_t, I>{}, std::get<I>(stages)), 0)...};
  return running;
}
template<class Tuple, class Fn> bool forEachStage(Tuple& stages, Fn fn) {
  return forEachStageImpl(stages, fn, std::make_index_sequence<std::tuple_size<typename std::remove_const<Tuple>::type>::value>{});
}

template<class Buffer, class Assembler, class Stages, class CoreSet, class HeaderInjector, class IO> class PipelineNode {
 public:
  PipelineNode(ExecutionConfig execution, Buffer buffer, Assembler assembler, Stages stages, CoreSet cores,
               HeaderInjector header_injector, IO io)
      : execution(execution), buffer(std::move(buffer)), assembler(std::move(assembler)), stages(std::move(stages)),
        cores(std::move(cores)), header_injector(std::move(header_injector)), io(std::move(io)) {
    bool helper_missing = false;
    forEachStage(this->stages, [&](auto, const auto& stage) {
      helper_missing = helper_missing || !stage.complete();
      return true;
    });
    const auto mode = this->execution.mode;
    const bool mode_valid = mode == ExecutionMode::Periodic || mode == ExecutionMode::OnInput || mode == ExecutionMode::Latched;
    if (std::tuple_size<Stages>::value == 0 || helper_missing || this->execution.max_health_checks == 0 || !mode_valid ||
        (mode == ExecutionMode::OnInput && !this->buffer.containsInput(this->execution.trigger_input)))
      throw std::invalid_argument("invalid execution configuration");
  }

  template<class InputTag, class Message>
  ProcessReport onInput(InputTag input_tag, const std::shared_ptr<const Message>& retained_message) {
    const auto receive_clocks = io.readClocks();
    const auto sample = buffer.pushInput(input_tag, retained_message, receive_clocks);
    if (execution.mode == ExecutionMode::OnInput && input_tag == execution.trigger_input) {
      const Trigger trigger{ExecutionMode::OnInput, input_tag, sample.id};
      return process(trigger);
    }
    return {};
  }
  void onSection(const Section& section) { buffer.storeRawSection(section); }
  ProcessReport onPeriodic() {
    const Trigger trigger{ExecutionMode::Periodic, 0, 0};
    return process(trigger);
  }
  ProcessReport onStartup() {
    const Trigger trigger{ExecutionMode::Latched, 0, 0};
    return process(trigger);
  }

 private:
  // 처리 한 회차의 예외는 Failed로 보고하고 노드는 계속 실행한다. 실패한 스냅샷도 시도한 것으로 기록한다.
  ProcessReport process(const Trigger& trigger) {
    attempt_pending_ = false;
    try {
      return processSnapshot(trigger);
    } catch (const std::exception& error) {
      return fail(error.what());
    } catch (...) {
      return fail("unknown exception");
    }
  }

  ProcessReport fail(const std::string& reason) {
    if (attempt_pending_) assembler.markAttempted(attempt_token_);
    return finish(ProcessStatus::Failed, 0, reason);
  }

  ProcessReport processSnapshot(const Trigger& trigger) {
    const auto assembled = assembler.assemble(buffer, trigger);
    Snapshot snapshot{StageData{}, assembled.context, assembled.token, assembled.is_new};
    snapshot.data.update(assembled.data);
    if ((trigger.mode == ExecutionMode::OnInput || execution.require_new_snapshot) && !snapshot.is_new)
      return finish(ProcessStatus::Skipped, 0, "bundle already attempted");
    attempt_token_ = snapshot.token;
    attempt_pending_ = true;
    const auto health_clocks = io.readClocks();
    std::size_t checks = 0;
    ProcessReport report;
    const bool completed = forEachStage(stages, [&](auto stage_index, auto& stage) {
      using DataType = typename std::decay<decltype(stage)>::type::DataType;
      checks = 0;
      bool settled = false;
      auto selection = stage.selector.selectSection(snapshot.context.section, health_clocks);
      if (!selection.selected) {
        report = finish(ProcessStatus::NoCore, checks, "no compatible core");
        return false;
      }
      while (checks < execution.max_health_checks && !settled) {
        checks += 1;
        auto profile = stage.coreProfile(selection.id);
        const auto health = profile.ContractCheck(snapshot.data, snapshot.context, health_clocks);
        io.reportHealth(health, checks);
        const bool required_present = snapshot.data.hasInputs(profile.required_inputs);
        const bool issues_allowed = stage.acceptsIssues(selection.id, health);
        if (required_present && issues_allowed) {
          settled = true;
          break;
        }
        const auto next_selection = stage.selector.overrideHealth(selection, health);
        selection = next_selection;
        profile = stage.coreProfile(selection.id);
        const bool modified = profile.policy.apply(snapshot.data.template typed<DataType>(), health);
        static_cast<void>(modified);
      }
      if (!settled) {
        report = finish(ProcessStatus::Rejected, checks, "health check limit");
        return false;
      }
      if (stage_index == 0) assembler.markAttempted(snapshot.token);
      CoreResult result;
      if (!stage.process(cores, selection.id, snapshot.data, snapshot.context.stamp_ns, result)) {
        report = finish(ProcessStatus::NoCore, checks, "required core input missing");
        return false;
      }
      replaceInputs(snapshot, result);
      return true;
    });
    if (!completed) return report;
    auto message = snapshot.data.template message<typename IO::MessageType>(0);
    header_injector.injectHeader(message, snapshot.context);
    io.publish(message);
    return finish(ProcessStatus::Published, checks, std::string());
  }

  ProcessReport finish(ProcessStatus status, std::size_t checks, const std::string& reason) {
    const ProcessReport report{status, checks, reason};
    io.report(report);
    return report;
  }

  ExecutionConfig execution;
  Buffer buffer;
  Assembler assembler;
  Stages stages;
  CoreSet cores;
  HeaderInjector header_injector;
  IO io;
  std::vector<std::uint64_t> attempt_token_;
  bool attempt_pending_ = false;
};
}  // namespace detail
}  // namespace node_pipeline
