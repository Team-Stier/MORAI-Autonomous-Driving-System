#pragma once
#include "stages.hpp"

namespace node_pipeline {
// Callbacks are serial; each stage owns its typed helpers, CoreSet owns persistent Cores.
template<class Buffer, class Assembler, class CoreSet, class HeaderInjector, class IO>
class PipelineNode {
 public:
  using InputId = typename Buffer::InputId;
  using Config = ExecutionConfig<InputId>;
  using StageType = Stage<CoreSet, typename Buffer::ContextType>;
  PipelineNode(Config config, Buffer buffer, Assembler assembler,
               std::vector<StageType> stages, CoreSet cores, HeaderInjector header_injector, IO io)
      : config_(config), buffer_(std::move(buffer)), assembler_(std::move(assembler)),
        stages_(std::move(stages)), cores_(std::move(cores)),
        header_injector_(std::move(header_injector)), io_(std::move(io)) {
    if (stages_.empty() || config_.max_health_checks == 0 ||
        (config_.mode != ExecutionMode::Periodic && config_.mode != ExecutionMode::OnInput) ||
        (config_.mode == ExecutionMode::OnInput && !buffer_.containsInput(config_.trigger_input)))
      throw std::invalid_argument("invalid execution configuration");
    for (const auto& stage : stages_)
      if (!stage.healthCheck || !stage.apply || !stage.select || !stage.process)
        throw std::invalid_argument("incomplete stage");
  }
  template<class InputTag, class Message> ProcessReport onInput(InputTag input, const Message& message) {
    const auto sample = buffer_.pushInput(input, message, io_.readClocks());
    if (config_.mode == ExecutionMode::OnInput && static_cast<InputId>(input) == config_.trigger_input)
      return process({ExecutionMode::OnInput, static_cast<InputId>(input), sample.id, sample.stamp_ns});
    return {};
  }
  template<class Message> void onSection(const Message& message) { buffer_.storeRawSection(message); }
  ProcessReport onPeriodic() {
    if (config_.mode != ExecutionMode::Periodic)
      return finish(ProcessStatus::Skipped, 0, "periodic trigger disabled");
    return process({});
  }
 private:
  ProcessReport process(const Trigger<InputId>& trigger) {
    const auto assembled = assembler_.assemble(buffer_, trigger);
    if ((trigger.mode == ExecutionMode::OnInput || config_.require_new_snapshot) && !assembled.is_new)
      return finish(ProcessStatus::Skipped, 0, "bundle already attempted");
    struct Snapshot {
      StageData data;
      decltype(assembled.context) context;
      decltype(assembled.token) token;
      bool is_new;
    } snapshot{{}, assembled.context, assembled.token, assembled.is_new};
    snapshot.data.update(assembled.data);
    const auto health_clocks = io_.readClocks();
    bool attempted = false;
    const BeforeCore before = [&] {
      if (!attempted) { assembler_.markAttempted(snapshot.token); attempted = true; }
    };
    std::size_t checks = 0;
    for (auto& stage : stages_) {
      checks = 0;
      bool settled = false;
      StageHealth health;
      for (; checks < config_.max_health_checks;) {
        ++checks;
        health = stage.healthCheck(snapshot.data, snapshot.context, health_clocks);
        io_.reportHealth(health, checks);
        if (!stage.apply(snapshot.data, health)) { settled = true; break; }
      }
      if (!settled) return finish(ProcessStatus::Rejected, checks, "health check limit");
      const auto selection = stage.select(health.section, health);
      if (!selection.selected) return finish(ProcessStatus::NoCore, checks, "no compatible core");
      CoreResult result;
      if (!stage.process(cores_, selection.id, snapshot.data, result, before))
        return finish(ProcessStatus::NoCore, checks, "required core input missing");
      replaceInputs(snapshot, result);
    }
    auto message = snapshot.data.template message<typename IO::MessageType>(0);
    header_injector_.injectHeader(message, snapshot.context);
    io_.publish(message);
    return finish(ProcessStatus::Published, checks, {});
  }
  ProcessReport finish(ProcessStatus status, std::size_t checks, const std::string& reason) noexcept {
    ProcessReport report{status, checks, reason};
    io_.report(report);
    return report;
  }
  Config config_;
  Buffer buffer_;
  Assembler assembler_;
  std::vector<StageType> stages_;
  CoreSet cores_;
  HeaderInjector header_injector_;
  IO io_;
};
}  // namespace node_pipeline
