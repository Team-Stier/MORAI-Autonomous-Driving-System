// 모든 실행 노드가 공유하는 main이다. np_registry.hpp와 NP_NODE_CONFIG_DIR는 빌드 디렉터리에 생성된다.
#include "np_registry.hpp"
#include <node_pipeline/detail/ros.hpp>
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  namespace npd = node_pipeline::detail;
  using Spec = np_generated::Spec;
  using Output = Spec::Output;
  try {
    ros::init(argc, argv, std::string(Spec::node()) + "_node");
    ros::NodeHandle nh;
    const npd::NodeParams params{Spec::package(), Spec::node(), NP_NODE_CONFIG_DIR, Spec::coreNames(), Spec::parameterLoaders()};

    std::vector<std::string> topics;
    const auto inputs = Spec::inputRegistry();
    const auto settings = npd::loadRosConfig(nh, params, topics, inputs);
    auto stages = npd::makeStages<Spec::CoreSet>(Spec::Schemas{}, [&](auto schema) {
      using Schema = typename decltype(schema)::type;
      const auto& stage_params = settings.stages.at(Schema::index);
      const auto input_registry = Schema::inputRegistry();
      auto core_profiles = Schema::coreProfiles();
      npd::CoreSelector<Spec::CoreId> selector;
      npd::loadStageConfig(stage_params, input_registry, core_profiles, selector);
      const auto dispatch = &Schema::processCore;
      return npd::makeStage(core_profiles, selector, dispatch);
    });

    auto cores = npd::createCoreObjects(Spec::Banks{}, Spec::Cores{}, settings.parameters, [](auto&&... core_objects) {
      return Spec::CoreSet{std::forward<decltype(core_objects)>(core_objects)...};
    });

    const auto execution = settings.execution;
    Spec::Buffer buffer(settings.queue_capacity);
    npd::SnapshotAssembler<Spec::Buffer> assembler(settings.reference_input);
    const npd::RosHeaderInjector<Output> header_injector{settings.output_frame};
    const npd::RosIO<Output> io{nh.advertise<Output>(settings.output_topic, 1, execution.mode == npd::ExecutionMode::Latched)};
    using Node = npd::PipelineNode<Spec::Buffer, npd::SnapshotAssembler<Spec::Buffer>, decltype(stages), Spec::CoreSet,
                                   npd::RosHeaderInjector<Output>, npd::RosIO<Output>>;
    Node node(execution, buffer, assembler, std::move(stages), std::move(cores), header_injector, io);

    using Callbacks = npd::RosCallbacks<Node>;
    Callbacks callbacks{node};
    const auto subscribers = npd::subscribeInputs<Spec::Inputs>(nh, topics, callbacks);
    ros::Subscriber section;
    if (!settings.section_topic.empty()) section = nh.subscribe(settings.section_topic, 1, &Callbacks::onSection, &callbacks);
    ros::SteadyTimer timer;
    if (execution.mode == npd::ExecutionMode::Periodic)
      timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &Callbacks::onPeriodic, &callbacks);
    if (execution.mode == npd::ExecutionMode::Latched) node.onStartup();
    ros::spin();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[node_pipeline] " << error.what() << std::endl;
    return 1;
  }
}
