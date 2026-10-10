#include <traffic_light/parameters/yolo_traffic_light_mac.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cstring>
#include <thread>

namespace {
class InferenceServer {
 public:
  explicit InferenceServer(bool valid = true) : valid_(valid) {
    socket_ = socket(AF_INET, SOCK_STREAM, 0);
    const int enabled = 1;
    setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    ready = bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 &&
            listen(socket_, 4) == 0;
    socklen_t length = sizeof(address);
    getsockname(socket_, reinterpret_cast<sockaddr*>(&address), &length);
    url = "http://127.0.0.1:" + std::to_string(ntohs(address.sin_port)) + "/infer";
    if (ready) worker_ = std::thread([this] { serve(); });
  }
  ~InferenceServer() {
    running_ = false;
    shutdown(socket_, SHUT_RDWR);
    close(socket_);
    if (worker_.joinable()) worker_.join();
  }
  bool ready = false;
  std::string url;
  std::atomic<int> requests{0};

 private:
  void serve() {
    while (running_) {
      const int client = accept(socket_, nullptr, nullptr);
      if (client < 0) continue;
      std::string header;
      char value = 0;
      while (recv(client, &value, 1, 0) == 1) {
        header.push_back(value);
        if (header.size() >= 4 && header.compare(header.size() - 4, 4, "\r\n\r\n") == 0) break;
      }
      const auto length_position = header.find("Content-Length:");
      const std::size_t length = length_position == std::string::npos ? 0 :
        static_cast<std::size_t>(std::stoul(header.substr(length_position + 15)));
      char buffer[65536];
      std::size_t received = 0;
      while (received < length) {
        const auto count = recv(client, buffer, std::min(sizeof(buffer), length - received), 0);
        if (count <= 0) break;
        received += static_cast<std::size_t>(count);
      }
      if (received == 19660800u) ++requests;
      const float detection[] = {640, 640, 20, 20, 0, 0, 0, 0.95F};
      const std::uint32_t shape[] = {8, 1};
      std::string body(valid_ ? "MORAIGPU" : "BADMAGIC", 8);
      body.append(reinterpret_cast<const char*>(shape), sizeof(shape));
      body.append(reinterpret_cast<const char*>(detection), sizeof(detection));
      const auto response = std::string("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Length: ") +
        std::to_string(body.size()) + "\r\n\r\n" + body;
      send(client, response.data(), response.size(), MSG_NOSIGNAL);
      close(client);
    }
  }
  int socket_ = -1;
  bool valid_;
  std::atomic<bool> running_{true};
  std::thread worker_;
};
}  // namespace

TEST(YoloTrafficLightMac, UnavailableBackendRemainsUnknownAndLeavesHeaderForPipeline) {
  ros::Time::init();
  auto parameters = traffic_light::loadYoloTrafficLightMacParameters(
      node_pipeline::ParamReader::fromFile(
          NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light_mac.yaml", NP_PACKAGE_DIR));
  ASSERT_EQ(3u, parameters.detection.minimum_votes);
  parameters.inference_url = "http://127.0.0.1:1/infer";
  sensor_msgs::CompressedImage frame;
  frame.format = "jpeg";
  frame.header.seq = 42;
  frame.header.stamp = ros::Time(10, 0);
  frame.header.frame_id = "camera_7_optical_frame";
  const cv::Mat black(parameters.input_height, parameters.input_width, CV_8UC3, cv::Scalar::all(0));
  ASSERT_TRUE(cv::imencode(".jpg", black, frame.data));

  traffic_light::YoloTrafficLightMac core(parameters);
  const auto light = core.process(frame);

  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, light.state);
  EXPECT_EQ(0u, light.header.seq);
  EXPECT_TRUE(light.header.stamp.isZero());
  EXPECT_TRUE(light.header.frame_id.empty());
}

TEST(YoloTrafficLightMac, UsesRemoteDetectionsAndPreservesVoting) {
  InferenceServer server;
  ASSERT_TRUE(server.ready);
  auto parameters = traffic_light::loadYoloTrafficLightMacParameters(
      node_pipeline::ParamReader::fromFile(
          NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light_mac.yaml", NP_PACKAGE_DIR));
  parameters.inference_url = server.url;
  traffic_light::YoloTrafficLightMac core(parameters);
  sensor_msgs::CompressedImage frame;
  const cv::Mat black(480, 640, CV_8UC3, cv::Scalar::all(0));
  ASSERT_TRUE(cv::imencode(".jpg", black, frame.data));
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(frame).state);
  EXPECT_EQ(3, server.requests.load());
}

TEST(YoloTrafficLightMac, RejectsMalformedInferenceResponse) {
  ros::Time::init();
  InferenceServer server(false);
  ASSERT_TRUE(server.ready);
  auto parameters = traffic_light::loadYoloTrafficLightMacParameters(
      node_pipeline::ParamReader::fromFile(
          NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light_mac.yaml", NP_PACKAGE_DIR));
  parameters.inference_url = server.url;
  traffic_light::YoloTrafficLightMac core(parameters);
  sensor_msgs::CompressedImage frame;
  const cv::Mat black(480, 640, CV_8UC3, cv::Scalar::all(0));
  ASSERT_TRUE(cv::imencode(".jpg", black, frame.data));
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(1, server.requests.load());
}

TEST(YoloTrafficLightMac, RejectsDifferentDimensionsWithTheSameTensorByteCount) {
  const auto config = node_pipeline::ParamReader::fromString("input_width: 640\ninput_height: 2560\n");
  EXPECT_THROW(traffic_light::loadYoloTrafficLightMacParameters(config), std::invalid_argument);
}
