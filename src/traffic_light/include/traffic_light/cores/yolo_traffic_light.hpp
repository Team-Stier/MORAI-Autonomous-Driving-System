#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light {

struct YoloTrafficLightParameters {
  struct DetectionParameters {
    std::array<float, 4> confidence{{0.4F, 0.4F, 0.35F, 0.4F}};
    float nms_iou = 0.45F;
    float ambiguity_margin = 0.05F;
    std::size_t history_size = 5;
    std::size_t minimum_votes = 3;
    bool unknown_votes = true;  // false: UNKNOWN frames stay in the history but do not vote.
    // Candidates whose box centre lies below this source-image row are dropped (lights stand above the
    // camera, so they appear above the horizon). Non-positive keeps every row.
    double maximum_center_y_px = 0.0;
    // true: a LEFT result reports RED_LEFT when a red lamp is lit in the left-turn box, GREEN_LEFT
    // when only green lamps are lit (TrackParameters colour bounds), otherwise UNKNOWN (combination
    // not determined).
    bool left_combination = false;
  };
  // A candidate below its class confidence is accepted when its box overlaps a head accepted within
  // timeout_sec and the lit lamp colour inside the box matches its class (go: green, stop: red,
  // prepare_stop: yellow). Hue is OpenCV HSV (0-180).
  struct TrackParameters {
    float minimum_confidence = 0.05F;
    float minimum_iou = 0.3F;
    double timeout_sec = 3.0;
    int minimum_lit_pixels = 3;
    int lit_saturation = 100;
    int lit_value = 120;
    int red_hue_below = 10;
    int red_hue_above = 165;
    std::array<int, 2> yellow_hue{{12, 35}};
    std::array<int, 2> green_hue{{40, 95}};
  };

  // Uniform-airlight haze removal before inference: airlight A is the mean colour of the brightest
  // airlight_fraction of the dark channel (per-pixel channel minimum eroded by window px), and
  // J = (I - A) / transmission + A. transmission 1 leaves the image unchanged.
  struct DehazeParameters {
    double transmission = 1.0;
    int window = 15;
    double airlight_fraction = 0.001;
  };

  std::string model;
  int input_width = 1280;
  int input_height = 1280;
  DetectionParameters detection;
  TrackParameters track;
  DehazeParameters dehaze;
};

class YoloTrafficLight {
 public:
  enum class DetectionState : std::uint8_t { Unknown, Go, LeftTurn, PrepareStop, Stop };

  // decodeYoloOutput gives the box in network-input pixels; process() maps it to source pixels.
  struct Detection {
    cv::Rect2f box;
    float confidence;
    DetectionState state;
  };

  struct Track {
    cv::Rect2f box;
    double stamp_sec;
  };

  struct LitPixels {
    int red = 0;
    int yellow = 0;
    int green = 0;
  };

  explicit YoloTrafficLight(YoloTrafficLightParameters value)
      : parameters(std::move(value)), env(ORT_LOGGING_LEVEL_WARNING, "traffic_light") {
    OrtCUDAProviderOptions cuda_options{};
    options.AppendExecutionProvider_CUDA(cuda_options);
    session.reset(new Ort::Session(env, parameters.model.data(), parameters.model.size(), options));
    input_name = session->GetInputNameAllocated(0, allocator).get();
    output_name = session->GetOutputNameAllocated(0, allocator).get();
  }

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message) {
    cv::Mat image = cv::imdecode(message.data, cv::IMREAD_COLOR);
    const cv::Mat input = dehaze(image, parameters.dehaze);
    const float scale = std::min(parameters.input_width / static_cast<float>(image.cols),
                                 parameters.input_height / static_cast<float>(image.rows));
    const int resized_width = static_cast<int>(std::round(image.cols * scale));
    const int resized_height = static_cast<int>(std::round(image.rows * scale));
    cv::Mat resized;
    cv::resize(input, resized, {resized_width, resized_height});
    const int left = (parameters.input_width - resized_width) / 2;
    const int right = parameters.input_width - resized_width - left;
    const int top = (parameters.input_height - resized_height) / 2;
    const int bottom = parameters.input_height - resized_height - top;
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0,
                                          {parameters.input_width, parameters.input_height},
                                          cv::Scalar(), true, false, CV_32F);
    const std::array<std::int64_t, 4> input_shape{{
        1, 3, parameters.input_height, parameters.input_width}};
    auto tensor = Ort::Value::CreateTensor<float>(
        memory, blob.ptr<float>(), blob.total(), input_shape.data(), input_shape.size());
    const char* input_names[] = {input_name.c_str()};
    const char* output_names[] = {output_name.c_str()};
    auto outputs = session->Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1,
                                output_names, 1);
    const auto shape = outputs.front().GetTensorTypeAndShapeInfo().GetShape();
    auto candidates = decodeYoloOutput(outputs.front().GetTensorData<float>(), shape, parameters);
    for (auto& candidate : candidates) {
      candidate.box = cv::Rect2f((candidate.box.x - left) / scale, (candidate.box.y - top) / scale,
                                 candidate.box.width / scale, candidate.box.height / scale);
    }
    candidates = aboveRow(candidates, parameters.detection.maximum_center_y_px);
    const auto accepted = acceptDetections(candidates, image, message.header.stamp.toSec(),
                                           parameters, tracks);
    const DetectionState state =
        pushState(chooseFrameState(accepted, parameters.detection.ambiguity_margin));
    interfaces::TrafficLight output;
    output.state = toRosState(state);
    if (state == DetectionState::LeftTurn && parameters.detection.left_combination) {
      const double stamp_sec = message.header.stamp.toSec();
      const auto left = std::find_if(accepted.begin(), accepted.end(), [](const Detection& item) {
        return item.state == DetectionState::LeftTurn;
      });
      if (left != accepted.end()) {
        left_lamp = litPixels(image, left->box, parameters.track);
        left_lamp_stamp_sec = stamp_sec;
      }
      const bool recent = stamp_sec >= left_lamp_stamp_sec &&
                          stamp_sec - left_lamp_stamp_sec <= parameters.track.timeout_sec;
      output.state = recent ? leftCombination(left_lamp, parameters.track.minimum_lit_pixels)
                            : interfaces::TrafficLight::UNKNOWN;
    }
    return output;
  }

  static cv::Mat dehaze(const cv::Mat& image,
                        const YoloTrafficLightParameters::DehazeParameters& parameters) {
    if (parameters.transmission >= 1.0) return image;
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::Mat minimum;
    cv::min(channels[0], channels[1], minimum);
    cv::min(minimum, channels[2], minimum);
    cv::Mat dark;
    cv::erode(minimum, dark,
              cv::getStructuringElement(cv::MORPH_RECT, {parameters.window, parameters.window}));
    std::array<double, 256> histogram{};
    for (int row = 0; row < dark.rows; ++row) {
      const std::uint8_t* values = dark.ptr<std::uint8_t>(row);
      for (int column = 0; column < dark.cols; ++column) ++histogram[values[column]];
    }
    const double wanted = std::max(1.0, parameters.airlight_fraction * static_cast<double>(dark.total()));
    int threshold = 255;
    for (double count = histogram[255]; threshold > 0 && count < wanted; count += histogram[threshold]) {
      --threshold;
    }
    const cv::Scalar airlight = cv::mean(image, dark >= threshold);
    cv::Mat value;
    image.convertTo(value, CV_32FC3);
    value = (value - airlight) * (1.0 / parameters.transmission) + airlight;
    cv::Mat output;
    value.convertTo(output, CV_8UC3);
    return output;
  }

  // Boxes in network-input pixels; keeps candidates down to the track minimum confidence.
  static std::vector<Detection> decodeYoloOutput(
      const float* data, const std::vector<std::int64_t>& shape,
      const YoloTrafficLightParameters& parameters) {
    const bool channels_first = shape[1] == 8;
    const std::size_t count = static_cast<std::size_t>(channels_first ? shape[2] : shape[1]);
    const auto value = [&](std::size_t row, std::size_t channel) {
      return channels_first ? data[channel * count + row] : data[row * 8 + channel];
    };
    std::array<float, 4> threshold;
    for (std::size_t class_id = 0; class_id < threshold.size(); ++class_id) {
      threshold[class_id] = std::min(parameters.detection.confidence[class_id],
                                     parameters.track.minimum_confidence);
    }

    std::array<std::vector<cv::Rect2d>, 4> boxes;
    std::array<std::vector<float>, 4> scores;
    for (std::size_t row = 0; row < count; ++row) {
      int best_class = 0;
      float best_score = value(row, 4);
      for (int class_id = 1; class_id < 4; ++class_id) {
        const float score = value(row, 4 + class_id);
        if (score > best_score) {
          best_class = class_id;
          best_score = score;
        }
      }
      if (best_score < threshold[best_class]) continue;
      const float width = value(row, 2);
      const float height = value(row, 3);
      if (width <= 0 || height <= 0) continue;
      boxes[best_class].emplace_back(value(row, 0) - width / 2,
                                     value(row, 1) - height / 2, width, height);
      scores[best_class].push_back(best_score);
    }

    std::vector<Detection> detections;
    for (int class_id = 0; class_id < 4; ++class_id) {
      std::vector<int> kept;
      cv::dnn::NMSBoxes(boxes[class_id], scores[class_id], threshold[class_id],
                        parameters.detection.nms_iou, kept);
      for (const int index : kept) {
        const cv::Rect2d& box = boxes[class_id][index];
        detections.push_back({cv::Rect2f(box), scores[class_id][index], stateForClass(class_id)});
      }
    }
    std::stable_sort(detections.begin(), detections.end(),
                     [](const Detection& a, const Detection& b) {
                       return a.confidence > b.confidence;
                     });
    return detections;
  }

  // Candidates whose box centre row is at most maximum_center_y_px (all when it is not positive).
  static std::vector<Detection> aboveRow(const std::vector<Detection>& candidates,
                                         double maximum_center_y_px) {
    if (maximum_center_y_px <= 0) return candidates;
    std::vector<Detection> kept;
    for (const auto& candidate : candidates) {
      if (candidate.box.y + candidate.box.height / 2 <= maximum_center_y_px) kept.push_back(candidate);
    }
    return kept;
  }

  // Confident candidates, plus colour-verified weak candidates on a tracked head, in candidate
  // (confidence) order; refreshes tracks.
  static std::vector<Detection> acceptDetections(
      const std::vector<Detection>& candidates, const cv::Mat& image, double stamp_sec,
      const YoloTrafficLightParameters& parameters, std::vector<Track>& tracks) {
    const auto& track = parameters.track;
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [&](const Track& item) {
                   return stamp_sec < item.stamp_sec || stamp_sec - item.stamp_sec > track.timeout_sec;
                 }),
                 tracks.end());
    std::vector<Detection> accepted;
    for (const auto& candidate : candidates) {
      if (candidate.confidence >= parameters.detection.confidence[classIndex(candidate.state)]) {
        accepted.push_back(candidate);
        continue;
      }
      const bool tracked = std::any_of(tracks.begin(), tracks.end(), [&](const Track& item) {
        return overlap(item.box, candidate.box) >= track.minimum_iou;
      });
      if (tracked && candidate.confidence >= track.minimum_confidence &&
          lampMatches(candidate.state, litPixels(image, candidate.box, track),
                      track.minimum_lit_pixels)) {
        accepted.push_back(candidate);
      }
    }
    for (const auto& detection : accepted) {
      tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [&](const Track& item) {
                     return overlap(item.box, detection.box) >= track.minimum_iou;
                   }),
                   tracks.end());
      tracks.push_back({detection.box, stamp_sec});
    }
    return accepted;
  }

  // Lit lamp pixels per colour inside the box widened by 2 px, clipped to the image.
  static LitPixels litPixels(const cv::Mat& image, const cv::Rect2f& box,
                             const YoloTrafficLightParameters::TrackParameters& track) {
    LitPixels lit;
    if (!std::isfinite(box.x) || !std::isfinite(box.y) || !std::isfinite(box.width) ||
        !std::isfinite(box.height)) {
      return lit;
    }
    const int x0 =std::max(0, static_cast<int>(std::floor(box.x)) - 2);
    const int y0 = std::max(0, static_cast<int>(std::floor(box.y)) - 2);
    const int x1 = std::min(image.cols, static_cast<int>(std::ceil(box.x + box.width)) + 2);
    const int y1 = std::min(image.rows, static_cast<int>(std::ceil(box.y + box.height)) + 2);
    if (x1 <= x0 || y1 <= y0) return lit;
    cv::Mat hsv;
    cv::cvtColor(image(cv::Rect(x0, y0, x1 - x0, y1 - y0)), hsv, cv::COLOR_BGR2HSV);
    for (int row = 0; row < hsv.rows; ++row) {
      const cv::Vec3b* pixels = hsv.ptr<cv::Vec3b>(row);
      for (int column = 0; column < hsv.cols; ++column) {
        const cv::Vec3b& pixel = pixels[column];
        if (pixel[1] <= track.lit_saturation || pixel[2] <= track.lit_value) continue;
        const int hue = pixel[0];
        if (hue < track.red_hue_below || hue > track.red_hue_above) {
          ++lit.red;
        } else if (hue >= track.yellow_hue[0] && hue <= track.yellow_hue[1]) {
          ++lit.yellow;
        } else if (hue >= track.green_hue[0] && hue <= track.green_hue[1]) {
          ++lit.green;
        }
      }
    }
    return lit;
  }

  static bool lampMatches(DetectionState state, const LitPixels& lit, int minimum) {
    switch (state) {
      case DetectionState::Go: return lit.green >= minimum && lit.red == 0 && lit.yellow == 0;
      case DetectionState::Stop: return lit.red >= minimum && lit.green == 0 && lit.yellow == 0;
      case DetectionState::PrepareStop:
        return lit.yellow >= minimum && lit.red == 0 && lit.green == 0;
      case DetectionState::LeftTurn:
      case DetectionState::Unknown: return false;
    }
    return false;
  }

  static DetectionState chooseFrameState(const std::vector<Detection>& detections,
                                         float ambiguity_margin) {
    if (detections.empty()) return DetectionState::Unknown;
    const Detection& best = detections.front();
    for (const auto& detection : detections) {
      if (detection.state != best.state &&
          best.confidence - detection.confidence <= ambiguity_margin) {
        return DetectionState::Unknown;
      }
    }
    return best.state;
  }

  static std::uint8_t leftCombination(const LitPixels& lit, int minimum) {
    if (lit.red >= minimum) return interfaces::TrafficLight::RED_LEFT;
    if (lit.green >= minimum && lit.red == 0 && lit.yellow == 0) {
      return interfaces::TrafficLight::GREEN_LEFT;
    }
    return interfaces::TrafficLight::UNKNOWN;
  }

  static std::uint8_t toRosState(DetectionState state) {
    switch (state) {
      case DetectionState::Go: return interfaces::TrafficLight::GREEN;
      case DetectionState::LeftTurn: return interfaces::TrafficLight::LEFT;
      case DetectionState::PrepareStop: return interfaces::TrafficLight::YELLOW;
      case DetectionState::Stop: return interfaces::TrafficLight::RED;
      case DetectionState::Unknown: return interfaces::TrafficLight::UNKNOWN;
    }
    return interfaces::TrafficLight::UNKNOWN;
  }

  // State of the newest frame after the vote over history (newest last).
  static DetectionState vote(const std::deque<DetectionState>& history,
                             const YoloTrafficLightParameters::DetectionParameters& parameters) {
    if (history.empty() || history.back() == DetectionState::Unknown) return DetectionState::Unknown;
    std::array<std::size_t, 5> counts{{0, 0, 0, 0, 0}};
    for (const auto item : history) {
      if (parameters.unknown_votes || item != DetectionState::Unknown)
        ++counts[static_cast<std::size_t>(item)];
    }
    const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
    if (maximum < parameters.minimum_votes) return DetectionState::Unknown;
    for (auto iterator = history.rbegin(); iterator != history.rend(); ++iterator) {
      if (counts[static_cast<std::size_t>(*iterator)] == maximum) return *iterator;
    }
    return DetectionState::Unknown;
  }

 private:
  DetectionState pushState(DetectionState state) {
    history.push_back(state);
    if (history.size() > parameters.detection.history_size) history.pop_front();
    return vote(history, parameters.detection);
  }


  static DetectionState stateForClass(int class_id) {
    static const std::array<DetectionState, 4> states{{
        DetectionState::Go, DetectionState::LeftTurn,
        DetectionState::PrepareStop, DetectionState::Stop}};
    return states[static_cast<std::size_t>(class_id)];
  }

  static std::size_t classIndex(DetectionState state) {
    return static_cast<std::size_t>(state) - 1;
  }

  static float overlap(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float intersection = (a & b).area();
    const float united = a.area() + b.area() - intersection;
    return united > 0 ? intersection / united : 0.0F;
  }

  YoloTrafficLightParameters parameters;
  Ort::Env env;
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string input_name;
  std::string output_name;
  std::deque<DetectionState> history;
  std::vector<Track> tracks;
  LitPixels left_lamp;
  double left_lamp_stamp_sec = -1.0;
};

}  // namespace traffic_light
