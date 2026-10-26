#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace traffic_light {

struct Camera7LampCpuParameters {
  // Upper image band. camera_7 looks forward, so lamps are above the road.
  double search_bottom_ratio = 0.58;
  int min_area = 4;
  int max_area = 280;
  int max_span = 36;
  double max_aspect = 2.6;
  double link_distance_px = 28.0;
  double center_sigma_px = 90.0;
  double ambiguity_ratio = 0.85;
  int color_margin = 18;
  // Identical non-UNKNOWN readings required before a state is returned.
  int minimum_repeat = 2;
};

class Camera7LampCpu {
 public:
  explicit Camera7LampCpu(Camera7LampCpuParameters value) : parameters_(value) {}

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message) {
    interfaces::TrafficLight result;
    const cv::Mat image = cv::imdecode(message.data, cv::IMREAD_COLOR);
    const std::uint8_t state = image.empty() || image.type() != CV_8UC3
        ? static_cast<std::uint8_t>(interfaces::TrafficLight::UNKNOWN)
        : readFrame(image);
    result.state = confirm(state);
    return result;
  }

 private:
  struct Lamp {
    int kind;
    int area;
    double span;
    double cx;
    double cy;
  };

  std::uint8_t confirm(std::uint8_t state) {
    if (parameters_.minimum_repeat <= 1) {
      previous_ = state;
      return state;
    }
    const bool agreed = state == previous_ && state != interfaces::TrafficLight::UNKNOWN;
    previous_ = state;
    return agreed ? state : static_cast<std::uint8_t>(interfaces::TrafficLight::UNKNOWN);
  }

  std::uint8_t readFrame(const cv::Mat& image) const {
    const int y_max = std::max(0, std::min(
        image.rows, static_cast<int>(parameters_.search_bottom_ratio * image.rows)));
    if (y_max < 1 || image.cols < 1) return interfaces::TrafficLight::UNKNOWN;
    cv::Mat red_mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat yellow_mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat green_mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
    const int margin = parameters_.color_margin;
    for (int y = 0; y < y_max; ++y) {
      const cv::Vec3b* row = image.ptr<cv::Vec3b>(y);
      std::uint8_t* red_row = red_mask.ptr<std::uint8_t>(y);
      std::uint8_t* yellow_row = yellow_mask.ptr<std::uint8_t>(y);
      std::uint8_t* green_row = green_mask.ptr<std::uint8_t>(y);
      for (int x = 0; x < image.cols; ++x) {
        const int blue = row[x][0];
        const int green = row[x][1];
        const int red = row[x][2];
        const bool is_red = red > 70 && red > green + margin && red > blue + margin &&
            green * 100 < red * 62 && blue * 100 < red * 62;
        const bool is_green = green > 60 && green > red + margin && green > blue + margin &&
            red * 100 < green * 85 && blue * 100 < green * 85;
        const bool is_yellow = red > 80 && green > 70 && blue < 150 && red > blue + 30 &&
            green > blue + 30 && std::abs(red - green) < 90 && !is_red && !is_green;
        if (is_red) red_row[x] = 1;
        if (is_green) green_row[x] = 1;
        if (is_yellow) yellow_row[x] = 1;
      }
    }
    std::vector<Lamp> lamps;
    collect(red_mask, 0, lamps);
    collect(yellow_mask, 1, lamps);
    collect(green_mask, 2, lamps);
    if (lamps.empty()) return interfaces::TrafficLight::UNKNOWN;

    std::vector<int> parent(lamps.size());
    for (int i = 0; i < static_cast<int>(lamps.size()); ++i) parent[i] = i;
    const auto find = [&parent](int index) {
      while (parent[index] != index) {
        parent[index] = parent[parent[index]];
        index = parent[index];
      }
      return index;
    };
    const double limit = parameters_.link_distance_px * parameters_.link_distance_px;
    for (int i = 0; i < static_cast<int>(lamps.size()); ++i) {
      for (int j = i + 1; j < static_cast<int>(lamps.size()); ++j) {
        const double dx = lamps[i].cx - lamps[j].cx;
        const double dy = lamps[i].cy - lamps[j].cy;
        if (dx * dx + dy * dy <= limit) parent[find(i)] = find(j);
      }
    }
    std::vector<std::vector<Lamp>> groups(lamps.size());
    for (int i = 0; i < static_cast<int>(lamps.size()); ++i) groups[find(i)].push_back(lamps[i]);

    const double center = image.cols * 0.5;
    const double sigma2 = 2.0 * parameters_.center_sigma_px * parameters_.center_sigma_px;
    double best_score = 0;
    std::uint8_t best_bits = 0;
    std::vector<std::pair<double, std::uint8_t>> scored;
    for (const auto& group : groups) {
      if (group.empty()) continue;
      int area = 0;
      double weighted_x = 0;
      for (const auto& lamp : group) {
        area += lamp.area;
        weighted_x += lamp.area * lamp.cx;
      }
      const double dx = weighted_x / area - center;
      const double score = area * std::exp(-(dx * dx) / sigma2);
      scored.emplace_back(score, bitsOf(group));
      if (score > best_score) {
        best_score = score;
        best_bits = scored.back().second;
      }
    }
    if (best_score <= 0 || best_bits == 0) return interfaces::TrafficLight::UNKNOWN;
    for (const auto& item : scored) {
      if (item.first > parameters_.ambiguity_ratio * best_score && item.second != best_bits)
        return interfaces::TrafficLight::UNKNOWN;
    }
    return stateFor(best_bits);
  }

  static std::uint8_t stateFor(std::uint8_t bits) {
    switch (bits) {
      case 1: return interfaces::TrafficLight::RED;
      case 2: return interfaces::TrafficLight::YELLOW;
      case 3: return interfaces::TrafficLight::RED_YELLOW;
      case 4: return interfaces::TrafficLight::GREEN;
      case 5: return interfaces::TrafficLight::RED_GREEN;
      case 6: return interfaces::TrafficLight::YELLOW_GREEN;
      case 7: return interfaces::TrafficLight::RED_YELLOW_GREEN;
      case 8: return interfaces::TrafficLight::LEFT;
      case 9: return interfaces::TrafficLight::RED_LEFT;
      case 10: return interfaces::TrafficLight::YELLOW_LEFT;
      case 11: return interfaces::TrafficLight::RED_YELLOW_LEFT;
      case 12: return interfaces::TrafficLight::GREEN_LEFT;
      case 13: return interfaces::TrafficLight::RED_GREEN_LEFT;
      case 14: return interfaces::TrafficLight::YELLOW_GREEN_LEFT;
      case 15: return interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT;
      default: return interfaces::TrafficLight::UNKNOWN;
    }
  }

  void collect(const cv::Mat& mask, int kind, std::vector<Lamp>& lamps) const {
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    for (int index = 1; index < count; ++index) {
      const int* stat = stats.ptr<int>(index);
      const int area = stat[cv::CC_STAT_AREA];
      const int width = stat[cv::CC_STAT_WIDTH];
      const int height = stat[cv::CC_STAT_HEIGHT];
      const int span = std::max(width, height);
      const int minor = std::max(1, std::min(width, height));
      if (area < parameters_.min_area || area > parameters_.max_area || span > parameters_.max_span)
        continue;
      if (span * 10 > static_cast<int>(parameters_.max_aspect * 10.0) * minor) continue;
      const double* centroid = centroids.ptr<double>(index);
      lamps.push_back(Lamp{kind, area, static_cast<double>(span), centroid[0], centroid[1]});
    }
  }

  // bit 0 red, bit 1 yellow, bit 2 circular green, bit 3 left arrow.
  // A red lamp with a green lamp in the same housing is the left arrow.
  // Circular green is a green lamp with no red lamp in that housing.
  static std::uint8_t bitsOf(const std::vector<Lamp>& lamps) {
    std::vector<const Lamp*> greens;
    bool red = false;
    bool yellow = false;
    for (const auto& lamp : lamps) {
      if (lamp.kind == 0) red = true;
      else if (lamp.kind == 1) yellow = true;
      else if (lamp.kind == 2) greens.push_back(&lamp);
    }
    std::sort(greens.begin(), greens.end(),
              [](const Lamp* a, const Lamp* b) { return a->cx < b->cx; });
    bool disk = false;
    bool arrow = false;
    if (!greens.empty() && red) {
      arrow = true;
    } else if (greens.size() >= 2) {
      disk = true;
      const double gap = greens.back()->cx - greens.front()->cx;
      double scale = 0;
      for (const auto* lamp : greens) scale = std::max(scale, lamp->span);
      if (gap > std::max(8.0, 1.25 * scale)) arrow = true;
    } else if (greens.size() == 1) {
      disk = true;
    }
    std::uint8_t bits = 0;
    if (red) bits = static_cast<std::uint8_t>(bits | 1);
    if (yellow) bits = static_cast<std::uint8_t>(bits | 2);
    if (disk) bits = static_cast<std::uint8_t>(bits | 4);
    if (arrow) bits = static_cast<std::uint8_t>(bits | 8);
    return bits;
  }

  Camera7LampCpuParameters parameters_;
  std::uint8_t previous_ = 255;
};

}  // namespace traffic_light
