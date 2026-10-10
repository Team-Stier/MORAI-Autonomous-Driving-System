
#pragma once

#include <interfaces/Objects.h>
#include <vector>
#include <cstdint>
#include <limits>

namespace object_detector {

class Tracking {
private:
    // 1. Kalman Filter: 위치와 속도 추정
    struct Kalman {
        double p = 0.0;
        double v = 0.0;

        double p00 = 1.0;
        double p01 = 0.0;
        double p10 = 0.0;
        double p11 = 25.0;

        void predict(double dt) {
            const double q = 4.0;

            p += v * dt;

            double a = p00 + dt * (p01 + p10)
                     + dt * dt * p11
                     + q * dt * dt * dt * dt / 4.0;
            double b = p01 + dt * p11
                     + q * dt * dt * dt / 2.0;
            double c = p10 + dt * p11
                     + q * dt * dt * dt / 2.0;
            double d = p11 + q * dt * dt;

            p00 = a;
            p01 = b;
            p10 = c;
            p11 = d;
        }

        void update(double measurement) {
            const double r = 0.09;

            double k0 = p00 / (p00 + r);
            double k1 = p10 / (p00 + r);
            double error = measurement - p;

            p += k0 * error;
            v += k1 * error;

            double a = p00;
            double b = p01;

            p00 -= k0 * a;
            p01 -= k0 * b;
            p10 -= k1 * a;
            p11 -= k1 * b;
        }
    };

    struct Track {
        uint32_t id = 0;
        Kalman x;
        Kalman y;
        Kalman z;
        bool have_velocity = false;
    };

    std::vector<Track> tracks_;
    uint32_t next_id_ = 1;
    double last_time_ = 0.0;

    // 2. Hungarian Algorithm: 전체 객체 최적 매칭
    std::vector<int> match(
        const interfaces::Objects& input) {

        int n = static_cast<int>(tracks_.size());
        int m = static_cast<int>(input.objects.size());

        std::vector<int> result(m, -1);
        if (n == 0 || m == 0) return result;

        const double max_dist_sq = 4.0;
        const double INF = std::numeric_limits<double>::infinity();
        int cols = m + n;

        std::vector<double> u(n + 1, 0.0);
        std::vector<double> v(cols + 1, 0.0);
        std::vector<int> p(cols + 1, 0);
        std::vector<int> way(cols + 1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;

            std::vector<double> minv(cols + 1, INF);
            std::vector<bool> used(cols + 1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = INF;
                int j1 = 0;

                for (int j = 1; j <= cols; ++j) {
                    if (used[j]) continue;

                    double cost = max_dist_sq;

                    if (j <= m) {
                        double dx = tracks_[i0 - 1].x.p
                                  - input.objects[j - 1].position.x;
                        double dy = tracks_[i0 - 1].y.p
                                  - input.objects[j - 1].position.y;

                        cost = dx * dx + dy * dy;
                    }

                    double cur = cost - u[i0] - v[j];

                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }

                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }

                for (int j = 0; j <= cols; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;

            } while (p[j0] != 0);

            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // 매칭 결과: 현재 객체 -> 이전 Track
        for (int j = 1; j <= m; ++j) {
            if (p[j] == 0) continue;

            int i = p[j] - 1;

            double dx = tracks_[i].x.p
                      - input.objects[j - 1].position.x;
            double dy = tracks_[i].y.p
                      - input.objects[j - 1].position.y;

            if (dx * dx + dy * dy < max_dist_sq) {
                result[j - 1] = i;
            }
        }

        return result;
    }


public:
    interfaces::Objects process(
        const interfaces::Objects& input,
        const ros::Time& stamp) {

        interfaces::Objects output = input;

        double now = stamp.toSec();
        double dt = 0.0;

        if (last_time_ != 0.0)
            dt = now - last_time_;

        last_time_ = now;

        // Kalman Filter 위치 예측 (map 기준)
        for (auto& track : tracks_) {
            track.x.predict(dt);
            track.y.predict(dt);
            track.z.predict(dt);
        }

        // Hungarian 객체 매칭 (map 기준)
        std::vector<int> matches = match(input);
        std::vector<Track> next_tracks;

        for (size_t i = 0; i < output.objects.size(); ++i) {
            auto& object = output.objects[i];
            const auto& measured = input.objects[i];

            Track track;
            bool matched = matches[i] >= 0;
            geometry_msgs::Vector3 acceleration;
            bool have_acceleration = false;

            if (matched) {
                track = tracks_[matches[i]];
                const double vx = track.x.v, vy = track.y.v, vz = track.z.v;
                track.x.update(measured.position.x);
                track.y.update(measured.position.y);
                track.z.update(measured.position.z);
                have_acceleration = track.have_velocity && dt > 0;
                if (have_acceleration) {
                    acceleration.x = (track.x.v - vx) / dt;
                    acceleration.y = (track.y.v - vy) / dt;
                    acceleration.z = (track.z.v - vz) / dt;
                }
            } else {
                track.id = next_id_++;
                track.x.p = measured.position.x;
                track.y.p = measured.position.y;
                track.z.p = measured.position.z;
            }

            object.id = track.id;

            object.velocity.x = track.x.v;
            object.velocity.y = track.y.v;
            object.velocity.z = track.z.v;
            object.acceleration = acceleration;

            track.have_velocity = matched && dt > 0;
            object.velocity_available = track.have_velocity;
            object.acceleration_available = have_acceleration;

            next_tracks.push_back(track);
        }

        tracks_ = next_tracks;

        return output;
    }
};

}  // namespace object_detector
