#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace fr3_inspection {
// One opening command, two physical fingers. No integral wind-up at contact.
struct LinkedFingerServo {
  double kp = 2000.0, kd = 20.0, sync_kp = 4000.0;
  double max_force = 4.0, max_speed = .006, reference = .0175;
  std::array<double, 2> step(double command, const std::array<double, 2> &q,
                            const std::array<double, 2> &v, double dt) {
    if (!std::isfinite(command) || !std::isfinite(dt) || dt <= 0 || dt > .1 ||
        !std::isfinite(q[0]) || !std::isfinite(q[1]) ||
        !std::isfinite(v[0]) || !std::isfinite(v[1])) return {0, 0};
    command = std::clamp(command, 0.0, .05);
    reference += std::clamp(command-reference, -max_speed*dt, max_speed*dt);
    const double sync = sync_kp*(q[1]-q[0]);
    return {std::clamp(kp*(reference-q[0])-kd*v[0]+sync, -max_force, max_force),
            std::clamp(kp*(reference-q[1])-kd*v[1]-sync, -max_force, max_force)};
  }
};
}  // namespace fr3_inspection
