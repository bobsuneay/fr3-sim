#pragma once
#include <array>
#include <cmath>
namespace fr3_inspection {
struct ContactEvidence {
  std::array<double, 2> seen{-1, -1};
  void sample(double now, const std::array<double, 2> &depth) {
    for (size_t i=0; i<2; ++i) {
      if (seen[i] > now) seen[i] = -1;
      if (!std::isfinite(depth[i]) || depth[i] > .0015) seen[i] = -1;
      else if (depth[i] >= 0) seen[i] = now;
    }
  }
  bool ready(double now) const {
    return std::isfinite(now) && seen[0] >= 0 && seen[1] >= 0 &&
      now >= seen[0] && now >= seen[1] && now-seen[0] <= .15 && now-seen[1] <= .15;
  }
};
}
