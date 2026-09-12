#pragma once
#include <array>
#include <cmath>
namespace fr3_inspection {
struct ContactEvidence {
  std::array<double, 2> seen{-1, -1};
  std::array<double, 2> since{-1, -1};
  void sample(double now, const std::array<double, 2> &depth) {
    for (size_t i=0; i<2; ++i) {
      if (seen[i] > now || now-seen[i] > .03) since[i] = seen[i] = -1;
      if (!std::isfinite(depth[i]) || depth[i] > .0015) since[i] = seen[i] = -1;
      else if (depth[i] >= 0) {
        if (since[i] < 0) since[i] = now;
        seen[i] = now;
      }
    }
  }
  bool ready(double now) const {
    return std::isfinite(now) && seen[0] >= 0 && seen[1] >= 0 &&
      since[0] >= 0 && since[1] >= 0 &&
      now >= seen[0] && now >= seen[1] && now-seen[0] <= .03 && now-seen[1] <= .03 &&
      seen[0]-since[0] >= .10 && seen[1]-since[1] >= .10;
  }
};
}
