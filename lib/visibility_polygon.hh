#pragma once

#include "lib/geometry.hh"
#include <algorithm>
#include <vector>

namespace vis {

template <typename T, typename P = geometry::Point2_t<T>>
std::vector<P> raycast2d(const std::vector<geometry::Segment<P>> &segs,
                         const P &obsv_pt, T view_radius, T dtheta) {
  using geometry::add, geometry::find_intersection, geometry::sub,
      geometry::length_squared;
  using Seg = geometry::Segment<P>;

  std::vector<P> intersections, visibility_polygon;
  intersections.reserve(segs.size());
  visibility_polygon.reserve(static_cast<size_t>((2 * 3.2) / dtheta + 1));

  for (T th = 0.0; th < 2 * 3.1416; th += dtheta) {
    const P fov_end{view_radius * std::cos(th), view_radius * std::sin(th)};
    const Seg sightline{obsv_pt, add(obsv_pt, fov_end)};

    // Loop over segments and apply find_intersection(sightline, seg)
    intersections.clear();
    intersections.push_back(sightline.second);
    std::for_each(segs.begin(), segs.end(),
                  [&sightline, &intersections](const Seg &s) {
                    if (auto pt = find_intersection(sightline, s)) {
                      intersections.push_back(pt.value_or(sightline.second));
                    }
                  });
    // Find nearest intersection point.
    auto nearest_pt =
        std::min_element(intersections.begin(), intersections.end(),
                         [&obsv_pt](const auto &p1, const auto &p2) {
                           auto d1 = sub(p1, obsv_pt), d2 = sub(p2, obsv_pt);
                           return length_squared(d1) < length_squared(d2);
                         });
    visibility_polygon.push_back(*nearest_pt);
  }
  return visibility_polygon;
};

} // namespace vis
