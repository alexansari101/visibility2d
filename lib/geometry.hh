#pragma once

#include <algorithm>
#include <cmath> // std::abs/sin/cos/atan2
#include <optional>
#include <utility>

namespace geometry {

template <typename T> using Point2_t = std::pair<T, T>;

constexpr auto add = []<typename V>(const V &v1, const V &v2) {
  return std::make_pair(v1.first + v2.first, v1.second + v2.second);
};
constexpr auto sub = []<typename V>(const V &v1, const V &v2) {
  return std::make_pair(v1.first - v2.first, v1.second - v2.second);
};
constexpr auto cross = []<typename V>(const V &v1, const V &v2) {
  return v1.first * v2.second - v1.second * v2.first;
};
constexpr auto dot = []<typename V>(const V &v1, const V &v2) {
  return v1.first * v2.first + v1.second * v2.second;
};
constexpr auto length_squared = []<typename V>(const V &v) {
  return dot(v, v);
};
constexpr auto almost_equal = []<typename T>(const T &a, const T &b,
                                             const T &eps) {
  return std::abs(a - b) < eps;
}; // Todo: Only works for T as float/double

template <typename P> struct Segment {
  using point_t = P;
  Segment(P a, P b) : first(std::move(a)), second(std::move(b)){};
  P first;
  P second;
};

// Note: Ignores intersections if the segments are parallel or collinear.
template <typename P>
std::optional<P> find_intersection(const Segment<P> &s1, const Segment<P> &s2) {
  auto d1 = sub(s1.second, s1.first), d2 = sub(s2.second, s2.first);

  // implies parallel or collinear
  auto d1xd2 = cross(d1, d2);
  constexpr decltype(d1xd2) lo = 0.0, hi = 1.0, eps = 1e-6;
  if (almost_equal(d1xd2, lo, eps)) {
    // (q − p) × r = 0 -> implies they are collinear AND !=0 implies parallel
    return {};
  } else {
    auto d21 = sub(s2.first, s1.first);
    auto t = cross(d21, d2) / d1xd2;
    if (t != std::clamp(t, lo, hi))
      return {};
    auto u = cross(d21, d1) / d1xd2;
    if (u == std::clamp(u, lo, hi)) {
      auto &&td1 = P{t * d1.first, t * d1.second};
      auto intersection = add(s1.first, td1);
      return std::make_optional(intersection);
    }
  }
  return {};
};

} // namespace geometry
