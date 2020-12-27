#include "lib/utility.hh"
#include "lib/visibility_polygon.hh"
#include "gtest/gtest.h"
#include <algorithm> // std::sort
#include <cmath>     // std::atan2
#include <cstddef>
#include <iostream>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

namespace test_shapes {

// Construct a closed trapezoid (smaller top, larger bottom)
template <typename P> struct Trapezoid {
  geometry::Segment<P> bot{{-1., 1.}, {1., 1.}}, right{{1., 1.}, {.5, 1.5}},
      top{{.5, 1.5}, {-.5, 1.5}}, left{{-.5, 1.5}, {-1., 1.}};
};

} // namespace test_shapes

TEST(GEOM_TEST, PointTest) {
  // Construct equal points
  using Point2_t = geometry::Point2_t<float>;
  Point2_t p1{0.f, 0.f}, p2{0.f, 0.f};
  EXPECT_EQ(p1, p2);

  // Init unordered points
  p1 = {-1.0, 0.0};
  p2 = {-.707, .707};
  Point2_t p3{.707, -.707}, p4{-.707, -.707};
  std::vector pts = {p1, p2, p3, p4};

  // Sort points
  auto angle_comparer = [](auto p1, auto p2) {
    return std::atan2(p1.second, p1.first) < std::atan2(p2.second, p2.first);
  };
  std::sort(pts.begin(), pts.end(), angle_comparer);

  // Test ordering
  std::vector expected_order = {p4, p3, p2, p1};
  for (size_t i = 0; i < pts.size(); i++) {
    EXPECT_EQ(pts[i], expected_order[i]);
  }
}

TEST(GEOM_TEST, SegmentTest) {
  // Construct a segment
  using Point2_t = geometry::Point2_t<float>;
  using Seg = geometry::Segment<Point2_t>;
  EXPECT_NO_THROW(Seg({0.f, 0.f}, {1.f, 0.f}));

  // Compare segments
  // Construct a closed trapezoid (smaller top, larger bottom)
  const test_shapes::Trapezoid<Point2_t> trap;

  // Test for intersection.
  auto intersection_left =
      geometry::find_intersection(Seg{{0., 0.}, {0., 10.}}, trap.left);
  auto intersection_right =
      geometry::find_intersection(Seg{{0., 0.}, {0., 10.}}, trap.right);
  EXPECT_FALSE(intersection_left);
  EXPECT_FALSE(intersection_right);

  auto intersection_top =
      geometry::find_intersection(Seg{{0., 0.}, {0., 10.}}, trap.top);
  auto intersection_bot =
      geometry::find_intersection(Seg{{0., 0.}, {0., 10.}}, trap.bot);
  EXPECT_EQ(intersection_top, Point2_t({0.0, 1.5}));
  EXPECT_EQ(intersection_bot, Point2_t({0.0, 1.0}));
}

TEST(VIS_TEST, VisibilityPolygonTest) {
  // Use a segment to represent the observers field of view.
  using Point2_t = geometry::Point2_t<float>;
  using Seg = geometry::Segment<Point2_t>;

  // Construct a closed trapezoid (smaller top, larger bottom)
  const test_shapes::Trapezoid<Point2_t> trap;
  const std::vector segs{trap.bot, trap.left, trap.top, trap.right};
  auto vis_poly = vis::raycast2d(segs, {0., 3.5}, 10.0, 0.033);

  // Only top and side points should be in the visibility polygon.
  auto find_midpt = [](const Seg &seg, const auto &vis_poly) {
    auto mid = geometry::add(seg.first, seg.second);
    mid.first *= 0.5, mid.second *= 0.5;
    return std::find_if(
        vis_poly.begin(), vis_poly.end(), [&mid](const auto &pt) {
          auto l2 = std::sqrt(geometry::length_squared(geometry::sub(mid, pt)));
          return geometry::almost_equal(l2, 0.0f, 0.2f);
        });
  };
  ASSERT_FALSE(find_midpt(trap.left, vis_poly) == vis_poly.end());
  ASSERT_FALSE(find_midpt(trap.right, vis_poly) == vis_poly.end());
  ASSERT_FALSE(find_midpt(trap.top, vis_poly) == vis_poly.end());
  ASSERT_TRUE(find_midpt(trap.bot, vis_poly) == vis_poly.end());
}

TEST(UTIL_TEST, SaveVisibilityPolygonTest) {
  // Use a segment to represent the observers field of view.
  using Point2_t = geometry::Point2_t<float>;

  // Construct a closed trapezoid (smaller top, larger bottom).
  const test_shapes::Trapezoid<Point2_t> trap;
  const std::vector segs{trap.bot, trap.left, trap.top, trap.right};
  auto vis_poly = vis::raycast2d(segs, {0., 1.25}, 5.0, 0.033);

  std::vector<float> first, second;
  first.reserve(vis_poly.size());
  second.reserve(vis_poly.size());
  std::transform(vis_poly.begin(), vis_poly.end(), std::back_inserter(first),
                 [](const auto &e) { return e.first; });
  std::transform(vis_poly.begin(), vis_poly.end(), std::back_inserter(second),
                 [](const auto &e) { return e.second; });
  ASSERT_NO_FATAL_FAILURE(util::csv(first, "test.csv"));
  ASSERT_NO_FATAL_FAILURE(util::csv(second, "test.csv", true));
}
