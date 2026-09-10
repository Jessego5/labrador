// These are the tests for the insertion helpers. They check that the peck search starts at the
// centre and works outwards, that it covers the whole search disc closely enough for a square
// hole's capture window, that it reaches the corners the part 5 acceptance offsets need, and that
// the progress monitor calls a stall only after a full window without progress. Run them with
// colcon test --packages-select cell_skills.
#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cell_skills/insertion.hpp"

using cell_skills::peckSearchPattern;
using cell_skills::ProgressMonitor;

TEST(PeckSearch, StartsAtCentreNearestFirst)
{
  const auto p = peckSearchPattern(0.0015, 0.006);
  ASSERT_FALSE(p.empty());
  EXPECT_NEAR(p.front().norm(), 0.0, 1e-12);
  for (std::size_t i = 1; i < p.size(); ++i) {
    EXPECT_LE(p[i - 1].norm(), p[i].norm() + 1e-12);
    EXPECT_LE(p[i].norm(), 0.006 + 1e-12);
  }
  // the first ring is the four axis neighbours
  for (int i = 1; i <= 4; ++i) {
    EXPECT_NEAR(p[static_cast<std::size_t>(i)].norm(), 0.0015, 1e-12);
  }
}

TEST(PeckSearch, CoversTheSearchAreaPerAxis)
{
  // Every offset well inside the disc has a probe within spacing/2 in both x and y, which is the
  // capture window of a square hole, so no hole position can be skipped.
  const double s = 0.0015, r = 0.006;
  const auto p = peckSearchPattern(s, r);
  for (double x = -(r - s); x <= r - s; x += 0.0002) {
    for (double y = -(r - s); y <= r - s; y += 0.0002) {
      if (std::hypot(x, y) > r - s) {
        continue;
      }
      const bool covered = std::any_of(p.begin(), p.end(), [&](const Eigen::Vector2d & q) {
            return std::abs(q.x() - x) <= s / 2 + 1e-9 && std::abs(q.y() - y) <= s / 2 + 1e-9;
          });
      EXPECT_TRUE(covered) << x << " " << y;
    }
  }
}

TEST(PeckSearch, CoversThreeMillimetreCorners)
{
  // The part 5 acceptance run moves the fixture by up to +-3 mm per axis, and the grasp adds about
  // 1.5 mm of offset on top of that.
  const auto p = peckSearchPattern(0.0015, 0.006);
  for (double x : {-0.0045, 0.0045}) {
    for (double y : {-0.003, 0.003}) {
      EXPECT_TRUE(std::any_of(p.begin(), p.end(), [&](const Eigen::Vector2d & q) {
          return std::abs(q.x() - x) <= 0.00075 + 1e-9 && std::abs(q.y() - y) <= 0.00075 + 1e-9;
        }));
    }
  }
}

TEST(PeckSearch, RejectsBadArguments)
{
  EXPECT_THROW(peckSearchPattern(0.0, 0.005), std::invalid_argument);
  EXPECT_EQ(peckSearchPattern(0.002, 0.0).size(), 1u);
}

TEST(ProgressMonitor, StallsWithoutProgress)
{
  ProgressMonitor m(0.0005, 1.0);
  m.reset(0.0, 0.0);
  EXPECT_FALSE(m.update(0.5, 0.0002));
  EXPECT_FALSE(m.update(0.9, 0.0006));   // progress, so the timer restarts at 0.9
  EXPECT_FALSE(m.update(1.8, 0.0008));
  EXPECT_TRUE(m.update(2.0, 0.0008));
  EXPECT_NEAR(m.best(), 0.0006, 1e-12);
}
