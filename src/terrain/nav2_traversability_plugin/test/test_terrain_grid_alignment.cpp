// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "nav2_traversability_plugin/terrain_grid_alignment.hpp"

using nav2_traversability_plugin::GridAlignment;

namespace
{

TEST(GridAlignment, IdentityMapsWorldToCell)
{
  // Grid origin at (10, 20), 0.1 m cells, no rotation.
  GridAlignment a;
  a.setFromTransform(10.0, 20.0, 0.0, 0.1);

  int mx = 0, my = 0;
  EXPECT_TRUE(a.worldToGrid(10.0, 20.0, mx, my));
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 0);

  EXPECT_TRUE(a.worldToGrid(10.25, 20.15, mx, my));
  EXPECT_EQ(mx, 2);
  EXPECT_EQ(my, 1);

  // Just outside the origin -> negative cell.
  EXPECT_TRUE(a.worldToGrid(9.95, 20.0, mx, my));
  EXPECT_EQ(mx, -1);
  EXPECT_EQ(my, 0);
}

TEST(GridAlignment, RotatedGridMapsWorldToCell)
{
  // Grid rotated by +90 deg around its origin (0,0): grid +x axis points to
  // world +y. A point 0.3 m along the grid's x axis is 0.3 m along world +y.
  GridAlignment a;
  a.setFromTransform(0.0, 0.0, M_PI / 2.0, 0.1);

  int mx = 0, my = 0;
  // world = R(+90deg) * grid  =>  grid = R(-90deg) * world
  EXPECT_TRUE(a.worldToGrid(0.0, 0.3, mx, my));
  EXPECT_EQ(mx, 3);
  EXPECT_EQ(my, 0);

  EXPECT_TRUE(a.worldToGrid(-0.2, 0.0, mx, my));
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 2);
}

TEST(GridAlignment, GridBoundsCoverTheRotatedRectangle)
{
  // 10 x 20 cells of 0.1 m, origin at (0,0), rotated 90 deg -> the rectangle
  // spans world x in [-2, 0], y in [0, 1].
  GridAlignment a;
  a.setFromTransform(0.0, 0.0, M_PI / 2.0, 0.1);

  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  a.gridBounds(10, 20, min_x, min_y, max_x, max_y);

  EXPECT_NEAR(min_x, -2.0, 1e-9);
  EXPECT_NEAR(max_x, 0.0, 1e-9);
  EXPECT_NEAR(min_y, 0.0, 1e-9);
  EXPECT_NEAR(max_y, 1.0, 1e-9);
}

TEST(GridAlignment, GridBoundsOfAxisAlignedGrid)
{
  GridAlignment a;
  a.setFromTransform(-25.0, -25.0, 0.0, 0.05);

  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  a.gridBounds(1000, 1000, min_x, min_y, max_x, max_y);

  EXPECT_NEAR(min_x, -25.0, 1e-9);
  EXPECT_NEAR(min_y, -25.0, 1e-9);
  EXPECT_NEAR(max_x, 25.0, 1e-9);
  EXPECT_NEAR(max_y, 25.0, 1e-9);
}

TEST(GridAlignment, NonPositiveResolutionIsRejected)
{
  GridAlignment a;
  EXPECT_FALSE(a.setFromTransform(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.setFromTransform(0.0, 0.0, 0.0, -0.1));
}

}  // namespace
