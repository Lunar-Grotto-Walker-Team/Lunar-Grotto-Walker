#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <Eigen/Geometry>
#include "go2_driver/map_odom_math.hpp"

static Eigen::Matrix4d trans(double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = x; T(1, 3) = y; T(2, 3) = z;
  return T;
}

int main() {
  // body 在 map 的 (1,2,0)，无安装偏移，odom 里的 base 在 (0,1,0):
  // map<-odom 应该是 (1,1,0)。
  Eigen::Matrix4d T = computeMapOdom(trans(1, 2, 0), Eigen::Matrix4d::Identity(),
                                     trans(0, 1, 0));
  assert((T.block<3, 1>(0, 3) - Eigen::Vector3d(1, 1, 0)).norm() < 1e-9);
  // 外参必须参与且方向正确：T_base_body = (0,0.5,0) 表示 body 原点在 base
  // 的 +y 0.5 m 处，故 T^body_base = (0,-0.5,0)；再叠加 odom base=(0,1,0)
  // => map<-odom 平移 (1, 0.5, 0)。
  Eigen::Matrix4d T2 = computeMapOdom(trans(1, 2, 0), trans(0, 0.5, 0),
                                      trans(0, 1, 0));
  assert((T2.block<3, 1>(0, 3) - Eigen::Vector3d(1, 0.5, 0)).norm() < 1e-9);

  // PGO freshness boundaries: exactly at the timeout is still fresh,
  // anything above it is stale.
  assert(isPgoFresh(0.0, 1.0));
  assert(isPgoFresh(0.999, 1.0));
  assert(isPgoFresh(1.0, 1.0));                       // fresh at exactly timeout
  assert(!isPgoFresh(1.0 + 1e-9, 1.0));               // stale just above
  assert(!isPgoFresh(std::numeric_limits<double>::infinity(), 1.0));

  printf("PASS\n");
  return 0;
}
