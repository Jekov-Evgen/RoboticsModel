#include "test_harness.hpp"

using namespace robotics;
using namespace test;

TEST_CASE("3D quaternion rotation") {
    Vec3 p = Quat::axisAngle({0, 1, 0}, pi / 2).rotate({1, 0, 0});
    near(p.z, -1, 1e-10, "Rotation around Y");
    near(p.x, 0, 1e-10, "Rotation X");
    Quat q = Quat::euler({0.2, -0.3, 0.7});
    near((q.conjugate().rotate(q.rotate({1, 2, 3})) - Vec3{1, 2, 3}).length(), 0, 1e-10, "Inverse rotation");
}
