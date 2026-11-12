// Physics micro benchmark: steps per second for a few representative scenes.
//   robotics_bench [--seconds N]    (rover, 6-link arm, 100-box pile, 50-hull piles)
// Prints one line per scene with the simulated steps per wall-clock second and a quality figure
// (peak joint anchor error or final pile height), so speed and accuracy can be compared together.

#include "robotics/demo_scenes.hpp"
#include "robotics/world.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

using namespace robotics;

namespace {

// Six motorized hinge links (alternating axes) on a static pedestal, leaning 0.5 rad so gravity loads
// every joint; speed motors at zero hold the pose.
World makeArm() {
    World world;
    Body base;
    base.name = "pedestal";
    base.mass = 0;
    base.size = {0.4, 0.4, 0.4};
    base.position = {0, 0.2, 0};
    world.addBody(base);
    const double lean = 0.5;
    const Vec3 direction{std::sin(lean), std::cos(lean), 0};
    Vec3 anchor{0, 0.4, 0};
    for (int i = 0; i < 6; ++i) {
        Body link;
        link.name = "link_" + std::to_string(i);
        link.mass = 3.0 - 0.4 * i;
        link.size = {0.12, 0.4, 0.12};
        link.position = anchor + direction * 0.2;
        link.orientation = Quat::axisAngle({0, 0, 1}, -lean);
        int index = world.addBody(link);
        Vec3 axis = i == 0 ? Vec3{0, 1, 0} : (i % 2 == 0 ? Vec3{1, 0, 0} : Vec3{0, 0, 1});
        Joint& joint = world.joints[static_cast<std::size_t>(
            world.connect(index - 1, index, JointType::Hinge, anchor, axis))];
        joint.maxTorque = 200;
        joint.motorSpeed = 0;
        anchor += direction * 0.4;
    }
    return world;
}

World makeBoxPile(int count) {
    World world;
    int side = 5;
    for (int i = 0; i < count; ++i) {
        Body box;
        box.size = {0.4, 0.4, 0.4};
        int layer = i / (side * side);
        int cell = i % (side * side);
        box.position = {(cell % side - 2) * 0.45 + 0.02 * layer, 0.25 + layer * 0.5,
                        (cell / side - 2) * 0.45};
        box.orientation = Quat::euler({0.05 * (i % 3), 0.1 * (i % 5), 0.03 * (i % 7)});
        world.addBody(box);
    }
    return world;
}

// Random convex rocks of 12-40 vertices dropped in a loose column: a settling hull pile.
World makeHullPile(int count, int minPoints, int maxPoints) {
    World world;
    std::mt19937 random(7);
    std::uniform_real_distribution<double> unit(-1, 1);
    int side = 4;
    for (int i = 0; i < count; ++i) {
        int points = minPoints + static_cast<int>(random() % static_cast<unsigned>(maxPoints - minPoints + 1));
        std::vector<Vec3> cloud;
        for (int k = 0; k < points; ++k) {
            Vec3 d{unit(random), unit(random), unit(random)};
            cloud.push_back(d.normalized() * 0.22 + Vec3{0.05 * unit(random), 0.05 * unit(random), 0});
        }
        auto hull = buildConvexHull(cloud);
        Body rock;
        rock.name = "rock_" + std::to_string(i);
        setConvexHull(rock, hull);
        rock.mass = 1000 * hull->volume;
        int layer = i / (side * side);
        int cell = i % (side * side);
        rock.position = {(cell % side - 1.5) * 0.55 + 0.03 * layer, 0.4 + layer * 0.55,
                         (cell / side - 1.5) * 0.55};
        rock.orientation = Quat::euler({0.7 * i, 0.3 * i, 0.1 * i});
        world.addBody(rock);
    }
    return world;
}

struct Result {
    double stepsPerSecond;
    double quality;
};

template <class Measure>
Result run(World world, int steps, Measure measure) {
    double quality = 0;
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        world.step();
        quality = measure(world, quality);
    }
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    return {steps / elapsed.count(), quality};
}

double peakAnchor(const World& world, double peak) {
    return std::max(peak, world.maxAnchorError());
}

} // namespace

int main(int argc, char** argv) {
    double seconds = 10;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--seconds") {
            seconds = std::atof(argv[i + 1]);
        }
    }
    int steps = static_cast<int>(seconds / World::defaultTimeStep);

    World rover = makeRoverDemo();
    driveRover(rover, 3, 0);
    Result r = run(rover, steps, peakAnchor);
    std::printf("rover        %9.0f steps/s  peak anchor error %.6f m\n", r.stepsPerSecond, r.quality);

    r = run(makeArm(), steps, peakAnchor);
    std::printf("6-link arm   %9.0f steps/s  peak anchor error %.6f m\n", r.stepsPerSecond, r.quality);

    r = run(makeBoxPile(100), steps, [](const World& world, double) {
        double top = 0;
        for (const auto& body : world.bodies) {
            top = std::max(top, body.position.y);
        }
        return top;
    });
    std::printf("100-box pile %9.0f steps/s  final top height %.4f m\n", r.stepsPerSecond, r.quality);

    auto topHeight = [](const World& world, double) {
        double top = 0;
        for (const auto& body : world.bodies) {
            top = std::max(top, body.position.y);
        }
        return top;
    };
    r = run(makeHullPile(50, 12, 40), steps, topHeight);
    std::printf("50-hull pile %9.0f steps/s  final top height %.4f m\n", r.stepsPerSecond, r.quality);
    r = run(makeHullPile(50, 64, 64), steps, topHeight);
    std::printf("50 big hulls %9.0f steps/s  final top height %.4f m\n", r.stepsPerSecond, r.quality);
    return 0;
}
