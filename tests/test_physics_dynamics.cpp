// Rotational dynamics (gyroscopic term), broad phase and determinism.
#include "test_harness.hpp"

#include "../src/physics/broadphase.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>

using namespace robotics;
using namespace test;

namespace {

// T-handle-like box with three distinct principal moments: Ixx < Iyy < Izz (Y is intermediate).
World spinningBox(Vec3 bodyRate) {
    World w;
    w.gravity = {};
    Body body;
    body.size = {1.0, 0.6, 0.2};
    body.mass = 2;
    body.position = {0, 5, 0};
    body.angularVelocity = bodyRate; // Identity orientation: body frame = world frame.
    w.addBody(body);
    return w;
}

Vec3 bodyRate(const Body& body) {
    return body.orientation.conjugate().rotate(body.angularVelocity);
}

double kineticEnergy(const Body& body) {
    Vec3 w = bodyRate(body);
    Vec3 inertia = body.principalInertia();
    return 0.5 * (body.mass * body.velocity.length2() + w.dot(componentProduct(inertia, w)));
}

Vec3 angularMomentum(const Body& body) {
    return body.orientation.rotate(componentProduct(body.principalInertia(), bodyRate(body)));
}

// Deterministic pseudo-random numbers (no <random> distributions: their output is implementation-defined).
struct Lcg {
    std::uint64_t state;

    double next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(state >> 11) / static_cast<double>(1ULL << 53);
    }

    double range(double low, double high) { return low + (high - low) * next(); }
};

World randomScene(std::uint64_t seed, int count, double extent) {
    Lcg rng{seed};
    World w;
    for (int i = 0; i < count; ++i) {
        Body body;
        int shape = static_cast<int>(rng.next() * 3);
        body.shape = shape == 0 ? Shape::Box : (shape == 1 ? Shape::Sphere : Shape::Cylinder);
        double s = rng.range(0.1, 0.8);
        body.size = body.shape == Shape::Box
                        ? Vec3{s, rng.range(0.1, 0.8), rng.range(0.1, 0.8)}
                        : (body.shape == Shape::Sphere ? Vec3{s, s, s} : Vec3{s, s, rng.range(0.1, 0.8)});
        body.position = {rng.range(-extent, extent), rng.range(0.2, 2 * extent), rng.range(-extent, extent)};
        body.orientation = Quat::euler({rng.range(-pi, pi), rng.range(-pi, pi), rng.range(-pi, pi)});
        body.mass = rng.next() < 0.15 ? 0 : rng.range(0.2, 3);
        w.addBody(body);
    }
    for (int i = 0; i + 1 < count; i += 7) {
        w.connect(i, i + 1, JointType::Ball, w.bodies[static_cast<std::size_t>(i)].position);
    }
    return w;
}

} // namespace

TEST_CASE("Dzhanibekov effect: spin near the intermediate axis flips") {
    World w = spinningBox({0.01, 6, 0.01});
    bool flipped = false;
    for (int i = 0; i < 10 * 240 && !flipped; ++i) {
        w.step();
        flipped = bodyRate(w.bodies[0]).y < -5;
    }
    check(flipped, "Intermediate-axis spin did not flip");
}

TEST_CASE("spin about the major and minor axes stays stable") {
    for (int axis : {0, 2}) {
        Vec3 rate{0.01, 0.01, 0.01};
        rate[axis] = 6;
        World w = spinningBox(rate);
        double lowest = 1;
        for (int i = 0; i < 10 * 240; ++i) {
            w.step();
            Vec3 r = bodyRate(w.bodies[0]);
            lowest = std::min(lowest, std::abs(r[axis]) / r.length());
        }
        check(lowest > 0.99, axis == 0 ? "Minor-axis spin wandered" : "Major-axis spin wandered");
    }
}

TEST_CASE("free tumbling conserves kinetic energy and angular momentum") {
    World w = spinningBox({2, 3, -1.5});
    double energy0 = kineticEnergy(w.bodies[0]);
    Vec3 momentum0 = angularMomentum(w.bodies[0]);
    double worstEnergy = 0;
    double worstMomentum = 0;
    for (int i = 0; i < 10 * 240; ++i) {
        w.step();
        worstEnergy = std::max(worstEnergy, std::abs(kineticEnergy(w.bodies[0]) / energy0 - 1));
        worstMomentum =
            std::max(worstMomentum, (angularMomentum(w.bodies[0]) - momentum0).length() / momentum0.length());
    }
    check(worstEnergy < 0.01, ("Kinetic energy drift " + std::to_string(worstEnergy)).c_str());
    check(worstMomentum < 0.01, ("Angular momentum drift " + std::to_string(worstMomentum)).c_str());
}

TEST_CASE("broad phase finds exactly the brute-force pairs") {
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        World w = randomScene(seed, 60, 2.0 + static_cast<double>(seed % 4));
        auto fast = detail::broadphasePairs(w.bodies, w.joints);
        auto slow = detail::bruteForcePairs(w.bodies, w.joints);
        check(fast == slow, "Sweep and prune differs from brute force");
        check(!slow.empty(), "Random scene has no overlapping pairs (test is vacuous)");
    }
}

TEST_CASE("two hundred bodies simulate quickly and stay sane") {
    World w = randomScene(42, 200, 4.0);
    for (auto& body : w.bodies) {
        body.position.y += 0.5;
    }
    auto start = std::chrono::steady_clock::now();
    const int steps = 2 * 240;
    for (int i = 0; i < steps; ++i) {
        w.step();
    }
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    std::cout << "    200 bodies: " << static_cast<int>(steps / elapsed.count()) << " steps/s, "
              << w.contacts.size() << " contacts\n";
    w.validate();
    for (const auto& body : w.bodies) {
        check(body.velocity.length() < 50, "200-body scene exploded");
    }
}

TEST_CASE("simulation is bit-identical run to run") {
    auto run = [] {
        World w = randomScene(7, 40, 1.5);
        World rover = makeRoverDemo();
        driveRover(rover, 3, 1);
        for (int i = 0; i < 480; ++i) {
            w.step();
            rover.step();
        }
        std::vector<double> state;
        for (const World* world : {&w, &rover}) {
            for (const auto& b : world->bodies) {
                for (double v : {b.position.x, b.position.y, b.position.z, b.orientation.w, b.orientation.x,
                                 b.orientation.y, b.orientation.z, b.velocity.x, b.angularVelocity.y}) {
                    state.push_back(v);
                }
            }
        }
        return state;
    };
    auto first = run();
    auto second = run();
    check(first.size() == second.size()
              && std::memcmp(first.data(), second.data(), first.size() * sizeof(double)) == 0,
          "Two identical runs differ");
}
