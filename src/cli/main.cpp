// Command-line runner: loads a scene or demo, simulates it and optionally exports CSV traces.

#include "robotics/demo_scenes.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/io/urdf.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double maxSeconds = 3600;
constexpr int stepsPerSecond = 240;   // Matches World::defaultTimeStep.
constexpr int stepsPerPoseSample = 4; // Pose CSV at 60 Hz.

struct Options {
    std::string input;
    std::string urdf;
    bool fixBase{};
    std::string output;
    std::string csv;
    std::string controlCsv;
    std::string sensorCsv;
    double seconds{5};
    bool controlDemo{};
    bool roverDemo{};
    bool actuatorDemo{};
    bool sensorRoverDemo{};
    bool noisyPidDemo{};
};

void printHelp() {
    std::cout << "Robotics Model: own C++ 3D physics\n"
              << "  --scene PATH       Load a .robot scene (default: manipulator demo)\n"
              << "  --urdf PATH        Import a URDF robot (warnings go to stderr)\n"
              << "  --fix-base         With --urdf: make the root link static (arms bolted down)\n"
              << "  --seconds N        Simulated seconds, 0..3600 (default: 5)\n"
              << "  --save PATH        Save initial scene before simulation\n"
              << "  --csv PATH         Export body poses at 60 Hz\n"
              << "  --control-demo     Use the PID step-response demo\n"
              << "  --rover-demo       Use the four-wheel rover (drives forward at 3 rad/s)\n"
              << "  --actuator-demo    Use the linear actuator (slider PID + limited hinge flap)\n"
              << "  --sensor-rover-demo Use the rover with lidar, IMU, camera and wheel encoders (drives)\n"
              << "  --noisy-pid-demo   Use the PID bench fed by a noisy, quantized, delayed encoder\n"
              << "  --control-csv PATH Export control samples of all hinge/slider joints at 240 Hz\n"
              << "  --sensor-csv PATH  Export every sensor sample (long format; camera images are skipped)\n";
}

// Returns false when the program should exit successfully right away (--help).
bool parseArguments(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printHelp();
            return false;
        }
        if (arg == "--control-demo") {
            options.controlDemo = true;
            continue;
        }
        if (arg == "--rover-demo") {
            options.roverDemo = true;
            continue;
        }
        if (arg == "--actuator-demo") {
            options.actuatorDemo = true;
            continue;
        }
        if (arg == "--sensor-rover-demo") {
            options.sensorRoverDemo = true;
            continue;
        }
        if (arg == "--noisy-pid-demo") {
            options.noisyPidDemo = true;
            continue;
        }
        if (arg == "--fix-base") {
            options.fixBase = true;
            continue;
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value for " + arg);
        }
        std::string value = argv[++i];
        if (arg == "--scene") {
            options.input = value;
        } else if (arg == "--urdf") {
            options.urdf = value;
        } else if (arg == "--save") {
            options.output = value;
        } else if (arg == "--csv") {
            options.csv = value;
        } else if (arg == "--control-csv") {
            options.controlCsv = value;
        } else if (arg == "--sensor-csv") {
            options.sensorCsv = value;
        } else if (arg == "--seconds") {
            std::size_t used = 0;
            options.seconds = std::stod(value, &used);
            if (used != value.size()) {
                throw std::runtime_error("Invalid seconds value");
            }
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (!std::isfinite(options.seconds) || options.seconds < 0 || options.seconds > maxSeconds) {
        throw std::runtime_error("Seconds must be in [0,3600]");
    }
    int sources = static_cast<int>(options.controlDemo) + static_cast<int>(options.roverDemo)
                  + static_cast<int>(options.actuatorDemo) + static_cast<int>(options.sensorRoverDemo)
                  + static_cast<int>(options.noisyPidDemo) + static_cast<int>(!options.input.empty())
                  + static_cast<int>(!options.urdf.empty());
    if (sources > 1) {
        throw std::runtime_error("Choose one scene or demo");
    }
    if (options.fixBase && options.urdf.empty()) {
        throw std::runtime_error("--fix-base only applies to --urdf");
    }
    return true;
}

robotics::World makeWorld(const Options& options) {
    if (!options.input.empty()) {
        return robotics::loadScene(options.input);
    }
    if (!options.urdf.empty()) {
        robotics::UrdfImportOptions importOptions;
        importOptions.fixBase = options.fixBase;
        auto imported = robotics::loadUrdf(options.urdf, importOptions);
        for (const auto& warning : imported.warnings) {
            std::cerr << "Warning: " << warning << '\n';
        }
        return std::move(imported.world);
    }
    if (options.roverDemo) {
        return robotics::makeRoverDemo();
    }
    if (options.actuatorDemo) {
        return robotics::makeLinearActuatorDemo();
    }
    if (options.sensorRoverDemo) {
        return robotics::makeSensorRoverDemo();
    }
    if (options.noisyPidDemo) {
        return robotics::makeNoisyPidDemo();
    }
    return options.controlDemo ? robotics::makeControlDemo() : robotics::makeDemo();
}

void writePoses(std::ostream& out, const robotics::World& world) {
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        const auto& body = world.bodies[i];
        out << world.time << ',' << i << ',' << body.position.x << ',' << body.position.y << ','
            << body.position.z << ',' << body.orientation.w << ',' << body.orientation.x << ','
            << body.orientation.y << ',' << body.orientation.z << '\n';
    }
}

int run(const Options& options) {
    auto world = makeWorld(options);
    if (options.roverDemo || options.sensorRoverDemo) {
        robotics::driveRover(world, 3, 0);
    }
    if (!options.output.empty()) {
        robotics::saveScene(world, options.output);
    }

    std::ofstream trace;
    std::ofstream controlTrace;
    std::ofstream sensorTrace;
    if (!options.sensorCsv.empty()) {
        if (options.sensorCsv == options.csv || options.sensorCsv == options.controlCsv
            || options.sensorCsv == options.input || options.sensorCsv == options.urdf
            || options.sensorCsv == options.output) {
            throw std::runtime_error("Sensor CSV needs a distinct output path");
        }
        sensorTrace.open(options.sensorCsv);
        if (!sensorTrace) {
            throw std::runtime_error("Cannot open sensor CSV output");
        }
        robotics::writeSensorCsvHeader(sensorTrace);
    }
    if (!options.controlCsv.empty()) {
        if (options.controlCsv == options.csv || options.controlCsv == options.input
            || options.controlCsv == options.urdf || options.controlCsv == options.output) {
            throw std::runtime_error("Control CSV needs a distinct output path");
        }
        controlTrace.open(options.controlCsv);
        if (!controlTrace) {
            throw std::runtime_error("Cannot open control CSV output");
        }
        robotics::writeControlCsvHeader(controlTrace);
    }
    if (!options.csv.empty()) {
        trace.open(options.csv);
        if (!trace) {
            throw std::runtime_error("Cannot open CSV output");
        }
        trace << "time,body_id,x,y,z,qw,qx,qy,qz\n" << std::setprecision(12);
    }
    auto record = [&] {
        if (trace.is_open()) {
            writePoses(trace, world);
        }
    };

    record();
    int steps = static_cast<int>(std::llround(options.seconds * stepsPerSecond));
    double peakError = 0;
    for (int i = 0; i < steps; ++i) {
        robotics::stepSimulation(world); // Physics step + sensor update (sensor feedback for controllers).
        if (sensorTrace.is_open()) {
            robotics::writeSensorCsvRows(sensorTrace, world);
        }
        peakError = std::max(peakError, world.maxAnchorError());
        if (controlTrace.is_open()) {
            for (std::size_t j = 0; j < world.joints.size(); ++j) {
                if (!world.joints[j].history.empty()) {
                    robotics::writeControlCsvRow(controlTrace, static_cast<int>(j),
                                                 world.joints[j].history.back());
                }
            }
        }
        if ((i + 1) % stepsPerPoseSample == 0) {
            record();
        }
    }
    world.validate();

    if (trace.is_open()) {
        trace.close();
        if (!trace) {
            throw std::runtime_error("Failed to write CSV");
        }
    }
    if (controlTrace.is_open()) {
        controlTrace.close();
        if (!controlTrace) {
            throw std::runtime_error("Failed to write control CSV");
        }
    }
    if (sensorTrace.is_open()) {
        sensorTrace.close();
        if (!sensorTrace) {
            throw std::runtime_error("Failed to write sensor CSV");
        }
    }
    std::cout << std::fixed << std::setprecision(6) << "Simulated " << world.time
              << " s; bodies=" << world.bodies.size() << "; joints=" << world.joints.size()
              << "; sensors=" << world.sensors.size() << "; peak anchor error=" << peakError << " m\n";
    for (const auto& body : world.bodies) {
        std::cout << body.name << ": (" << body.position.x << ", " << body.position.y << ", "
                  << body.position.z << ")\n";
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        Options options;
        if (!parseArguments(argc, argv, options)) {
            return 0;
        }
        return run(options);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
