// EditorController (SmokeTest): end-to-end self check run by `robotics_editor --smoke-test`.
// Drives the real UI objects, then renders build/editor-preview.png and build/control-lab-preview.png.

#import "mac/editor_controller.h"
#import "mac/control_lab.h"
#import "mac/sensor_panel.h"

#include "robotics/demo_scenes.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace robotics;
using namespace editor_ui;

@implementation EditorController (SmokeTest)

- (NSBitmapImageRep*)windowImage {
    NSView* content = _window.contentView;
    [content display];
    NSBitmapImageRep* ui = [content bitmapImageRepForCachingDisplayInRect:content.bounds];
    [_window.effectiveAppearance performAsCurrentDrawingAppearance:^{
        [content cacheDisplayInRect:content.bounds toBitmapImageRep:ui];
    }];
    NSBitmapImageRep* scene = [_scene snapshot];
    NSBitmapImageRep* out = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:nullptr
                                                                    pixelsWide:ui.pixelsWide
                                                                    pixelsHigh:ui.pixelsHigh
                                                                 bitsPerSample:8
                                                               samplesPerPixel:4
                                                                      hasAlpha:YES
                                                                      isPlanar:NO
                                                                colorSpaceName:NSDeviceRGBColorSpace
                                                                   bytesPerRow:0
                                                                  bitsPerPixel:0];
    out.size = content.bounds.size;
    [NSGraphicsContext saveGraphicsState];
    [NSGraphicsContext setCurrentContext:[NSGraphicsContext graphicsContextWithBitmapImageRep:out]];
    [[NSColor colorWithCalibratedRed:0.1 green:0.11 blue:0.13 alpha:1] setFill];
    NSRectFill(content.bounds);
    [ui drawInRect:content.bounds];
    [scene drawInRect:[_scene convertRect:_scene.bounds toView:content]];
    // Viewport overlays are regular views drawn over the Metal layer.
    for (NSView* overlay in _scene.subviews) {
        if (!overlay.hidden) {
            NSBitmapImageRep* rep = [overlay bitmapImageRepForCachingDisplayInRect:overlay.bounds];
            [_window.effectiveAppearance performAsCurrentDrawingAppearance:^{
                [overlay cacheDisplayInRect:overlay.bounds toBitmapImageRep:rep];
            }];
            [rep drawInRect:[overlay convertRect:overlay.bounds toView:content]
                      fromRect:NSZeroRect
                     operation:NSCompositingOperationSourceOver
                      fraction:1
                respectFlipped:YES
                         hints:nil];
        }
    }
    [NSGraphicsContext restoreGraphicsState];
    return out;
}

- (BOOL)writeLabImage:(NSString*)path {
    NSView* content = _lab->_window.contentView;
    [content display];
    NSBitmapImageRep* bitmap = [content bitmapImageRepForCachingDisplayInRect:content.bounds];
    [_lab->_window.effectiveAppearance performAsCurrentDrawingAppearance:^{
        [content cacheDisplayInRect:content.bounds toBitmapImageRep:bitmap];
    }];
    NSData* png = [bitmap representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    return [png writeToFile:path atomically:YES];
}

// Creates a slider with limits through the inspector, then turns it into a limited hinge.
- (void)checkSliderInspector {
    auto check = [](bool ok, const char* message) {
        if (!ok) {
            throw std::runtime_error(message);
        }
    };
    [self addBox:nil];
    int part = _selected;
    setVector(_position, {3, 1.2, 0});
    [self applyBody:nil];
    [_parentList selectItemWithTag:0];
    [self chooseParent:_parentList];
    [_jointType setSelected:YES forSegment:static_cast<int>(JointType::Prismatic)];
    [self jointTypeChanged:_jointType];
    check([_speedLabel.stringValue isEqualToString:@"Motor speed (m/s)"] &&
              [_torqueLabel.stringValue isEqualToString:@"Max force (N)"] && _axis[0].enabled
              && _limitsEnabled.enabled && !_lowerLimit.enabled,
          "Slider inspector labels or enabled state wrong");
    setVector(_anchor, {3, 1.2, 0});
    setVector(_axis, {1, 0, 0});
    _speed.stringValue = @"1";
    _torque.stringValue = @"30";
    _limitsEnabled.state = NSControlStateValueOn;
    [self toggleLimits:_limitsEnabled];
    check([_lowerLimit.stringValue isEqualToString:@"0"] && [_upperLimit.stringValue isEqualToString:@"0.5"]
              && [_lowerLabel.stringValue isEqualToString:@"Min (m)"] && _lowerLimit.enabled,
          "Slider limit defaults wrong");
    _upperLimit.stringValue = @"0.25";
    [self applyJoint:nil];
    const Joint& slider = _world.joints.back();
    check(slider.type == JointType::Prismatic && slider.b == part && slider.limitsEnabled
              && slider.lowerLimit == 0 && slider.upperLimit == 0.25 && slider.maxTorque == 30,
          "Slider creation through the inspector failed");
    check([jointName(slider.type).lowercaseString isEqualToString:@"slider"], "Parts list joint label");
    [self toggleRun:nil];
    for (int i = 0; i < 480; ++i) {
        _world.step();
    }
    [self toggleRun:nil];
    check(std::abs(_world.joints.back().angle - 0.25) < 0.01, "Slider limit did not stop the motor");
    [self reset:nil];
    [self selectBody:part];
    check(_jointType.selectedSegment == static_cast<int>(JointType::Prismatic)
              && _limitsEnabled.state == NSControlStateValueOn &&
              [_upperLimit.stringValue isEqualToString:@"0.25"],
          "Inspector did not reload the slider");
    // Switching to a hinge resets the range to degrees.
    [_jointType setSelected:YES forSegment:static_cast<int>(JointType::Hinge)];
    [self jointTypeChanged:_jointType];
    check([_lowerLabel.stringValue isEqualToString:@"Min (°)"] &&
              [_lowerLimit.stringValue isEqualToString:@"-90"] &&
              [_torqueLabel.stringValue isEqualToString:@"Max torque (N·m)"],
          "Hinge limit units wrong");
    [self applyJoint:nil];
    check(_world.joints.back().type == JointType::Hinge
              && std::abs(_world.joints.back().lowerLimit + robotics::pi / 2) < 1e-12,
          "Hinge limits not converted to radians");
    // Inverted range is rejected and leaves the world unchanged.
    _lowerLimit.stringValue = @"10";
    _upperLimit.stringValue = @"-10";
    bool rejected = false;
    try {
        [self applyJoint:nil];
    } catch (const std::exception&) {
        rejected = true;
    }
    check(rejected && std::abs(_world.joints.back().upperLimit - robotics::pi / 2) < 1e-12,
          "Inverted limits were not rejected");
    [_jointType setSelected:YES forSegment:static_cast<int>(JointType::Fixed)];
    [self jointTypeChanged:_jointType];
    check(!_limitsEnabled.enabled && !_speed.enabled, "Limits must be disabled for fixed joints");
    [self deleteBody:nil];
}

// Loads the Linear Actuator demo, runs it and renders build/linear-actuator-preview.png and the
// slider's Control Lab plots (build/linear-actuator-lab-preview.png).
- (void)checkLinearActuatorDemo {
    auto check = [](bool ok, const char* message) {
        if (!ok) {
            throw std::runtime_error(message);
        }
    };
    [self actuatorDemo:nil];
    check(_world.joints.size() == 2 && _world.joints[0].type == JointType::Prismatic, "Actuator demo failed");
    for (int i = 0; i < 300; ++i) {
        _world.step();
    }
    [self refresh];
    [_scene homeCamera];
    NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:@"build/linear-actuator-preview.png" atomically:YES],
          "Could not write linear actuator preview");
    [self openControlLab:nil];
    check(_lab->_joint.numberOfItems == 2, "Control Lab must list the slider and the hinge");
    [_lab->_joint selectItemWithTag:0];
    [_lab loadFields:nil];
    check([_lab->_units.stringValue hasSuffix:@"in m"] &&
              [_lab->_limitLabel.stringValue isEqualToString:@"Max force (N)"],
          "Control Lab units for a slider");
    check([self writeLabImage:@"build/linear-actuator-lab-preview.png"], "Could not render slider plots");
    [_lab->_window orderOut:nil];

    // Re-applying an unchanged connection of a moved slider keeps its zero, so limits stay physical.
    [self selectBody:2];
    double before = _world.joints[0].angle;
    check(before > 0.3, "Actuator demo carriage did not move");
    [self applyJoint:nil];
    const Joint& kept = _world.joints[0];
    check(kept.type == JointType::Prismatic && kept.b == 2 && std::abs(kept.angle - before) < 1e-12
              && kept.lowerLimit == 0 && kept.upperLimit == 0.5 && kept.controller.kp == 60,
          "Re-applying an unchanged slider re-zeroed it");
    // Moving the anchor re-zeroes the joint; limits and the position target shift to stay in place.
    setVector(_anchor, {0.2, 1.66, 0});
    [self applyJoint:nil];
    const Joint& moved = _world.joints.back();
    check(moved.type == JointType::Prismatic && moved.b == 2 && std::abs(moved.lowerLimit + before) < 1e-9
              && std::abs(moved.upperLimit - (0.5 - before)) < 1e-9
              && std::abs(moved.controller.target + before) < 1e-9,
          "Re-zeroed slider limits or target were not shifted");
    double peak = 0;
    for (int i = 0; i < 600; ++i) {
        _world.step();
        peak = std::max(peak, _world.bodies[2].position.x);
    }
    check(peak < 0.51 && std::abs(_world.bodies[2].position.x - 0.4) < 0.03,
          "Re-zeroed slider left its physical range");
    // Hinge -> Slider with "Limit range" unticked must not keep degree values for metres.
    [self selectBody:3];
    check(_limitsEnabled.state == NSControlStateValueOn && [_lowerLimit.stringValue isEqualToString:@"-35"],
          "Flap limits not shown in degrees");
    _limitsEnabled.state = NSControlStateValueOff;
    [self toggleLimits:_limitsEnabled];
    [_jointType setSelected:YES forSegment:static_cast<int>(JointType::Prismatic)];
    [self jointTypeChanged:_jointType];
    _limitsEnabled.state = NSControlStateValueOn;
    [self toggleLimits:_limitsEnabled];
    check([_lowerLimit.stringValue isEqualToString:@"0"] && [_upperLimit.stringValue isEqualToString:@"0.5"],
          "Hinge degree limits leaked into slider metres");
}

// Sensors: inspector add/edit/remove with undo, live readings, camera rendering, lidar, CSV, persistence,
// Control Lab feedback source and the sensor examples.
- (void)checkSensors {
    auto check = [](bool ok, const char* message) {
        if (!ok) {
            throw std::runtime_error(message);
        }
    };
    auto addItem = [&](SensorType type) {
        for (NSMenuItem* item in _sensorAdd.itemArray) {
            if (item.action == @selector(addSensor:) && item.tag == static_cast<NSInteger>(type)) {
                return item;
            }
        }
        throw std::runtime_error("Add sensor menu item missing");
    };
    [self sensorRover:nil];
    check(_world.sensors.size() == 7 && _selected == 6 && !_sensorCard.hidden, "Sensor Rover did not load");
    check(_sensorList.numberOfItems == 2, "Inspector lists the sensor head's lidar and camera");
    check(!addItem(SensorType::JointEncoder).enabled && addItem(SensorType::Imu).enabled,
          "Joint sensors must be disabled on a part without a hinge");
    [self addSensor:addItem(SensorType::Imu)];
    check(_world.sensors.size() == 8 && _shownSensor == 7, "Add IMU failed");
    _sensorFields[0].stringValue = @"50";
    _sensorFields[2].stringValue = @"0.2";
    setVector(_sensorMount, {0, 0.2, 0});
    [self applySensor:nil];
    const Sensor& imu = _world.sensors[7];
    check(imu.rate == 50 && imu.noise.stddev == 0.2 && imu.position.y == 0.2, "Sensor edit did not apply");
    _sensorFields[6].stringValue = @"150";
    bool rejected = false;
    try {
        [self applySensor:nil];
    } catch (const std::exception&) {
        rejected = true;
    }
    check(rejected && _world.sensors[7].dropout == 0, "Invalid dropout accepted");
    [self refresh];
    [_undo undo];
    check(_world.sensors[7].rate == 200, "Undo sensor edit failed");
    [_undo undo];
    check(_world.sensors.size() == 7, "Undo add sensor failed");
    [_undo redo];
    check(_world.sensors.size() == 8, "Redo add sensor failed");
    [self selectBody:6];
    [_sensorList selectItemWithTag:7];
    [self chooseSensor:_sensorList];
    [self removeSensor:nil];
    check(_world.sensors.size() == 7, "Remove sensor failed");
    [self selectBody:1];
    [self addSensor:addItem(SensorType::JointVelocity)];
    check(sensorJoint(_world, _world.sensors.back()) >= 0, "Joint velocity sensor has no joint");

    // Drive with sensors updating exactly like the run loop.
    check([self startSensorLog:@"build/sensor-log.csv"], "Cannot start sensor log");
    [self drive:@"w"];
    for (int i = 0; i < 480; ++i) {
        stepSimulation(_world);
        [self afterSimulationStep];
    }
    [self updateSensorViews];
    [self stopSensorLog];
    const Sensor& lidar = _world.sensors[0];
    int hits = 0;
    for (char hit : lidar.state.rayHits) {
        hits += hit;
    }
    check(lidar.type == SensorType::Lidar2D && lidar.state.valid && hits > 10, "Lidar saw no obstacles");
    check(_world.sensors[1].state.valid && std::abs(_world.sensors[1].state.value[1] - 9.81) < 3,
          "IMU reading implausible");
    check(_cameraSensor == 2 && _cameraColor.size() == 160u * 120u * 4u && _cameraDepth.size() == 160u * 120u,
          "Camera did not render");
    float nearest = 1e9f;
    float farthest = 0;
    for (float d : _cameraDepth) {
        check(std::isfinite(d) && d > 0, "Camera depth not metric");
        nearest = std::min(nearest, d);
        farthest = std::max(farthest, d);
    }
    check(nearest < 10 && farthest > nearest + 0.5, "Camera depth has no structure");
    long brightness = 0;
    for (std::size_t i = 0; i < _cameraColor.size(); i += 4) {
        brightness += _cameraColor[i] + _cameraColor[i + 1] + _cameraColor[i + 2];
    }
    check(brightness > 0, "Camera image is black");
    std::ifstream log("build/sensor-log.csv");
    std::string line;
    int rows = 0;
    while (std::getline(log, line)) {
        ++rows;
    }
    check(rows > 1000, "Sensor CSV log too short");
    [self openSensors:nil];
    [_sensorPanel->_sensor selectItemWithTag:0];
    [_sensorPanel tick];
    check([_sensorPanel exportHistory:@"build/sensor-history.csv"], "Sensor history export failed");
    NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:@"build/sensors-preview.png" atomically:YES], "Could not write sensors preview");
    NSView* content = _sensorPanel->_window.contentView;
    [content display];
    NSBitmapImageRep* panelImage = [content bitmapImageRepForCachingDisplayInRect:content.bounds];
    [content cacheDisplayInRect:content.bounds toBitmapImageRep:panelImage];
    [[panelImage representationUsingType:NSBitmapImageFileTypePNG
                              properties:@{}] writeToFile:@"build/sensors-window-preview.png" atomically:YES];
    [self toggleRun:nil];
    saveScene(_world, "build/sensor-rover.robot");
    World loaded = loadScene("build/sensor-rover.robot");
    check(loaded.sensors.size() == _world.sensors.size() && loaded.sensors[0].beams == lidar.beams,
          "Sensor save/load failed");
    [self reset:nil];
    check(_world.sensors[0].state.samples == 0, "Reset kept sensor state");
    [self selectBody:6];
    std::size_t before = _world.sensors.size();
    [self deleteBody:nil];
    check(_world.sensors.size() == before - 2, "Deleting a part must delete its sensors");

    // Noisy PID: the Control Lab switches the feedback source.
    [self noisyPidDemo:nil];
    check(_lab->_feedback.indexOfSelectedItem == 1, "Lab does not show encoder feedback");
    [_lab->_feedback selectItemAtIndex:0];
    [_lab apply:nil];
    check(!_world.sensors[0].feedsController, "Ideal feedback not applied");
    [_lab->_feedback selectItemAtIndex:1];
    _lab->_encoderLatency.stringValue = @"2";
    [_lab apply:nil];
    check(_world.sensors[0].feedsController && _world.sensors[0].latency == 2,
          "Encoder feedback not applied");
    [self toggleRun:nil];
    bool differs = false;
    for (int i = 0; i < 480; ++i) {
        stepSimulation(_world);
        const ControlSample& sample = _world.joints[0].history.back();
        differs = differs || std::abs(sample.measured - sample.angle) > 1e-6;
    }
    [self toggleRun:nil];
    check(differs, "Controller did not use the encoder reading");
    check([self writeLabImage:@"build/noisy-pid-lab-preview.png"], "Could not render noisy PID plots");
    [self reset:nil];
}

- (void)smokeCheck {
    try {
        auto check = [](bool ok, const char* message) {
            if (!ok) {
                throw std::runtime_error(message);
            }
        };
        // Establish preconditions explicitly instead of trusting the launch state.
        _running = NO;
        _hasSnapshot = NO;
        [self refresh];
        check(_world.bodies.size() == 7 && _world.bodies[0].name == "rover_chassis",
              "Starter rover did not initialize");
        [self selectBody:1];
        setVector(_size, {0.8, 0.8, 0.25});
        [self applyBody:nil];
        check(_world.bodies[1].size.x == 0.8, "Starter wheel is not editable");
        [_undo undo];
        check(_world.bodies[1].size.x != 0.8, "Undo did not restore the wheel");
        [_undo redo];
        check(_world.bodies[1].size.x == 0.8, "Redo did not reapply the wheel");
        [self rover:nil];
        [self drive:@"w"];
        for (int i = 0; i < 480; ++i) {
            _world.step();
        }
        [self drive:@"x"];
        [self toggleRun:nil];
        check(_world.bodies[0].position.x > 0.5, "Rover did not drive");
        [self reset:nil];
        check(_world.bodies[0].position.x == 0, "Rover reset failed");
        // Moving the chassis carries the whole rover with it and keeps every joint closed.
        [self selectBody:0];
        setVector(_position, {1, 0.7, 2});
        [self applyBody:nil];
        check(_world.bodies[0].position.x == 1 && std::abs(_world.bodies[1].position.z - 2.65) < 1e-9
                  && _world.maxAnchorError() < 1e-9,
              "Assembly move failed");
        [self sceneBeginMove:0];
        [self sceneMoveBy:{-1, 0, -2}];
        [self sceneEndMove];
        check(_world.bodies[0].position.x == 0 && _world.maxAnchorError() < 1e-9, "Viewport drag failed");
        [_undo undo];
        check(_world.bodies[0].position.x == 1, "Undo drag failed");
        _world = makeDemo();
        _selected = 1;
        [self invalidateSnapshot];
        [self refresh];
        [self addBox:nil];
        check(_world.bodies.size() == 6 && _selected == 5, "Add box failed");
        setVector(_position, {2, 2, 0});
        setVector(_size, {0.4, 0.8, 0.4});
        _mass.stringValue = @"2";
        [self applyBody:nil];
        check(_world.bodies[5].position.x == 2 && _world.bodies[5].mass == 2, "Inspector did not apply");
        [_parentList selectItemWithTag:0];
        [self chooseParent:nil];
        setVector(_anchor, {1, 1, 0});
        setVector(_axis, {0, 1, 0});
        [_jointType setSelected:YES forSegment:2];
        _speed.stringValue = @"0.4";
        _torque.stringValue = @"3";
        [self applyJoint:nil];
        check(_world.joints.size() == 4 && _world.joints.back().b == 5, "Joint creation failed");
        [self toggleRun:nil];
        for (int i = 0; i < 20; ++i) {
            _world.step();
        }
        [self toggleRun:nil];
        [self reset:nil];
        check(_world.time == 0 && _world.bodies[5].position.x == 2, "Reset failed");
        bool rejected = false;
        _mass.stringValue = @"abc";
        try {
            [self applyBody:nil];
        } catch (const std::exception&) {
            rejected = true;
        }
        check(rejected && _world.bodies[5].mass == 2, "Invalid number was not rejected");
        [self refresh];
        [self duplicateBody:nil];
        check(_world.bodies.size() == 7 && _world.bodies.back().name == "box_1_copy", "Duplicate failed");
        [self deleteBody:nil];
        [self selectBody:5];
        [self deleteBody:nil];
        check(_world.bodies.size() == 5 && _world.joints.size() == 3, "Delete failed");
        [self addSphere:nil];
        setVector(_size, {0.8, 0.1, 0.1});
        [self applyBody:nil];
        check(_world.bodies.back().size.y == 0.8, "Sphere diameter handling failed");
        std::stringstream data;
        writeScene(_world, data);
        World restored = readScene(data);
        check(restored.bodies.size() == 6, "Editor scene round trip failed");
        [self checkSliderInspector];
        _world = makeRoverDemo();
        _selected = 0;
        _hasSnapshot = NO;
        [self refresh];
        [_scene homeCamera];
        NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
        if (![png writeToFile:@"build/editor-preview.png" atomically:YES]) {
            throw std::runtime_error("Could not write editor preview");
        }
        [self checkLinearActuatorDemo];
        [self checkUrdfImport];
        [self checkSensors];
        [self checkEditorUX];
        [self openControlLab:nil];
        [_lab demo:nil];
        check(_world.joints.size() == 1 && _world.joints[0].controller.type == ControllerType::PID,
              "PID demo did not load");
        [_lab apply:nil];
        [self toggleRun:nil];
        for (int i = 0; i < 480; ++i) {
            _world.step();
        }
        _lab->_gains[0].stringValue = @"14";
        [_lab apply:nil];
        check(_running && _world.joints[0].history.size() == 480,
              "Live tuning lost history or paused the simulation");
        for (int i = 0; i < 960; ++i) {
            _world.step();
        }
        [self toggleRun:nil];
        check(std::abs(_world.joints[0].angle - 1) < 0.08, "PID lab did not track the step");
        saveScene(_world, "build/control-lab.robot");
        auto loaded = loadScene("build/control-lab.robot");
        check(loaded.joints[0].controller.kp == 14, "Controller save/load failed");
        std::ofstream csv("build/control-lab.csv");
        writeControlCsvHeader(csv);
        for (const auto& sample : _world.joints[0].history) {
            writeControlCsvRow(csv, 0, sample);
        }
        csv.close();
        check(static_cast<bool>(csv), "Control CSV export failed");
        check([self writeLabImage:@"build/control-lab-preview.png"], "Could not render control plots");
        [self reset:nil];
        check(_world.time == 0 && _world.joints[0].controller.kp == 14 && _world.joints[0].history.empty(),
              "Reset lost new gains or retained telemetry");
        [_lab->_kind selectItemAtIndex:2];
        [_lab->_variable selectItemAtIndex:1];
        [_lab->_signal selectItemAtIndex:0];
        _lab->_target.stringValue = @"1";
        [_lab apply:nil];
        [self toggleRun:nil];
        for (int i = 0; i < 120; ++i) {
            _world.step();
        }
        [_lab->_kind selectItemAtIndex:3];
        [_lab->_variable selectItemAtIndex:0];
        [_lab apply:nil];
        check(_world.joints[0].history.empty(), "Changing plot units retained old history");
        [self toggleRun:nil];
        [_lab->_kind selectItemAtIndex:5];
        [_lab apply:nil];
        check(_world.joints[0].controller.type == ControllerType::Relay, "Relay UI selection failed");
        std::cout
            << "PASS editor: construction, undo/redo, assembly move, validation, control lab, live "
               "tuning, reset, persistence, CSV, sensors, gizmos, timeline, plots, library, GPU rendering\n"
            << std::flush;
        [NSApp terminate:nil];
    } catch (const std::exception& e) {
        std::cerr << "FAIL editor: " << e.what() << '\n';
        std::exit(1);
    }
}

@end
