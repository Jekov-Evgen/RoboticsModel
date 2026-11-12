// EditorController (Actions): creating, duplicating and deleting parts; examples; driving; camera.

#import "mac/editor_controller.h"
#import "mac/control_lab.h"

#include "robotics/demo_scenes.hpp"

#include <algorithm>
#include <string>

using namespace robotics;
using namespace editor_ui;

namespace {

constexpr double maxWheelSpeed = 12.0; // rad/s, as driveRover().

// Like driveRover(), but independent of how the robot is oriented in its own frames, so imported
// robots drive too: forward is the chassis' local +X (URDF convention, also the rover's), sides and
// spin direction come from the world geometry. Gives the same commands as driveRover() for the rover.
int driveWheels(World& world, double forward, double turn) {
    int count = 0;
    for (auto& joint : world.joints) {
        const Body& chassis = world.bodies[static_cast<std::size_t>(joint.a)];
        if (!isWheelJoint(world, joint) || !joint.usesSpeedMotor()) {
            continue;
        }
        Vec3 ahead = chassis.orientation.rotate({1, 0, 0});
        Vec3 right = ahead.cross({0, 1, 0}).normalized();
        Vec3 axis = chassis.orientation.rotate(joint.axisA);
        bool left = (chassis.point(joint.anchorA) - chassis.position).dot(right) < 0;
        double speed = std::clamp(-forward + (left ? turn : -turn), -maxWheelSpeed, maxWheelSpeed);
        joint.motorSpeed = axis.dot(right) < 0 ? -speed : speed;
        ++count;
    }
    return count;
}

} // namespace

@implementation EditorController (Actions)

- (void)openControlLab:(id)sender {
    (void)sender;
    if (!_lab) {
        _lab = [[ControlLab alloc] initWithEditor:self];
    }
    [_lab show];
}

- (void)homeCamera:(id)sender {
    (void)sender;
    [_scene homeCamera];
}

- (void)cameraPreset:(NSMenuItem*)sender {
    [_scene viewPreset:sender.tag];
}

- (void)loadWorld:(World)world selected:(int)selected name:(NSString*)name {
    _running = NO;
    [_window makeFirstResponder:_scene];
    [self recordUndo:name];
    _world = std::move(world);
    _selected = selected;
    [self invalidateSnapshot];
    [self refresh];
    [_scene homeCamera];
    if (!isEditorSmokeTest()) {
        [self toast:[NSString stringWithFormat:@"%@ — undo with ⌘Z", name]];
    }
}

- (void)newScene:(id)sender {
    (void)sender;
    [self loadWorld:World {} selected:-1 name:@"New Robot"];
    _fileURL = nil;
    [self updateStatus];
}

- (void)demo:(id)sender {
    (void)sender;
    [self loadWorld:makeDemo() selected:1 name:@"Load Robot Arm"];
}

- (void)rover:(id)sender {
    (void)sender;
    [self loadWorld:makeRoverDemo() selected:0 name:@"Load Starter Rover"];
}

- (void)pidDemo:(id)sender {
    (void)sender;
    [self loadWorld:makeControlDemo() selected:1 name:@"Load PID Test Bench"];
    [self openControlLab:nil];
}

- (void)actuatorDemo:(id)sender {
    (void)sender;
    [self loadWorld:makeLinearActuatorDemo() selected:2 name:@"Load Linear Actuator & Limits"];
}

- (void)drive:(NSString*)key {
    double forward = [key isEqualToString:@"w"] ? 3 : ([key isEqualToString:@"s"] ? -3 : 0);
    double turn = [key isEqualToString:@"a"] ? 2 : ([key isEqualToString:@"d"] ? -2 : 0);
    int count = driveWheels(_world, forward, turn);
    if (count == 0) {
        [self showError:@"Nothing to drive: WASD controls wheels attached with a hinge joint."];
        return;
    }
    if (!_running && ![key isEqualToString:@"x"]) {
        [self toggleRun:nil];
    }
    NSString* what =
        [key isEqualToString:@"x"]
            ? @"Braking"
            : ([key isEqualToString:@"w"] ? @"Driving forward"
                                          : ([key isEqualToString:@"s"] ? @"Reversing" : @"Turning"));
    [self toast:[NSString
                    stringWithFormat:@"%@ · %d wheel%@ · X to brake", what, count, count == 1 ? @"" : @"s"]];
}

- (void)addShape:(Shape)shape {
    if (![self canEdit]) {
        return;
    }
    [_window makeFirstResponder:_scene];
    [self recordUndo:[NSString stringWithFormat:@"Add %@", shapeName(shape)]];
    Body b;
    b.shape = shape;
    std::string base = shape == Shape::Box ? "box" : (shape == Shape::Sphere ? "ball" : "wheel");
    int n = 1;
    auto taken = [&](const std::string& name) {
        return std::any_of(_world.bodies.begin(), _world.bodies.end(),
                           [&](const Body& x) { return x.name == name; });
    };
    while (taken(base + "_" + std::to_string(n))) {
        ++n;
    }
    b.name = base + "_" + std::to_string(n);
    if (shape == Shape::Cylinder) {
        b.size = {0.6, 0.6, 0.2};
    }
    b.position = _selected >= 0 ? _world.bodies[_selected].position
                                      + Vec3{0, _world.bodies[_selected].size.y / 2 + b.size.y / 2 + 0.3, 0}
                                : Vec3{0, b.size.y / 2 + 0.5, 0};
    _selected = _world.addBody(b);
    [self invalidateSnapshot];
    [self refresh];
    [self toast:[NSString stringWithFormat:@"Added %s. Use \"Attach to\" in the Inspector to connect it.",
                                           b.name.c_str()]];
}

- (void)addBox:(id)sender {
    (void)sender;
    [self addShape:Shape::Box];
}

- (void)addSphere:(id)sender {
    (void)sender;
    [self addShape:Shape::Sphere];
}

- (void)addCylinder:(id)sender {
    (void)sender;
    [self addShape:Shape::Cylinder];
}

- (void)duplicateBody:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    [_window makeFirstResponder:_scene];
    [self recordUndo:@"Duplicate Part"];
    Body copy = _world.bodies[_selected];
    copy.name += "_copy";
    copy.position += Vec3{0, 0, copy.size.z + 0.2};
    copy.velocity = {};
    copy.angularVelocity = {};
    _selected = _world.addBody(copy);
    [self invalidateSnapshot];
    [self refresh];
    [self toast:@"Duplicated. The copy is not connected to anything yet."];
}

- (void)deleteBody:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    std::string name = _world.bodies[_selected].name;
    [_window makeFirstResponder:_scene];
    [self recordUndo:@"Delete Part"];
    _world.removeBody(_selected);
    if (_selected >= static_cast<int>(_world.bodies.size())) {
        _selected = static_cast<int>(_world.bodies.size()) - 1;
    }
    [self invalidateSnapshot];
    [self refresh];
    [self toast:[NSString stringWithFormat:@"Deleted %s — undo with ⌘Z", name.c_str()]];
}

@end
