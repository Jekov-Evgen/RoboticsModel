// EditorController (Sensors): inspector card for the selected part's sensors, sensor demos, camera
// picture-in-picture rendering and CSV recording. Sensor models live in the core (robotics/sensors).

#import "mac/editor_controller.h"
#import "mac/control_lab.h"
#import "mac/sensor_panel.h"

#include "robotics/demo_scenes.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <array>
#include <cmath>
#include <string>

using namespace robotics;
using namespace editor_ui;

namespace {

// Generic number fields of the card. Slots 0-7 are common; 8-11 depend on the sensor type.
enum Slot {
    rateSlot,
    latencySlot,
    noiseSlot,
    biasSlot,
    walkSlot,
    resolutionSlot,
    dropoutSlot,
    seedSlot,
    slotA,
    slotB,
    slotC,
    slotD,
    slotCount
};

constexpr double degrees = 180 / robotics::pi;

// Label of a slot for the sensor type, or nil when the slot does not apply.
NSString* slotLabel(SensorType type, bool slider, int slot) {
    NSString* unit = @"";
    switch (type) {
    case SensorType::JointEncoder:
        unit = slider ? @"m" : @"rad";
        break;
    case SensorType::JointVelocity:
        unit = slider ? @"m/s" : @"rad/s";
        break;
    case SensorType::Imu:
        unit = @"m/s²";
        break;
    case SensorType::ForceTorque:
    case SensorType::Contact:
        unit = @"N";
        break;
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
        unit = @"m";
        break;
    case SensorType::Camera:
        break;
    }
    bool camera = type == SensorType::Camera;
    bool imu = type == SensorType::Imu;
    switch (slot) {
    case rateSlot:
        return camera ? @"Frame rate (Hz)" : @"Rate (Hz, 0 = every step)";
    case latencySlot:
        return camera ? nil : @"Latency (samples)";
    case noiseSlot:
        return camera ? nil : [NSString stringWithFormat:@"%@ σ (%@)", imu ? @"Accel noise" : @"Noise", unit];
    case biasSlot:
        return camera ? nil : [NSString stringWithFormat:@"%@ (%@)", imu ? @"Accel bias" : @"Bias", unit];
    case walkSlot:
        return camera ? nil : [NSString stringWithFormat:@"Bias walk (%@/√s)", unit];
    case resolutionSlot:
        return camera ? nil : [NSString stringWithFormat:@"Resolution (%@)", unit];
    case dropoutSlot:
        return camera ? nil : @"Dropout (%)";
    case seedSlot:
        return camera ? nil : @"Noise seed";
    default:
        break;
    }
    switch (type) {
    case SensorType::JointEncoder:
        return slot == slotA ? (slider ? @"Counts per metre" : @"Counts per rev") : nil;
    case SensorType::Imu:
        return slot == slotA
                   ? @"Gyro noise σ (rad/s)"
                   : (slot == slotB ? @"Gyro bias walk" : (slot == slotC ? @"Attitude σ (°)" : nil));
    case SensorType::ForceTorque:
        return slot == slotA ? @"Torque noise σ (N·m)" : (slot == slotB ? @"Torque bias (N·m)" : nil);
    case SensorType::Rangefinder:
        return slot == slotA ? @"Min range (m)" : (slot == slotB ? @"Max range (m)" : nil);
    case SensorType::Lidar2D:
        return slot == slotA
                   ? @"Min range (m)"
                   : (slot == slotB ? @"Max range (m)" : (slot == slotC ? @"Field of view (°)" : @"Beams"));
    case SensorType::Camera:
        return slot == slotA ? @"Width (px)"
                             : (slot == slotB ? @"Height (px)"
                                              : (slot == slotC ? @"Vertical FOV (°)" : @"Max depth (m)"));
    case SensorType::JointVelocity:
    case SensorType::Contact:
        return nil;
    }
    return nil;
}

std::array<double, slotCount> slotValues(const Sensor& s) {
    std::array<double, slotCount> v{};
    v[rateSlot] = s.rate;
    v[latencySlot] = s.latency;
    v[noiseSlot] = s.noise.stddev;
    v[biasSlot] = s.noise.bias;
    v[walkSlot] = s.noise.biasWalk;
    v[resolutionSlot] = s.noise.resolution;
    v[dropoutSlot] = s.dropout * 100;
    v[seedSlot] = static_cast<double>(s.seed);
    switch (s.type) {
    case SensorType::JointEncoder:
        v[slotA] = s.encoderCounts;
        break;
    case SensorType::Imu:
        v[slotA] = s.noise2.stddev;
        v[slotB] = s.noise2.biasWalk;
        v[slotC] = s.orientationNoise * degrees;
        break;
    case SensorType::ForceTorque:
        v[slotA] = s.noise2.stddev;
        v[slotB] = s.noise2.bias;
        break;
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
        v[slotA] = s.minRange;
        v[slotB] = s.maxRange;
        v[slotC] = s.fov * degrees;
        v[slotD] = s.beams;
        break;
    case SensorType::Camera:
        v[slotA] = s.width;
        v[slotB] = s.height;
        v[slotC] = s.fov * degrees;
        v[slotD] = s.maxRange;
        break;
    case SensorType::JointVelocity:
    case SensorType::Contact:
        break;
    }
    return v;
}

int wholeNumber(double value, const char* what) {
    if (!std::isfinite(value) || value != std::round(value) || std::abs(value) > 1e9) {
        throw std::runtime_error(std::string(what) + " must be a whole number");
    }
    return static_cast<int>(value);
}

void applySlots(Sensor& s, const std::array<double, slotCount>& v) {
    s.rate = v[rateSlot];
    if (s.type == SensorType::Camera) {
        s.width = wholeNumber(v[slotA], "Width");
        s.height = wholeNumber(v[slotB], "Height");
        s.fov = v[slotC] / degrees;
        s.maxRange = v[slotD];
        s.minRange = std::min(s.minRange, s.maxRange / 2);
        return;
    }
    s.latency = wholeNumber(v[latencySlot], "Latency");
    s.noise.stddev = v[noiseSlot];
    s.noise.bias = v[biasSlot];
    s.noise.biasWalk = v[walkSlot];
    s.noise.resolution = v[resolutionSlot];
    s.dropout = v[dropoutSlot] / 100;
    if (!(v[seedSlot] >= 0) || v[seedSlot] != std::round(v[seedSlot]) || v[seedSlot] > 9007199254740992.0) {
        throw std::runtime_error("Seed must be a non-negative whole number");
    }
    s.seed = static_cast<std::uint64_t>(v[seedSlot]);
    switch (s.type) {
    case SensorType::JointEncoder:
        s.encoderCounts = v[slotA];
        break;
    case SensorType::Imu:
        s.noise2.stddev = v[slotA];
        s.noise2.biasWalk = v[slotB];
        s.orientationNoise = v[slotC] / degrees;
        break;
    case SensorType::ForceTorque:
        s.noise2.stddev = v[slotA];
        s.noise2.bias = v[slotB];
        break;
    case SensorType::Rangefinder:
        s.minRange = v[slotA];
        s.maxRange = v[slotB];
        break;
    case SensorType::Lidar2D:
        s.minRange = v[slotA];
        s.maxRange = v[slotB];
        s.fov = v[slotC] / degrees;
        s.beams = wholeNumber(v[slotD], "Beams");
        break;
    case SensorType::JointVelocity:
    case SensorType::Contact:
    case SensorType::Camera:
        break;
    }
}

bool usesMount(SensorType type) {
    return type == SensorType::Imu || type == SensorType::Rangefinder || type == SensorType::Lidar2D
           || type == SensorType::Camera || type == SensorType::ForceTorque;
}

// Default mount: cameras and rangefinders on the part's +X face, lidars on top, others at the centre.
Vec3 defaultMount(const Body& body, SensorType type) {
    double halfX = body.shape == Shape::Box ? body.size.x / 2 : body.size.x / 2;
    double halfY = body.shape == Shape::Cylinder ? body.size.x / 2 : body.size.y / 2;
    switch (type) {
    case SensorType::Camera:
    case SensorType::Rangefinder:
        return {halfX, 0, 0};
    case SensorType::Lidar2D:
        return {0, halfY + 0.03, 0};
    default:
        return {};
    }
}

NSImage* rgbaImage(const std::vector<unsigned char>& rgba, int width, int height) {
    NSBitmapImageRep* rep = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:nullptr
                                                                    pixelsWide:width
                                                                    pixelsHigh:height
                                                                 bitsPerSample:8
                                                               samplesPerPixel:4
                                                                      hasAlpha:YES
                                                                      isPlanar:NO
                                                                colorSpaceName:NSDeviceRGBColorSpace
                                                                   bytesPerRow:width * 4
                                                                  bitsPerPixel:32];
    std::copy(rgba.begin(), rgba.end(), rep.bitmapData);
    NSImage* image = [[NSImage alloc] initWithSize:NSMakeSize(width, height)];
    [image addRepresentation:rep];
    return image;
}

} // namespace

@implementation EditorController (Sensors)

#pragma mark Inspector card

- (void)buildSensorCard {
    _shownSensor = -1;
    _cameraSensor = -1;
    _sensorCard = [[CardView alloc] initWithFrame:NSMakeRect(12, 0, 300, 596)];
    [_inspector addSubview:_sensorCard];
    heading(_sensorCard, @"SENSORS", 12);
    _sensorAdd = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(10, 32, 280, 26) pullsDown:YES];
    _sensorAdd.autoenablesItems = NO;
    [_sensorAdd addItemWithTitle:@"+ Add sensor"];
    NSArray* kinds = @[
        @[ @"IMU (accelerometer + gyro)", @(static_cast<int>(SensorType::Imu)) ],
        @[ @"Rangefinder", @(static_cast<int>(SensorType::Rangefinder)) ],
        @[ @"2D Lidar", @(static_cast<int>(SensorType::Lidar2D)) ],
        @[ @"Camera (RGB + depth)", @(static_cast<int>(SensorType::Camera)) ],
        @[ @"Contact / touch", @(static_cast<int>(SensorType::Contact)) ],
        @[ @"Encoder (on its joint)", @(static_cast<int>(SensorType::JointEncoder)) ],
        @[ @"Joint velocity (on its joint)", @(static_cast<int>(SensorType::JointVelocity)) ],
        @[ @"Force/torque (on its joint)", @(static_cast<int>(SensorType::ForceTorque)) ]
    ];
    for (NSArray* kind in kinds) {
        [_sensorAdd addItemWithTitle:kind[0]];
        _sensorAdd.lastItem.tag = [kind[1] integerValue];
        _sensorAdd.lastItem.target = self;
        _sensorAdd.lastItem.action = @selector(addSensor:);
    }
    _sensorAdd.toolTip = @"Attach a sensor to the selected part. Joint sensors measure the connection to its "
                         @"parent.";
    [_sensorCard addSubview:_sensorAdd];
    _sensorList = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(10, 64, 196, 26) pullsDown:NO];
    _sensorList.target = self;
    _sensorList.action = @selector(chooseSensor:);
    _sensorList.toolTip = @"Sensors attached to this part";
    [_sensorCard addSubview:_sensorList];
    _sensorRemove = pushButton(_sensorCard, @"Remove", NSMakeRect(208, 63, 84, 28), self,
                               @selector(removeSensor:), @"trash");
    label(_sensorCard, @"Name", NSMakeRect(14, 102, 60, 16), 11.5, NSFontWeightMedium);
    _sensorName = [[NSTextField alloc] initWithFrame:NSMakeRect(60, 98, 226, 24)];
    _sensorName.target = self;
    _sensorName.action = @selector(applySensor:);
    _sensorName.cell.sendsActionOnEndEditing = YES;
    [_sensorCard addSubview:_sensorName];
    NSMutableArray<NSTextField*>* fields = [NSMutableArray array];
    NSMutableArray<NSTextField*>* labels = [NSMutableArray array];
    for (int k = 0; k < slotCount; ++k) {
        CGFloat x = k % 2 == 0 ? 14 : 152;
        CGFloat y = 130 + (k / 2) * 44;
        [labels addObject:label(_sensorCard, @"", NSMakeRect(x, y, 136, 16), 10.5, NSFontWeightMedium)];
        [fields addObject:numberField(_sensorCard, NSMakeRect(x, y + 17, 128, 22), self,
                                      @selector(applySensor:))];
    }
    _sensorFields = fields;
    _sensorLabels = labels;
    _sensorMount = xyzRow(_sensorCard, @"Mount offset (m, part frame)", 400, self, @selector(applySensor:),
                          @"Sensor position relative to the part's centre.");
    _sensorMountRotation =
        xyzRow(_sensorCard, @"Mount rotation (degrees)", 450, self, @selector(applySensor:),
               @"Sensor orientation relative to the part. Sensors look along their +X.");
    _sensorFeeds = [NSButton checkboxWithTitle:@"Joint controller uses this reading"
                                        target:self
                                        action:@selector(applySensor:)];
    _sensorFeeds.frame = NSMakeRect(12, 502, 276, 20);
    _sensorFeeds.toolTip = @"Feedback source for the joint's P/PI/PD/PID controller: this (noisy, delayed, "
                           @"quantized) reading instead of the ideal value.";
    [_sensorCard addSubview:_sensorFeeds];
    _sensorHint = wrapping(_sensorCard, @"", NSMakeRect(14, 528, 272, 56), 11);
}

- (NSString*)sensorFieldSignature {
    NSMutableString* s = [NSMutableString stringWithString:_sensorName.stringValue];
    for (NSTextField* f in [[_sensorFields arrayByAddingObjectsFromArray:_sensorMount]
             arrayByAddingObjectsFromArray:_sensorMountRotation]) {
        [s appendFormat:@"\x1f%@", f.stringValue];
    }
    [s appendFormat:@"\x1f%ld", static_cast<long>(_sensorFeeds.state)];
    return s;
}

- (void)refreshSensorCard {
    if (!_sensorCard) {
        return;
    }
    BOOL edit = !_running && !_reviewing;
    [_sensorList removeAllItems];
    if (_selected < 0 || _selected >= static_cast<int>(_world.bodies.size())) {
        _shownSensor = -1;
        return;
    }
    bool hasJoint = false;
    bool hasActuated = false;
    for (const auto& j : _world.joints) {
        if (j.b == _selected) {
            hasJoint = true;
            hasActuated = hasActuated || j.isActuated();
        }
    }
    for (NSMenuItem* item in _sensorAdd.itemArray) {
        if (item.action != @selector(addSensor:)) {
            continue;
        }
        auto type = static_cast<SensorType>(item.tag);
        item.enabled =
            edit
            && (type == SensorType::ForceTorque ? hasJoint : (isJointSensorType(type) ? hasActuated : true));
    }
    _sensorAdd.enabled = edit;
    bool shownHere = false;
    for (int i = 0; i < static_cast<int>(_world.sensors.size()); ++i) {
        const Sensor& s = _world.sensors[static_cast<std::size_t>(i)];
        if (s.body != _selected) {
            continue;
        }
        [_sensorList
            addItemWithTitle:[NSString stringWithFormat:@"%s · %s", s.name.c_str(),
                                                        std::string(sensorTypeName(s.type)).c_str()]];
        _sensorList.lastItem.tag = i;
        shownHere = shownHere || i == _shownSensor;
    }
    if (_sensorList.numberOfItems == 0) {
        [_sensorList addItemWithTitle:@"No sensors on this part"];
        _sensorList.lastItem.tag = -1;
        _shownSensor = -1;
    } else if (!shownHere) {
        _shownSensor = static_cast<int>(_sensorList.itemArray.firstObject.tag);
    }
    [_sensorList selectItemWithTag:_shownSensor];
    BOOL has = _shownSensor >= 0;
    setEnabledRecursive(_sensorCard, edit);
    _sensorAdd.enabled = edit;
    _sensorList.enabled = has;
    _sensorRemove.enabled = edit && has;
    _sensorName.enabled = edit && has;
    _sensorFeeds.hidden = YES;
    for (NSTextField* f in [_sensorFields
             arrayByAddingObjectsFromArray:[_sensorMount
                                               arrayByAddingObjectsFromArray:_sensorMountRotation]]) {
        f.textColor = NSColor.labelColor;
    }
    if (!has) {
        _sensorName.stringValue = @"";
        for (int k = 0; k < slotCount; ++k) {
            _sensorLabels[k].stringValue = @"";
            _sensorFields[k].stringValue = @"";
            _sensorFields[k].hidden = YES;
        }
        for (NSTextField* f in [_sensorMount arrayByAddingObjectsFromArray:_sensorMountRotation]) {
            f.stringValue = @"";
            f.enabled = NO;
        }
        _sensorHint.stringValue =
            @"Add a sensor with the menu above. Sensors are simulated with noise, rate, "
            @"latency and dropouts; see their readings in Simulation > Sensors.";
        _shownSensorValues = [self sensorFieldSignature];
        return;
    }
    const Sensor& s = _world.sensors[static_cast<std::size_t>(_shownSensor)];
    int joint = sensorJoint(_world, s);
    bool slider = joint >= 0 && _world.joints[static_cast<std::size_t>(joint)].type == JointType::Prismatic;
    _sensorName.stringValue = @(s.name.c_str());
    std::array<double, slotCount> values = slotValues(s);
    for (int k = 0; k < slotCount; ++k) {
        NSString* title = slotLabel(s.type, slider, k);
        _sensorLabels[k].stringValue = title ? title : @"";
        _sensorFields[k].hidden = title == nil;
        _sensorFields[k].stringValue = title ? [NSString stringWithFormat:@"%.9g", values[k]] : @"";
    }
    if (s.type == SensorType::Rangefinder) { // A rangefinder has one beam straight ahead.
        _sensorFields[slotC].hidden = _sensorFields[slotD].hidden = YES;
        _sensorLabels[slotC].stringValue = _sensorLabels[slotD].stringValue = @"";
    }
    BOOL mount = usesMount(s.type);
    setVector(_sensorMount, s.position);
    setVector(_sensorMountRotation, s.orientation.eulerAngles() * degrees);
    for (NSTextField* f in _sensorMount) {
        f.enabled = edit && mount && s.type != SensorType::ForceTorque;
    }
    for (NSTextField* f in _sensorMountRotation) {
        f.enabled = edit && mount;
    }
    bool feedable = s.type == SensorType::JointEncoder || s.type == SensorType::JointVelocity;
    _sensorFeeds.hidden = !feedable;
    _sensorFeeds.state = s.feedsController ? NSControlStateValueOn : NSControlStateValueOff;
    NSString* hint = @"";
    switch (s.type) {
    case SensorType::JointEncoder:
    case SensorType::JointVelocity:
        hint = joint >= 0 ? @"Measures this part's hinge or slider. Tick the box to close the control loop "
                            @"through the sensor (also in the Control Lab)."
                          : @"Needs a hinge or slider to the parent part.";
        break;
    case SensorType::ForceTorque:
        hint = @"Wrench the parent applies to this part through the joint, at the anchor, in the part frame "
               @"(force N, torque N·m).";
        break;
    case SensorType::Imu:
        hint = @"Specific force (incl. gravity: +9.81 up at rest), angular velocity and orientation in the "
               @"sensor frame.";
        break;
    case SensorType::Contact:
        hint = @"Reads 1 while the part touches anything, and the total normal contact force.";
        break;
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
        hint = @"Rays along +X (lidar: sweeping the X-Z plane) ignore this part. No return reads max range.";
        break;
    case SensorType::Camera:
        hint = @"Looks along +X with +Y up. The image appears in the 3D view and in the Sensors window.";
        break;
    }
    _sensorHint.stringValue = _running ? @"Pause the simulation (Space) to edit sensors." : hint;
    _sensorHint.textColor = _running ? NSColor.systemOrangeColor : NSColor.secondaryLabelColor;
    _shownSensorValues = [self sensorFieldSignature];
}

- (void)addSensor:(NSMenuItem*)sender {
    if (![self canEdit] || _selected < 0) {
        return;
    }
    auto type = static_cast<SensorType>(sender.tag);
    Sensor sensor = makeSensor(type, _selected);
    const Body& body = _world.bodies[static_cast<std::size_t>(_selected)];
    if (sensor.isJointSensor() && sensorJoint(_world, sensor) < 0) {
        [self showError:
                  type == SensorType::ForceTorque
                      ? @"A force/torque sensor needs a connection to a parent part."
                      : @"Encoders and joint velocity sensors need a hinge or slider to the parent part."];
        return;
    }
    sensor.position = defaultMount(body, type);
    std::string base = body.name + "_" + sensor.name;
    std::string name = base;
    for (int n = 2;; ++n) {
        bool taken = false;
        for (const auto& other : _world.sensors) {
            taken = taken || other.name == name;
        }
        if (!taken) {
            break;
        }
        name = base + "_" + std::to_string(n);
    }
    sensor.name = name;
    sensor.seed = _world.sensors.size() + 1;
    [_window makeFirstResponder:_scene];
    [self recordUndo:[NSString stringWithFormat:@"Add %s", std::string(sensorTypeName(type)).c_str()]];
    _world.sensors.push_back(sensor);
    _shownSensor = static_cast<int>(_world.sensors.size()) - 1;
    [self invalidateSnapshot];
    [self refresh];
    [self toast:[NSString stringWithFormat:@"Added %s. Press Run to see its readings (Simulation > Sensors).",
                                           name.c_str()]];
}

- (void)chooseSensor:(id)sender {
    (void)sender;
    _shownSensor = static_cast<int>(_sensorList.selectedItem.tag);
    [self refreshSensorCard];
    if (_sensorPanel && _shownSensor >= 0) {
        [_sensorPanel->_sensor selectItemWithTag:_shownSensor];
        [_sensorPanel tick];
    }
    _scene.needsDisplay = YES;
}

- (void)applySensor:(id)sender {
    (void)sender;
    if (_running || _reviewing || _shownSensor < 0
        || _shownSensor >= static_cast<int>(_world.sensors.size())) {
        return;
    }
    if ([[self sensorFieldSignature] isEqualToString:_shownSensorValues]) {
        return;
    }
    clearInvalidNumberField();
    try {
        Sensor sensor = _world.sensors[static_cast<std::size_t>(_shownSensor)];
        std::string name = _sensorName.stringValue.UTF8String;
        if (name.empty()) {
            throw std::runtime_error("A sensor needs a name");
        }
        sensor.name = name;
        std::array<double, slotCount> values = slotValues(sensor);
        for (int k = 0; k < slotCount; ++k) {
            if (!_sensorFields[k].hidden) {
                values[k] = number(_sensorFields[k]);
            }
        }
        applySlots(sensor, values);
        if (usesMount(sensor.type)) {
            if (sensor.type != SensorType::ForceTorque) {
                sensor.position = getVector(_sensorMount);
            }
            sensor.orientation = Quat::euler(getVector(_sensorMountRotation) / degrees);
        }
        sensor.feedsController = !_sensorFeeds.hidden && _sensorFeeds.state == NSControlStateValueOn;
        sensor.state = {};
        validateSensor(sensor, _world.bodies.size());
        [self recordUndo:@"Edit Sensor"];
        _world.sensors[static_cast<std::size_t>(_shownSensor)] = std::move(sensor);
        [self invalidateSnapshot];
        [self refresh];
    } catch (const std::exception& e) {
        markInvalidNumberField();
        [self showError:@(e.what())];
    }
}

- (void)removeSensor:(id)sender {
    (void)sender;
    if (![self canEdit] || _shownSensor < 0 || _shownSensor >= static_cast<int>(_world.sensors.size())) {
        return;
    }
    std::string name = _world.sensors[static_cast<std::size_t>(_shownSensor)].name;
    [self recordUndo:@"Remove Sensor"];
    _world.sensors.erase(_world.sensors.begin() + _shownSensor);
    _shownSensor = -1;
    [self invalidateSnapshot];
    [self refresh];
    [self toast:[NSString stringWithFormat:@"Removed %s — undo with ⌘Z", name.c_str()]];
}

#pragma mark Window, demos

- (void)openSensors:(id)sender {
    (void)sender;
    if (!_sensorPanel) {
        _sensorPanel = [[SensorPanel alloc] initWithEditor:self];
    }
    [_sensorPanel show];
}

- (void)sensorRover:(id)sender {
    (void)sender;
    [self loadWorld:makeSensorRoverDemo() selected:6 name:@"Load Sensor Rover"];
    if (!isEditorSmokeTest()) {
        [self
            toast:
                @"Sensor Rover: press W to drive. Lidar rays are green, hits red; the camera view is at the "
                @"bottom right."];
    }
}

- (void)noisyPidDemo:(id)sender {
    (void)sender;
    [self loadWorld:makeNoisyPidDemo() selected:1 name:@"Load Noisy PID"];
    [self openControlLab:nil];
}

#pragma mark Live data

- (void)afterSimulationStep {
    if (_sensorLog) {
        writeSensorCsvRows(*_sensorLog, _world);
    }
}

- (BOOL)startSensorLog:(NSString*)path {
    auto log = std::make_unique<std::ofstream>(path.fileSystemRepresentation);
    if (!*log) {
        return NO;
    }
    writeSensorCsvHeader(*log);
    _sensorLog = std::move(log);
    return YES;
}

- (void)stopSensorLog {
    if (_sensorLog) {
        _sensorLog->close();
        _sensorLog.reset();
    }
}

- (void)updateSensorViews {
    // Camera shown in the picture-in-picture: the selected part's camera, else the first one.
    int camera = -1;
    for (int i = 0; i < static_cast<int>(_world.sensors.size()); ++i) {
        const Sensor& s = _world.sensors[static_cast<std::size_t>(i)];
        if (s.type == SensorType::Camera && s.body >= 0 && s.body < static_cast<int>(_world.bodies.size())
            && (camera < 0 || s.body == _selected)) {
            camera = i;
        }
    }
    if (camera < 0) {
        _cameraSensor = -1;
        [_scene showCameraColor:nil depth:nil title:@""];
    } else {
        const Sensor& s = _world.sensors[static_cast<std::size_t>(camera)];
        // While running, render at the camera's frame rate; while editing, on every refresh.
        bool due = !_running || s.state.fresh || camera != _cameraSensor || _cameraColor.empty();
        if (due && [_scene renderSensorCamera:s color:_cameraColor depth:_cameraDepth]) {
            _cameraSensor = camera;
            std::vector<unsigned char> gray(_cameraDepth.size() * 4);
            for (std::size_t i = 0; i < _cameraDepth.size(); ++i) {
                auto v =
                    static_cast<unsigned char>(255 * std::clamp(1 - _cameraDepth[i] / s.maxRange, 0.0, 1.0));
                gray[i * 4] = gray[i * 4 + 1] = gray[i * 4 + 2] = v;
                gray[i * 4 + 3] = 255;
            }
            [_scene showCameraColor:rgbaImage(_cameraColor, s.width, s.height)
                              depth:rgbaImage(gray, s.width, s.height)
                              title:[NSString stringWithFormat:@"%s  ·  %d×%d  ·  RGB | depth",
                                                               s.name.c_str(), s.width, s.height]];
        }
    }
    [_sensorPanel tick];
}

@end
