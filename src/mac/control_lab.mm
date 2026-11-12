// Control Lab window: controller settings per hinge or slider joint and live response plots
// (target, error, torque/force, P/I/D). Units follow the joint type: rad and N·m, or m and N.

#import "mac/control_lab.h"
#import "mac/editor_controller.h"
#import "mac/sensor_panel.h"

#include "robotics/demo_scenes.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <deque>
#include <fstream>
#include <vector>

using namespace robotics;
using namespace editor_ui;

@interface ControlPlotView : NSView
@property (nonatomic, weak) ControlLab* lab;
@end

namespace {

void plotText(NSString* text, NSPoint position, NSColor* color, CGFloat size = 11) {
    [text drawAtPoint:position
        withAttributes:@{
            NSFontAttributeName : [NSFont monospacedDigitSystemFontOfSize:size weight:NSFontWeightRegular],
            NSForegroundColorAttributeName : color
        }];
}

// Encoder sensor measuring joint `index` in `world`, or -1.
int encoderFor(const World& world, int index) {
    for (int i = 0; i < static_cast<int>(world.sensors.size()); ++i) {
        const Sensor& s = world.sensors[static_cast<std::size_t>(i)];
        if (s.type == SensorType::JointEncoder && sensorJoint(world, s) == index) {
            return i;
        }
    }
    return -1;
}

// Applies the Control Lab feedback settings to joint `index` of `world` (creating the encoder if needed).
void applyFeedback(World& world, int index, bool useSensor, double noise, int latency, double counts,
                   double rate) {
    int encoder = encoderFor(world, index);
    if (encoder < 0 && !useSensor) {
        for (auto& s : world.sensors) {
            if (s.type == SensorType::JointVelocity && sensorJoint(world, s) == index) {
                s.feedsController = false;
            }
        }
        return;
    }
    if (encoder < 0) {
        int child = world.joints[static_cast<std::size_t>(index)].b;
        Sensor sensor = makeSensor(SensorType::JointEncoder, child);
        sensor.name = world.bodies[static_cast<std::size_t>(child)].name + "_encoder";
        world.sensors.push_back(sensor);
        encoder = static_cast<int>(world.sensors.size()) - 1;
    }
    Sensor& sensor = world.sensors[static_cast<std::size_t>(encoder)];
    sensor.feedsController = useSensor;
    sensor.noise.stddev = noise;
    sensor.latency = latency;
    sensor.encoderCounts = counts;
    sensor.rate = rate;
    sensor.state = {};
    for (auto& s : world.sensors) {
        if (s.type == SensorType::JointVelocity && sensorJoint(world, s) == index && !useSensor) {
            s.feedsController = false;
        }
    }
}

double channelValue(const ControlSample& sample, int channel, bool position = true) {
    switch (channel) {
    case 7:
        return position ? sample.angle : sample.speed;
    case 0:
        return sample.target;
    case 1:
        return sample.measured;
    case 2:
        return sample.error;
    case 3:
        return sample.effort;
    case 4:
        return sample.p;
    case 5:
        return sample.i;
    default:
        return sample.d;
    }
}

// showTrue: the controller measures through a sensor, so "Measured" and "True" differ (chart 0).
void drawChart(NSRect frame, const std::deque<ControlSample>& history, int chart, bool position, bool linear,
               double torqueLimit, bool showTrue = false) {
    NSArray<NSColor*>* colors = @[
        NSColor.systemOrangeColor, NSColor.systemCyanColor, NSColor.systemRedColor, NSColor.systemGreenColor,
        NSColor.systemOrangeColor, NSColor.systemPurpleColor, NSColor.systemBlueColor, NSColor.systemPinkColor
    ];
    NSArray<NSString*>* names = @[
        @"Target", showTrue ? @"Measured" : @"Actual", @"Error", linear ? @"Force" : @"Torque", @"P", @"I",
        @"D", @"True"
    ];
    const std::vector<int> channels = chart == 0
                                          ? (showTrue ? std::vector<int>{0, 1, 7} : std::vector<int>{0, 1})
                                          : (chart == 1 ? std::vector<int>{2} : std::vector<int>{3, 4, 5, 6});
    NSString* unit = linear ? (position ? @"m" : @"m/s") : (position ? @"rad" : @"rad/s");
    NSString* effortTitle = linear ? @"Force and PID terms (N)" : @"Torque and PID terms (N·m)";
    NSString* title = chart == 0
                          ? [NSString stringWithFormat:@"Response (%@)", unit]
                          : (chart == 1 ? [NSString stringWithFormat:@"Error (%@)", unit] : effortTitle);
    plotText(title, NSMakePoint(frame.origin.x + 10, frame.origin.y + 3), NSColor.labelColor, 12);
    CGFloat legendX = frame.origin.x + frame.size.width - 270;
    for (int channel : channels) {
        plotText(names[channel], NSMakePoint(legendX, frame.origin.y + 3), colors[channel]);
        legendX += 65;
    }
    NSRect area =
        NSMakeRect(frame.origin.x + 65, frame.origin.y + 30, frame.size.width - 85, frame.size.height - 58);
    [[NSColor colorWithCalibratedWhite:0.07 alpha:1] setFill];
    NSRectFill(area);
    if (history.empty()) {
        plotText(@"Press Run to collect samples", NSMakePoint(area.origin.x + 20, area.origin.y + 30),
                 NSColor.secondaryLabelColor);
        return;
    }
    double ymin = 0;
    double ymax = 0;
    for (const auto& sample : history) {
        for (int channel : channels) {
            double v = channelValue(sample, channel, position);
            ymin = std::min(ymin, v);
            ymax = std::max(ymax, v);
        }
    }
    if (chart == 2) {
        ymin = std::min(ymin, -torqueLimit);
        ymax = std::max(ymax, torqueLimit);
    }
    double padding = std::max(0.01, (ymax - ymin) * 0.08);
    ymin -= padding;
    ymax += padding;
    double tmin = history.front().time;
    double tmax = std::max(tmin + 1, history.back().time);
    auto point = [&](double t, double value) {
        return NSMakePoint(area.origin.x + (t - tmin) / (tmax - tmin) * area.size.width,
                           area.origin.y + area.size.height
                               - (value - ymin) / (ymax - ymin) * area.size.height);
    };
    for (int k = 0; k <= 4; ++k) {
        double value = ymin + (ymax - ymin) * k / 4;
        NSPoint a = point(tmin, value);
        NSPoint b = point(tmax, value);
        [NSColor.quaternaryLabelColor setStroke];
        NSBezierPath* grid = [NSBezierPath bezierPath];
        [grid moveToPoint:a];
        [grid lineToPoint:b];
        [grid stroke];
        plotText([NSString stringWithFormat:@"%.3g", value], NSMakePoint(frame.origin.x + 5, a.y - 7),
                 NSColor.secondaryLabelColor, 10);
        double t = tmin + (tmax - tmin) * k / 4;
        NSPoint tx = point(t, ymin);
        plotText([NSString stringWithFormat:@"%.2f", t], NSMakePoint(tx.x - 12, NSMaxY(area) + 5),
                 NSColor.secondaryLabelColor, 10);
    }
    plotText(@"s", NSMakePoint(NSMaxX(area) + 8, NSMaxY(area) + 5), NSColor.secondaryLabelColor, 10);
    [NSGraphicsContext saveGraphicsState];
    NSRectClip(area);
    if (chart == 2) {
        for (double limit : {-torqueLimit, torqueLimit}) {
            [NSColor.secondaryLabelColor setStroke];
            NSBezierPath* path = [NSBezierPath bezierPath];
            CGFloat dash[] = {4, 4};
            [path setLineDash:dash count:2 phase:0];
            [path moveToPoint:point(tmin, limit)];
            [path lineToPoint:point(tmax, limit)];
            [path stroke];
        }
    }
    for (int channel : channels) {
        [colors[channel] setStroke];
        NSBezierPath* path = [NSBezierPath bezierPath];
        path.lineWidth = (channel == 1 || channel == 3) ? 2 : 1.2;
        bool first = true;
        for (const auto& sample : history) {
            NSPoint p = point(sample.time, channelValue(sample, channel, position));
            if (first) {
                [path moveToPoint:p];
            } else {
                [path lineToPoint:p];
            }
            first = false;
        }
        [path stroke];
    }
    [NSGraphicsContext restoreGraphicsState];
}
} // namespace

@implementation ControlPlotView

- (BOOL)isFlipped {
    return YES;
}

- (void)drawRect:(NSRect)dirtyRect {
    (void)dirtyRect;
    [NSGraphicsContext saveGraphicsState];
    NSRectClip(self.bounds);
    [[NSColor colorWithCalibratedRed:0.055 green:0.075 blue:0.105 alpha:1] setFill];
    NSRectFill(self.bounds);
    ControlLab* lab = self.lab;
    EditorController* editor = lab ? lab->_editor : nil;
    int index = lab ? [lab jointIndex] : -1;
    if (!editor || index < 0 || index >= static_cast<int>(editor->_world.joints.size())) {
        plotText(@"No hinge or slider joint selected.", NSMakePoint(30, 30), NSColor.labelColor, 15);
        plotText(@"Choose a joint in the list on the left, or click \"Load PID step-response demo\".",
                 NSMakePoint(30, 56), NSColor.secondaryLabelColor, 12);
        [NSGraphicsContext restoreGraphicsState];
        return;
    }
    const Joint& j = editor->_world.joints[index];
    bool position =
        j.controller.type != ControllerType::SpeedMotor && j.controller.variable == ControlVariable::Position;
    bool linear = j.type == JointType::Prismatic;
    plotText([NSString stringWithFormat:@"Joint %d (%@)  |  %@  |  %zu samples / last 10 s at 240 Hz", index,
                                        jointName(j.type).lowercaseString,
                                        editor->_running ? @"RUNNING" : @"PAUSED", j.history.size()],
             NSMakePoint(18, 15), NSColor.labelColor, 12);
    if (!j.history.empty()) {
        const auto& sample = j.history.back();
        plotText([NSString stringWithFormat:@"t %.2f s   target %.3f   actual %.3f   %@ %.3f %@  %@",
                                            sample.time, sample.target, sample.measured,
                                            linear ? @"force" : @"torque", sample.effort,
                                            linear ? @"N" : @"N·m", sample.saturated ? @"[SATURATED]" : @""],
                 NSMakePoint(18, 39),
                 sample.saturated ? NSColor.systemOrangeColor : NSColor.secondaryLabelColor);
    }
    double height = (self.bounds.size.height - 76) / 3;
    for (int i = 0; i < 3; ++i) {
        drawChart(NSMakeRect(8, 70 + i * height, self.bounds.size.width - 16, height - 6), j.history, i,
                  position, linear, j.maxTorque, j.controllerState.sensorFeedback);
    }
    [NSGraphicsContext restoreGraphicsState];
}

@end

@implementation ControlLab

- (instancetype)initWithEditor:(EditorController*)editor {
    self = [super init];
    if (!self) {
        return nil;
    }
    _editor = editor;
    _window =
        [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 1180, 880)
                                    styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                              | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable
                                      backing:NSBackingStoreBuffered
                                        defer:NO];
    _window.title = @"Control Lab — PID and joint response";
    _window.releasedWhenClosed = NO;
    _window.minSize = NSMakeSize(1000, 880);
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    FlippedView* panel = [[FlippedView alloc] initWithFrame:NSMakeRect(0, 0, 310, 880)];
    panel.autoresizingMask = NSViewHeightSizable;
    [_window.contentView addSubview:panel];
    _plot = [[ControlPlotView alloc] initWithFrame:NSMakeRect(310, 0, 870, 880)];
    _plot.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    _plot.lab = self;
    [_window.contentView addSubview:_plot];
    label(panel, @"Joint (hinge or slider)", NSMakeRect(12, 16, 290, 18), 13, NSFontWeightSemibold);
    _joint = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(10, 40, 290, 26) pullsDown:NO];
    _joint.target = self;
    _joint.action = @selector(loadFields:);
    [panel addSubview:_joint];
    auto popup = [&](NSString* title, CGFloat y, NSArray<NSString*>* items, NSString* tip) {
        label(panel, title, NSMakeRect(12, y + 4, 82, 18));
        NSPopUpButton* menu = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(95, y, 205, 26) pullsDown:NO];
        [menu addItemsWithTitles:items];
        menu.target = self;
        menu.action = @selector(updateEnabled:);
        menu.toolTip = tip;
        [panel addSubview:menu];
        return menu;
    };
    _kind = popup(@"Controller", 84, @[ @"Speed motor", @"P", @"PI", @"PD", @"PID", @"Relay" ],
                  @"Speed motor: constant speed. P/PI/PD/PID/Relay: feedback control of position or speed.");
    _variable = popup(@"Controls", 120, @[ @"Angle (position)", @"Speed (velocity)" ],
                      @"Which joint quantity the controller tracks.");
    _signal = popup(@"Reference", 156, @[ @"Constant", @"Step", @"Sine" ],
                    @"Shape of the target signal over time.");
    _units =
        label(panel, @"", NSMakeRect(12, 192, 287, 16), 11, NSFontWeightRegular, NSColor.secondaryLabelColor);
    label(panel, @"Kp", NSMakeRect(25, 220, 70, 18));
    label(panel, @"Ki", NSMakeRect(123, 220, 70, 18));
    label(panel, @"Kd", NSMakeRect(221, 220, 70, 18));
    _gains = @[
        numberField(panel, NSMakeRect(12, 242, 88, 24)), numberField(panel, NSMakeRect(110, 242, 88, 24)),
        numberField(panel, NSMakeRect(208, 242, 88, 24))
    ];
    auto field = [&](NSString* title, CGFloat y, NSTextField** titleLabel = nullptr) {
        NSTextField* text = label(panel, title, NSMakeRect(12, y + 3, 160, 18));
        if (titleLabel) {
            *titleLabel = text;
        }
        return numberField(panel, NSMakeRect(174, y, 122, 24));
    };
    _target = field(@"Target / baseline", 284);
    _amplitude = field(@"Step / sine amplitude", 318);
    _frequency = field(@"Sine frequency (Hz)", 352);
    _start = field(@"Signal start time (s)", 386);
    NSTextField* limitLabel = nil;
    NSTextField* integralLabel = nil;
    _limit = field(@"Max torque (N·m)", 420, &limitLabel);
    _integralLimit = field(@"I term limit (N·m)", 454, &integralLabel);
    _limitLabel = limitLabel;
    _integralLimitLabel = integralLabel;
    _filter = field(@"D filter time (s)", 488);
    _deadband = field(@"Relay deadband", 522);
    _feedback =
        popup(@"Feedback", 556, @[ @"Ideal (true angle/speed)", @"Encoder sensor" ],
              @"What the controller measures: the exact joint value, or the joint's encoder with noise, "
              @"latency and quantization (see the Measured and True curves).");
    auto small = [&](NSString* title, CGFloat x, CGFloat y, NSString* tip) {
        label(panel, title, NSMakeRect(x, y + 3, 72, 18), 11);
        NSTextField* f = numberField(panel, NSMakeRect(x + 72, y, 70, 24));
        f.toolTip = tip;
        return f;
    };
    _encoderNoise = small(@"Noise σ", 12, 592, @"Encoder Gaussian noise (rad or m)");
    _encoderLatency = small(@"Latency", 158, 592, @"Delay in encoder samples");
    _encoderCounts =
        small(@"Counts", 12, 624, @"Counts per revolution (hinge) or per metre (slider); 0 = none");
    _encoderRate = small(@"Rate (Hz)", 158, 624, @"Encoder sample rate");
    NSButton* apply = pushButton(panel, @"Apply controller", NSMakeRect(10, 662, 290, 30), self,
                                 @selector(apply:), @"checkmark.circle");
    apply.keyEquivalent = @"\r";
    pushButton(panel, @"Run / Pause", NSMakeRect(10, 698, 142, 30), self, @selector(run:), @"playpause.fill");
    pushButton(panel, @"Reset run", NSMakeRect(158, 698, 142, 30), self, @selector(reset:),
               @"arrow.counterclockwise");
    pushButton(panel, @"Load PID step-response demo", NSMakeRect(10, 734, 290, 30), self, @selector(demo:),
               @"wand.and.stars");
    pushButton(panel, @"CSV…", NSMakeRect(10, 770, 90, 30), self, @selector(exportCsv:));
    pushButton(panel, @"PNG…", NSMakeRect(108, 770, 90, 30), self, @selector(exportPng:));
    pushButton(panel, @"Clear plot", NSMakeRect(206, 770, 94, 30), self, @selector(clearPlot:));
    _info = wrapping(panel,
                     @"Apply works while running and resets the I/D state. Reset run restarts the experiment "
                     @"with the new gains.",
                     NSMakeRect(12, 812, 287, 48));
    [_window center];
    return self;
}

- (int)jointIndex {
    return _joint.selectedItem ? static_cast<int>(_joint.selectedItem.tag) : -1;
}

- (void)show {
    [self refresh];
    for (NSMenuItem* item in _joint.itemArray) {
        if (item.tag >= 0 && _editor->_world.joints[item.tag].b == _editor->_selected) {
            [_joint selectItem:item];
            break;
        }
    }
    [self loadFields:nil];
    [_window makeKeyAndOrderFront:nil];
}

- (void)refresh {
    int previous = [self jointIndex];
    [_joint removeAllItems];
    for (int i = 0; i < static_cast<int>(_editor->_world.joints.size()); ++i) {
        const auto& j = _editor->_world.joints[i];
        if (!j.isActuated()) {
            continue;
        }
        [_joint addItemWithTitle:[NSString stringWithFormat:@"%s → %s (%@)",
                                                            _editor->_world.bodies[j.a].name.c_str(),
                                                            _editor->_world.bodies[j.b].name.c_str(),
                                                            jointName(j.type).lowercaseString]];
        _joint.lastItem.tag = i;
    }
    if (_joint.numberOfItems == 0) {
        [_joint addItemWithTitle:@"No hinge or slider joints in this robot"];
        _joint.lastItem.tag = -1;
        _joint.lastItem.enabled = NO;
    }
    [_joint selectItemWithTag:previous];
    [self loadFields:nil];
}

- (void)loadFields:(id)sender {
    (void)sender;
    int index = [self jointIndex];
    if (index < 0) {
        _plot.needsDisplay = YES;
        return;
    }
    const Joint& j = _editor->_world.joints[index];
    const auto& c = j.controller;
    bool linear = j.type == JointType::Prismatic;
    [_variable itemAtIndex:0].title = linear ? @"Position (m)" : @"Angle (position)";
    [_variable itemAtIndex:1].title = linear ? @"Speed (m/s)" : @"Speed (velocity)";
    _limitLabel.stringValue = linear ? @"Max force (N)" : @"Max torque (N·m)";
    _integralLimitLabel.stringValue = linear ? @"I term limit (N)" : @"I term limit (N·m)";
    [_kind selectItemAtIndex:static_cast<int>(c.type)];
    [_variable selectItemAtIndex:static_cast<int>(c.variable)];
    [_signal selectItemAtIndex:static_cast<int>(c.signal)];
    setVector(_gains, {c.kp, c.ki, c.kd});
    auto set = [](NSTextField* field, double value) {
        field.stringValue = [NSString stringWithFormat:@"%.9g", value];
    };
    set(_target, c.type == ControllerType::SpeedMotor ? j.motorSpeed : c.target);
    set(_amplitude, c.amplitude);
    set(_frequency, c.frequency);
    set(_start, c.startTime);
    set(_limit, j.maxTorque);
    set(_integralLimit, c.integralLimit);
    set(_filter, c.derivativeFilter);
    set(_deadband, c.deadband);
    int encoder = encoderFor(_editor->_world, index);
    Sensor shown = encoder >= 0 ? _editor->_world.sensors[static_cast<std::size_t>(encoder)]
                                : makeSensor(SensorType::JointEncoder, j.b);
    if (encoder < 0) { // Suggested settings for a first experiment.
        shown.noise.stddev = linear ? 0.002 : 0.01;
        shown.latency = 4;
        shown.encoderCounts = linear ? 2000 : 1024;
        shown.rate = 200;
    }
    [_feedback selectItemAtIndex:encoder >= 0 && shown.feedsController ? 1 : 0];
    set(_encoderNoise, shown.noise.stddev);
    set(_encoderLatency, shown.latency);
    set(_encoderCounts, shown.encoderCounts);
    set(_encoderRate, shown.rate);
    [self updateEnabled:nil];
    _plot.needsDisplay = YES;
}

- (void)updateEnabled:(id)sender {
    (void)sender;
    NSInteger type = _kind.indexOfSelectedItem;
    bool legacy = type == 0;
    bool relay = type == 5;
    bool hasI = type == 2 || type == 4;
    bool hasD = type == 3 || type == 4;
    _variable.enabled = !legacy;
    _signal.enabled = !legacy;
    _gains[0].enabled = !legacy && !relay;
    _gains[1].enabled = hasI;
    _gains[2].enabled = hasD;
    _integralLimit.enabled = hasI;
    _filter.enabled = hasD;
    _deadband.enabled = relay;
    _amplitude.enabled = !legacy && _signal.indexOfSelectedItem != 0;
    _start.enabled = _amplitude.enabled;
    _frequency.enabled = !legacy && _signal.indexOfSelectedItem == 2;
    int index = [self jointIndex];
    bool linear = index >= 0 && index < static_cast<int>(_editor->_world.joints.size())
                  && _editor->_world.joints[index].type == JointType::Prismatic;
    bool speed = legacy || _variable.indexOfSelectedItem == 1;
    NSString* unit = linear ? (speed ? @"m/s" : @"m") : (speed ? @"rad/s" : @"rad");
    _units.stringValue = [NSString stringWithFormat:@"Target, amplitude and deadband are in %@", unit];
    bool sensor = !legacy && _feedback.indexOfSelectedItem == 1;
    _feedback.enabled = !legacy;
    for (NSTextField* f in @[ _encoderNoise, _encoderLatency, _encoderCounts, _encoderRate ]) {
        f.enabled = sensor;
    }
}

- (void)apply:(id)sender {
    (void)sender;
    int index = [self jointIndex];
    if (index < 0) {
        [_editor showError:@"Create a hinge or slider joint first (Inspector → Connection to parent)."];
        return;
    }
    try {
        ControllerConfig config;
        config.type = static_cast<ControllerType>(_kind.indexOfSelectedItem);
        config.variable = static_cast<ControlVariable>(_variable.indexOfSelectedItem);
        config.signal = static_cast<ReferenceSignal>(_signal.indexOfSelectedItem);
        Vec3 gains = getVector(_gains);
        config.kp = gains.x;
        config.ki = gains.y;
        config.kd = gains.z;
        config.target = number(_target);
        config.amplitude = number(_amplitude);
        config.frequency = number(_frequency);
        config.startTime = number(_start);
        config.integralLimit = number(_integralLimit);
        config.derivativeFilter = number(_filter);
        config.deadband = number(_deadband);
        config.validate();
        bool useSensor = config.type != ControllerType::SpeedMotor && _feedback.indexOfSelectedItem == 1;
        double encoderNoise = number(_encoderNoise);
        double encoderLatency = number(_encoderLatency);
        double encoderCounts = number(_encoderCounts);
        double encoderRate = number(_encoderRate);
        bool hasEncoder = encoderFor(_editor->_world, index) >= 0;
        if (useSensor || hasEncoder) {
            Sensor probe = makeSensor(SensorType::JointEncoder, 0);
            probe.noise.stddev = encoderNoise;
            probe.encoderCounts = encoderCounts;
            probe.rate = encoderRate;
            if (encoderLatency != std::round(encoderLatency)
                || std::abs(encoderLatency) > Sensor::maxLatency) {
                throw std::runtime_error("Latency must be a whole number of samples (0..1000)");
            }
            probe.latency = static_cast<int>(encoderLatency);
            validateSensor(probe, 1);
        }
        double limit = number(_limit);
        if (limit < 0) {
            throw std::runtime_error("Maximum torque or force must be zero or positive");
        }
        Joint& joint = _editor->_world.joints[index];
        bool wasPosition = joint.controller.type != ControllerType::SpeedMotor
                           && joint.controller.variable == ControlVariable::Position;
        bool isPosition =
            config.type != ControllerType::SpeedMotor && config.variable == ControlVariable::Position;
        if (wasPosition != isPosition) {
            joint.history.clear(); // Do not mix positions and speeds on one axis.
        }
        joint.controller = config;
        joint.maxTorque = limit;
        joint.controllerState = {};
        if (config.type == ControllerType::SpeedMotor) {
            joint.motorSpeed = config.target;
        }
        size_t sensorsBefore = _editor->_world.sensors.size();
        int latency = static_cast<int>(encoderLatency);
        if (useSensor || hasEncoder) {
            applyFeedback(_editor->_world, index, useSensor, encoderNoise, latency, encoderCounts,
                          encoderRate);
        }
        if (_editor->_hasSnapshot && (useSensor || encoderFor(_editor->_snapshot, index) >= 0)) {
            applyFeedback(_editor->_snapshot, index, useSensor, encoderNoise, latency, encoderCounts,
                          encoderRate);
        }
        if (_editor->_hasSnapshot) {
            auto& initial = _editor->_snapshot.joints[index];
            initial.controller = config;
            initial.maxTorque = limit;
            initial.motorSpeed = joint.motorSpeed;
            initial.controllerState = {};
            initial.history.clear();
        }
        _editor->_window.documentEdited = YES;
        if (!_editor->_running) {
            [_editor refresh];
        } else if (_editor->_world.sensors.size() != sensorsBefore) {
            [_editor->_sensorPanel refresh];
        }
        _info.stringValue = @"Applied. I/D state was reset; the reference clock continues. Reset run starts "
                            @"the experiment from zero.";
        _plot.needsDisplay = YES;
    } catch (const std::exception& e) {
        [_editor showError:[NSString stringWithUTF8String:e.what()]];
    }
}

- (void)demo:(id)sender {
    (void)sender;
    [_editor loadWorld:makeControlDemo() selected:1 name:@"Load PID Demo"];
}

- (void)run:(id)sender {
    [_editor toggleRun:sender];
}

- (void)reset:(id)sender {
    [_editor reset:sender];
}

- (void)clearPlot:(id)sender {
    (void)sender;
    int index = [self jointIndex];
    if (index >= 0) {
        _editor->_world.joints[index].history.clear();
    }
    _plot.needsDisplay = YES;
}

- (void)exportCsv:(id)sender {
    (void)sender;
    int index = [self jointIndex];
    if (index < 0) {
        return;
    }
    // Capture before a modal save panel so an ongoing run cannot change the export.
    auto history = _editor->_world.joints[index].history;
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"joint-response.csv";
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    try {
        std::ofstream out(panel.URL.fileSystemRepresentation);
        if (!out) {
            throw std::runtime_error("Cannot create CSV file");
        }
        writeControlCsvHeader(out);
        for (const auto& s : history) {
            writeControlCsvRow(out, index, s);
        }
        out.close();
        if (!out) {
            throw std::runtime_error("Failed to write CSV");
        }
    } catch (const std::exception& e) {
        [_editor showError:[NSString stringWithUTF8String:e.what()]];
    }
}

- (void)exportPng:(id)sender {
    (void)sender;
    NSBitmapImageRep* bitmap = [_plot bitmapImageRepForCachingDisplayInRect:_plot.bounds];
    [_window.effectiveAppearance performAsCurrentDrawingAppearance:^{
        [_plot cacheDisplayInRect:_plot.bounds toBitmapImageRep:bitmap];
    }];
    NSData* png = [bitmap representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"joint-response.png";
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    if (![png writeToURL:panel.URL atomically:YES]) {
        [_editor showError:@"Could not save the plot image."];
    }
}

- (void)tick {
    if (_window.visible) {
        _plot.needsDisplay = YES;
    }
}

@end
