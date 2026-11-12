// Sensors window: live readings, channel plot / lidar scan / camera images, CSV recording and export.

#import "mac/sensor_panel.h"
#import "mac/editor_controller.h"

#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

using namespace robotics;
using namespace editor_ui;

@interface SensorPlotView : NSView
@property (nonatomic, weak) SensorPanel* panel;
@end

namespace {

void drawText(NSString* text, NSPoint position, NSColor* color, CGFloat size = 11) {
    [text drawAtPoint:position
        withAttributes:@{
            NSFontAttributeName : [NSFont monospacedDigitSystemFontOfSize:size weight:NSFontWeightRegular],
            NSForegroundColorAttributeName : color
        }];
}

NSImage* imageFromRgba(const std::vector<unsigned char>& rgba, int width, int height) {
    if (rgba.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4) {
        return nil;
    }
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

// Line plot of one history channel.
void drawHistory(NSRect area, const Sensor& sensor, int channel, NSString* unit) {
    [[NSColor colorWithCalibratedWhite:0.07 alpha:1] setFill];
    NSRectFill(area);
    const auto& history = sensor.state.history;
    if (history.empty() || channel < 0) {
        drawText(@"Press Run to collect samples", NSMakePoint(area.origin.x + 20, area.origin.y + 20),
                 NSColor.secondaryLabelColor);
        return;
    }
    auto c = static_cast<std::size_t>(channel);
    double lo = 1e300;
    double hi = -1e300;
    for (const auto& sample : history) {
        if (c < sample.values.size()) {
            lo = std::min(lo, sample.values[c]);
            hi = std::max(hi, sample.values[c]);
        }
    }
    if (lo > hi) {
        return;
    }
    double pad = std::max(1e-6, (hi - lo) * 0.08);
    lo -= pad;
    hi += pad;
    double t0 = history.front().time;
    double t1 = std::max(t0 + 1e-3, history.back().time);
    NSRect plot =
        NSMakeRect(area.origin.x + 64, area.origin.y + 8, area.size.width - 76, area.size.height - 34);
    auto point = [&](double t, double v) {
        return NSMakePoint(plot.origin.x + (t - t0) / (t1 - t0) * plot.size.width,
                           NSMaxY(plot) - (v - lo) / (hi - lo) * plot.size.height);
    };
    for (int k = 0; k <= 4; ++k) {
        double v = lo + (hi - lo) * k / 4;
        [NSColor.quaternaryLabelColor setStroke];
        NSBezierPath* grid = [NSBezierPath bezierPath];
        [grid moveToPoint:point(t0, v)];
        [grid lineToPoint:point(t1, v)];
        [grid stroke];
        drawText([NSString stringWithFormat:@"%.4g", v], NSMakePoint(area.origin.x + 4, point(t0, v).y - 7),
                 NSColor.secondaryLabelColor, 10);
        double t = t0 + (t1 - t0) * k / 4;
        drawText([NSString stringWithFormat:@"%.2f s", t], NSMakePoint(point(t, lo).x - 16, NSMaxY(plot) + 6),
                 NSColor.secondaryLabelColor, 10);
    }
    drawText(unit, NSMakePoint(area.origin.x + 4, area.origin.y - 2), NSColor.secondaryLabelColor, 10);
    [NSGraphicsContext saveGraphicsState];
    NSRectClip(plot);
    [NSColor.systemCyanColor setStroke];
    NSBezierPath* path = [NSBezierPath bezierPath];
    path.lineWidth = 1.5;
    bool first = true;
    for (const auto& sample : history) {
        if (c >= sample.values.size()) {
            continue;
        }
        NSPoint p = point(sample.time, sample.values[c]);
        if (first) {
            [path moveToPoint:p];
        } else {
            [path lineToPoint:p];
        }
        first = false;
    }
    [path stroke];
    [NSGraphicsContext restoreGraphicsState];
}

// Top-down polar plot of the latest lidar scan (sensor +X points up in the plot).
void drawScan(NSRect area, const Sensor& sensor) {
    [[NSColor colorWithCalibratedWhite:0.07 alpha:1] setFill];
    NSRectFill(area);
    NSPoint centre = NSMakePoint(NSMidX(area), NSMidY(area));
    double scale = std::min(area.size.width, area.size.height) * 0.46 / sensor.maxRange;
    [NSColor.quaternaryLabelColor setStroke];
    for (int ring = 1; ring <= 4; ++ring) {
        double r = sensor.maxRange * ring / 4 * scale;
        [[NSBezierPath bezierPathWithOvalInRect:NSMakeRect(centre.x - r, centre.y - r, 2 * r, 2 * r)] stroke];
        drawText([NSString stringWithFormat:@"%.1f m", sensor.maxRange * ring / 4],
                 NSMakePoint(centre.x + 3, centre.y - r - 1), NSColor.tertiaryLabelColor, 9);
    }
    const auto& value = sensor.state.value;
    if (!sensor.state.valid) {
        drawText(@"No scan yet - press Run", NSMakePoint(area.origin.x + 20, area.origin.y + 20),
                 NSColor.secondaryLabelColor);
        return;
    }
    for (std::size_t k = 0; k < value.size(); ++k) {
        Vec3 d = lidarBeamDirection(sensor, static_cast<int>(k));
        double r = value[k] * scale;
        // Flipped view: plot up = sensor +X, plot right = sensor +Z (right).
        NSPoint p = NSMakePoint(centre.x + d.z * r, centre.y - d.x * r);
        bool hit = value[k] < sensor.maxRange - 1e-9;
        [(hit ? NSColor.systemRedColor : NSColor.systemGreenColor) setFill];
        NSRectFill(NSMakeRect(p.x - 1.5, p.y - 1.5, 3, 3));
    }
    [NSColor.systemYellowColor setFill];
    NSRectFill(NSMakeRect(centre.x - 3, centre.y - 3, 6, 6));
}

} // namespace

@implementation SensorPlotView

- (BOOL)isFlipped {
    return YES;
}

- (void)drawRect:(NSRect)dirtyRect {
    (void)dirtyRect;
    [[NSColor colorWithCalibratedRed:0.055 green:0.075 blue:0.105 alpha:1] setFill];
    NSRectFill(self.bounds);
    SensorPanel* panel = self.panel;
    EditorController* editor = panel ? panel->_editor : nil;
    int index = panel ? [panel sensorIndex] : -1;
    if (!editor || index < 0 || index >= static_cast<int>(editor->_world.sensors.size())) {
        drawText(@"No sensors in this robot.", NSMakePoint(24, 24), NSColor.labelColor, 14);
        drawText(
            @"Select a part and use Inspector > Sensors > + Add sensor, or load Examples > Sensor Rover.",
            NSMakePoint(24, 48), NSColor.secondaryLabelColor, 12);
        return;
    }
    const Sensor& sensor = editor->_world.sensors[static_cast<std::size_t>(index)];
    NSRect area = NSInsetRect(self.bounds, 14, 14);
    area.origin.y += 22;
    area.size.height -= 22;
    if (sensor.type == SensorType::Camera) {
        drawText(@"Camera: color (left) and depth (right, near = bright)", NSMakePoint(14, 10),
                 NSColor.labelColor, 12);
        if (editor->_cameraSensor == index) {
            NSImage* color = imageFromRgba(editor->_cameraColor, sensor.width, sensor.height);
            std::vector<unsigned char> gray(editor->_cameraDepth.size() * 4);
            for (std::size_t i = 0; i < editor->_cameraDepth.size(); ++i) {
                auto v = static_cast<unsigned char>(
                    255 * std::clamp(1 - editor->_cameraDepth[i] / sensor.maxRange, 0.0, 1.0));
                gray[i * 4] = gray[i * 4 + 1] = gray[i * 4 + 2] = v;
                gray[i * 4 + 3] = 255;
            }
            NSImage* depth = imageFromRgba(gray, sensor.width, sensor.height);
            CGFloat half = (area.size.width - 10) / 2;
            CGFloat h = std::min<CGFloat>(area.size.height, half * sensor.height / sensor.width);
            [color drawInRect:NSMakeRect(area.origin.x, area.origin.y, half, h)
                      fromRect:NSZeroRect
                     operation:NSCompositingOperationSourceOver
                      fraction:1
                respectFlipped:YES
                         hints:@{NSImageHintInterpolation : @(NSImageInterpolationNone)}];
            [depth drawInRect:NSMakeRect(area.origin.x + half + 10, area.origin.y, half, h)
                      fromRect:NSZeroRect
                     operation:NSCompositingOperationSourceOver
                      fraction:1
                respectFlipped:YES
                         hints:@{NSImageHintInterpolation : @(NSImageInterpolationNone)}];
        } else {
            drawText(
                @"The camera renders while the simulation runs (or when it is the selected part's camera).",
                NSMakePoint(14, 40), NSColor.secondaryLabelColor);
        }
        return;
    }
    if (sensor.type == SensorType::Lidar2D) {
        drawText(@"Latest scan, top view (sensor forward = up). Red: hits, green: no return.",
                 NSMakePoint(14, 10), NSColor.labelColor, 12);
        drawScan(area, sensor);
        return;
    }
    int channel = [panel channelIndex];
    std::vector<std::string> names = sensorChannelNames(editor->_world, sensor);
    NSString* name =
        channel >= 0 && channel < static_cast<int>(names.size()) ? @(names[channel].c_str()) : @"";
    drawText([NSString stringWithFormat:@"%s · %@ · last %zu samples", sensor.name.c_str(), name,
                                        sensor.state.history.size()],
             NSMakePoint(14, 10), NSColor.labelColor, 12);
    std::string unit =
        channel >= 0 ? sensorChannelUnit(editor->_world, sensor, static_cast<std::size_t>(channel)) : "";
    drawHistory(area, sensor, channel, @(unit.c_str()));
}

@end

@implementation SensorPanel

- (instancetype)initWithEditor:(EditorController*)editor {
    self = [super init];
    if (!self) {
        return nil;
    }
    _editor = editor;
    _window =
        [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 980, 600)
                                    styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                              | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable
                                      backing:NSBackingStoreBuffered
                                        defer:NO];
    _window.title = @"Sensors — live readings";
    _window.releasedWhenClosed = NO;
    _window.minSize = NSMakeSize(820, 520);
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    FlippedView* side = [[FlippedView alloc] initWithFrame:NSMakeRect(0, 0, 320, 600)];
    side.autoresizingMask = NSViewHeightSizable;
    [_window.contentView addSubview:side];
    _plot = [[SensorPlotView alloc] initWithFrame:NSMakeRect(320, 0, 660, 600)];
    _plot.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    _plot.panel = self;
    [_window.contentView addSubview:_plot];
    label(side, @"Sensor", NSMakeRect(12, 14, 290, 18), 13, NSFontWeightSemibold);
    _sensor = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(10, 36, 300, 26) pullsDown:NO];
    _sensor.target = self;
    _sensor.action = @selector(sensorChanged:);
    [side addSubview:_sensor];
    label(side, @"Plot channel", NSMakeRect(12, 72, 120, 18));
    _channel = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(110, 68, 200, 26) pullsDown:NO];
    _channel.target = self;
    _channel.action = @selector(channelChanged:);
    [side addSubview:_channel];
    _readings = [NSTextField wrappingLabelWithString:@""];
    _readings.frame = NSMakeRect(12, 104, 298, 360);
    _readings.font = [NSFont monospacedDigitSystemFontOfSize:11 weight:NSFontWeightRegular];
    _readings.textColor = NSColor.labelColor;
    [side addSubview:_readings];
    _record = pushButton(side, @"Record CSV…", NSMakeRect(10, 476, 148, 30), self, @selector(toggleRecord:),
                         @"record.circle");
    _record.toolTip = @"Write every sensor sample to a CSV file while the simulation runs.";
    NSButton* exportButton = pushButton(side, @"Export history…", NSMakeRect(162, 476, 148, 30), self,
                                        @selector(exportCsv:), @"square.and.arrow.up");
    exportButton.toolTip = @"Save the plotted history (last samples of every sensor) as CSV.";
    wrapping(side,
             @"Values are what a controller would see: noise, bias, quantization, rate (sample-and-hold), "
             @"latency and dropouts applied. 'truth' is the ideal value of the newest sample.",
             NSMakeRect(12, 514, 298, 70), 11);
    [_window center];
    return self;
}

- (int)sensorIndex {
    return _sensor.selectedItem ? static_cast<int>(_sensor.selectedItem.tag) : -1;
}

- (int)channelIndex {
    return _channel.selectedItem ? static_cast<int>(_channel.selectedItem.tag) : -1;
}

- (void)show {
    [self refresh];
    // Prefer a sensor of the selected part.
    for (NSMenuItem* item in _sensor.itemArray) {
        if (item.tag >= 0
            && _editor->_world.sensors[static_cast<std::size_t>(item.tag)].body == _editor->_selected) {
            [_sensor selectItem:item];
            [self sensorChanged:nil];
            break;
        }
    }
    [_window makeKeyAndOrderFront:nil];
}

- (void)refresh {
    int previous = [self sensorIndex];
    [_sensor removeAllItems];
    const World& world = _editor->_world;
    for (std::size_t i = 0; i < world.sensors.size(); ++i) {
        const Sensor& s = world.sensors[i];
        std::string body = s.body >= 0 && s.body < static_cast<int>(world.bodies.size())
                               ? world.bodies[static_cast<std::size_t>(s.body)].name
                               : "?";
        [_sensor addItemWithTitle:[NSString stringWithFormat:@"%s (%s on %s)", s.name.c_str(),
                                                             std::string(sensorTypeName(s.type)).c_str(),
                                                             body.c_str()]];
        _sensor.lastItem.tag = static_cast<NSInteger>(i);
    }
    if (_sensor.numberOfItems == 0) {
        [_sensor addItemWithTitle:@"No sensors"];
        _sensor.lastItem.tag = -1;
    }
    if (![_sensor selectItemWithTag:previous]) {
        [_sensor selectItemAtIndex:0];
    }
    [self sensorChanged:nil];
}

- (void)sensorChanged:(id)sender {
    (void)sender;
    int previous = [self channelIndex];
    [_channel removeAllItems];
    int index = [self sensorIndex];
    if (index >= 0 && index < static_cast<int>(_editor->_world.sensors.size())) {
        const Sensor& sensor = _editor->_world.sensors[static_cast<std::size_t>(index)];
        std::vector<std::string> names = sensorChannelNames(_editor->_world, sensor);
        for (std::size_t c = 0; c < names.size() && c < Sensor::historyChannels; ++c) {
            [_channel addItemWithTitle:@(names[c].c_str())];
            _channel.lastItem.tag = static_cast<NSInteger>(c);
        }
    }
    _channel.enabled = _channel.numberOfItems > 0;
    if (sender == nil && previous >= 0) {
        [_channel selectItemWithTag:previous];
    }
    [self tick];
}

- (void)channelChanged:(id)sender {
    (void)sender;
    _plot.needsDisplay = YES;
}

- (void)tick {
    if (!_window.visible && !isEditorSmokeTest()) {
        return;
    }
    const World& world = _editor->_world;
    int index = [self sensorIndex];
    _record.title = _editor->_sensorLog ? @"Stop recording" : @"Record CSV…";
    if (index < 0 || index >= static_cast<int>(world.sensors.size())) {
        _readings.stringValue = @"";
        _plot.needsDisplay = YES;
        return;
    }
    const Sensor& sensor = world.sensors[static_cast<std::size_t>(index)];
    const auto& state = sensor.state;
    NSMutableString* text = [NSMutableString string];
    [text appendFormat:@"%s  ·  %@\n", std::string(sensorTypeName(sensor.type)).c_str(),
                       sensor.rate > 0 ? [NSString stringWithFormat:@"%.4g Hz", sensor.rate] : @"every step"];
    [text appendFormat:@"samples %llu   dropped %llu\n", static_cast<unsigned long long>(state.samples),
                       static_cast<unsigned long long>(state.dropped)];
    if (sensor.isJointSensor() && sensorJoint(world, sensor) < 0) {
        [text appendString:@"\nNo joint: attach this part to a parent first.\n"];
    }
    if (!state.valid) {
        [text appendString:@"\nNo reading yet. Press Run (Space)."];
    } else if (sensor.type == SensorType::Camera) {
        if (_editor->_cameraSensor == index && !_editor->_cameraDepth.empty()) {
            std::size_t centre =
                static_cast<std::size_t>(sensor.height / 2) * static_cast<std::size_t>(sensor.width)
                + static_cast<std::size_t>(sensor.width / 2);
            [text appendFormat:@"\n%d x %d px, FOV %.0f°\ncentre depth  %.3f m\n", sensor.width,
                               sensor.height, sensor.fov * 180 / robotics::pi, _editor->_cameraDepth[centre]];
        }
    } else if (sensor.type == SensorType::Lidar2D) {
        double nearest = sensor.maxRange;
        int beam = -1;
        int hits = 0;
        for (std::size_t k = 0; k < state.value.size(); ++k) {
            if (state.value[k] < sensor.maxRange - 1e-9) {
                ++hits;
            }
            if (state.value[k] < nearest) {
                nearest = state.value[k];
                beam = static_cast<int>(k);
            }
        }
        [text appendFormat:@"\n%zu beams, %d returns\n", state.value.size(), hits];
        if (beam >= 0) {
            Vec3 d = lidarBeamDirection(sensor, beam);
            [text appendFormat:@"nearest  %.3f m at %+.1f°\n", nearest,
                               std::atan2(-d.z, d.x) * 180 / robotics::pi];
        }
    } else {
        std::vector<std::string> names = sensorChannelNames(world, sensor);
        [text appendFormat:@"\n%-13s %11s %11s\n", "channel", "value", "truth"];
        for (std::size_t c = 0; c < state.value.size() && c < names.size(); ++c) {
            std::string unit = sensorChannelUnit(world, sensor, c);
            [text appendFormat:@"%-13s %11.5f %11.5f %s\n", names[c].c_str(), state.value[c],
                               c < state.truth.size() ? state.truth[c] : 0.0, unit.c_str()];
        }
        if (sensor.type == SensorType::JointEncoder) {
            [text appendFormat:@"%-13s %11.5f\n", "speed_est", state.speedEstimate];
        }
        if (sensor.feedsController) {
            [text appendString:@"\nFeeds the joint's controller."];
        }
    }
    _readings.stringValue = text;
    _plot.needsDisplay = YES;
}

- (void)toggleRecord:(id)sender {
    (void)sender;
    if (_editor->_sensorLog) {
        [_editor stopSensorLog];
        [self tick];
        return;
    }
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"sensors.csv";
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    if (![_editor startSensorLog:@(panel.URL.fileSystemRepresentation)]) {
        [_editor showError:@"Cannot create the CSV file."];
    }
    [self tick];
}

- (BOOL)exportHistory:(NSString*)path {
    std::ofstream out(path.fileSystemRepresentation);
    if (!out) {
        return NO;
    }
    out.precision(12);
    out << "time,sensor,name,type,channel,value\n";
    const World& world = _editor->_world;
    for (std::size_t i = 0; i < world.sensors.size(); ++i) {
        const Sensor& sensor = world.sensors[i];
        std::vector<std::string> names = sensorChannelNames(world, sensor);
        for (const auto& sample : sensor.state.history) {
            for (std::size_t c = 0; c < sample.values.size() && c < names.size(); ++c) {
                out << sample.time << ',' << i << ",\"" << sensor.name << "\"," << sensorTypeName(sensor.type)
                    << ',' << names[c] << ',' << sample.values[c] << '\n';
            }
        }
    }
    out.close();
    return static_cast<bool>(out);
}

- (void)exportCsv:(id)sender {
    (void)sender;
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"sensor-history.csv";
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    if (![self exportHistory:@(panel.URL.fileSystemRepresentation)]) {
        [_editor showError:@"Could not write the sensor CSV."];
    }
}

@end
