// EditorController (SmokeTestUX): gizmos, timeline, plots and part library, part of --smoke-test.
//
// Mouse and keyboard go through -[NSWindow sendEvent:] with synthesized NSEvents, so window hit testing,
// the overlays and the real handlers are exercised. The simulation advances through -tick:.

#import "mac/editor_controller.h"
#import "mac/library_panel.h"
#import "mac/plot_window.h"
#import "mac/timeline_bar.h"

#include "robotics/demo_scenes.hpp"
#include "robotics/editing/assembly.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace robotics;
using namespace editor_ui;

namespace {

void check(bool ok, const std::string& message) {
    if (!ok) {
        throw std::runtime_error(message);
    }
}

NSEvent* mouseEvent(NSWindow* window, NSEventType type, NSPoint windowPoint, NSEventModifierFlags flags) {
    return [NSEvent mouseEventWithType:type
                              location:windowPoint
                         modifierFlags:flags
                             timestamp:NSProcessInfo.processInfo.systemUptime
                          windowNumber:window.windowNumber
                               context:nil
                           eventNumber:0
                            clickCount:1
                              pressure:1];
}

BOOL writeWindowImage(NSWindow* window, NSString* path) {
    NSView* content = window.contentView.superview; // The frame view also paints the window background.
    [content display];
    NSBitmapImageRep* bitmap = [content bitmapImageRepForCachingDisplayInRect:content.bounds];
    [window.effectiveAppearance performAsCurrentDrawingAppearance:^{
        [content cacheDisplayInRect:content.bounds toBitmapImageRep:bitmap];
    }];
    return [[bitmap representationUsingType:NSBitmapImageFileTypePNG properties:@{}] writeToFile:path
                                                                                      atomically:YES];
}

std::size_t csvRows(const char* path, std::string* header = nullptr) {
    std::ifstream in(path);
    std::string line;
    std::size_t rows = 0;
    while (std::getline(in, line)) {
        if (rows == 0 && header) {
            *header = line;
        }
        ++rows;
    }
    return rows > 0 ? rows - 1 : 0;
}

} // namespace

@implementation EditorController (SmokeTestUX)

// Scene-view coordinates of a world point.
- (NSPoint)viewPoint:(Vec3)world {
    GizmoPoint p;
    check([_scene gizmoView].project(world, p), "Point behind the camera");
    return NSMakePoint(p.x, p.y);
}

- (void)send:(NSEventType)type at:(NSPoint)viewPoint flags:(NSEventModifierFlags)flags {
    [_window sendEvent:mouseEvent(_window, type, [_scene convertPoint:viewPoint toView:nil], flags)];
}

// A whole drag: press, moves along the path in `steps`, release.
- (void)dragFrom:(NSPoint)from to:(NSPoint)to flags:(NSEventModifierFlags)flags {
    [self send:NSEventTypeLeftMouseDown at:from flags:flags];
    constexpr int steps = 12;
    for (int k = 1; k <= steps; ++k) {
        double t = static_cast<double>(k) / steps;
        [self send:NSEventTypeLeftMouseDragged
                at:NSMakePoint(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
             flags:flags];
    }
    [self send:NSEventTypeLeftMouseUp at:to flags:flags];
}

- (void)pressKey:(unsigned short)code {
    NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                      location:NSZeroPoint
                                 modifierFlags:0
                                     timestamp:NSProcessInfo.processInfo.systemUptime
                                  windowNumber:_window.windowNumber
                                       context:nil
                                    characters:@""
                   charactersIgnoringModifiers:@""
                                     isARepeat:NO
                                       keyCode:code];
    [_window sendEvent:event];
}

- (void)loadForTest:(World)world selected:(int)selected {
    [self loadWorld:std::move(world) selected:selected name:@"Test Scene"];
    [_window makeKeyAndOrderFront:nil];
    [_window makeFirstResponder:_scene];
}

// Advances the simulation through the real run loop callback: every call is 24 physics steps.
- (void)runTicks:(int)count {
    for (int k = 0; k < count; ++k) {
        _previous = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
        [self tick:nil];
    }
}

- (void)writeScenePreview:(NSString*)path {
    NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:path atomically:YES], "Could not write preview");
}

- (void)checkMoveGizmo {
    [self loadForTest:makeRoverDemo() selected:0];
    [_scene viewPreset:0];
    [self pressKey:19]; // Key 2.
    check(_scene.gizmoTool == GizmoTool::Move, "Key 2 must choose the Move tool");
    _scene.moveSnap = 0;
    Vec3 start = _world.bodies[0].position;
    Vec3 wheel = _world.bodies[1].position;
    double unit = [_scene gizmoView].worldPerPoint(start);

    // Hover highlight through the real handler.
    NSPoint handle = [self viewPoint:start + Vec3{80 * unit, 0, 0}];
    [_scene
        mouseMoved:mouseEvent(_window, NSEventTypeMouseMoved, [_scene convertPoint:handle toView:nil], 0)];
    check(_scene.hoveredHandle == GizmoHandle::AxisX, "Hovering the X arrow must highlight it");

    // Drag the X arrow. Mouse position on the arrow, so the part must follow in X only.
    NSPoint target = [self viewPoint:start + Vec3{80 * unit + 0.5, 0, 0}];
    [self dragFrom:handle to:target flags:0];
    Vec3 moved = _world.bodies[0].position;
    check(std::abs(moved.x - start.x - 0.5) < 2e-3,
          "X arrow drag did not follow the mouse: " + std::to_string(moved.x - start.x));
    check(moved.y == start.y && moved.z == start.z, "X arrow drag moved another axis");
    check((_world.bodies[1].position - wheel - (moved - start)).length() < 1e-9,
          "Attached wheel did not follow");
    check(_world.maxAnchorError() < 1e-9, "Move broke a joint");
    check(_selected == 0, "A handle drag must keep the selection");
    // One undo step per drag.
    [_undo undo];
    check((_world.bodies[0].position - start).length() < 1e-12, "One undo must revert the whole drag");

    // A click on a handle changes nothing and leaves no undo step behind.
    Vec3 before = _world.bodies[0].position;
    [self send:NSEventTypeLeftMouseDown at:handle flags:0];
    [self send:NSEventTypeLeftMouseUp at:handle flags:0];
    check((_world.bodies[0].position - before).length() == 0 && _selected == 0,
          "Click on a handle moved the part");

    // Snapping to 25 cm, and ⌘ inverting it. (The camera keeps its place after a drag, so handles are
    // located again after every undo.)
    auto arrow = [&](double offset) {
        return [self viewPoint:start + Vec3{80 * unit + offset, 0, 0}];
    };
    _scene.moveSnap = 0.25;
    [self dragFrom:arrow(0) to:arrow(0.5) flags:0];
    double x = _world.bodies[0].position.x;
    check(std::abs(x - snapToStep(x, 0.25)) < 1e-9 && std::abs(x) > 0.2,
          "Drag did not land on the 25 cm grid");
    [_undo undo];
    [self dragFrom:arrow(0) to:arrow(0.53) flags:NSEventModifierFlagCommand];
    x = _world.bodies[0].position.x;
    check(std::abs(x - snapToStep(x, 0.25)) > 1e-3, "⌘ must turn snapping off during the drag");
    [_undo undo];
    _scene.moveSnap = 0;
    [self dragFrom:arrow(0) to:arrow(0.53) flags:NSEventModifierFlagCommand];
    x = _world.bodies[0].position.x;
    check(std::abs(x - snapToStep(x, 0.05)) < 1e-9 && std::abs(x) > 0.4,
          "⌘ must turn snapping on when it is off");
    [_undo undo];

    // ⌘ pressed while holding still re-evaluates the drag at once (flagsChanged).
    _scene.moveSnap = 0.25;
    [self send:NSEventTypeLeftMouseDown at:arrow(0) flags:0];
    [self send:NSEventTypeLeftMouseDragged at:arrow(0.53) flags:0];
    check(std::abs(_world.bodies[0].position.x - 0.5) < 1e-9, "Snapped before ⌘");
    [_window sendEvent:[NSEvent keyEventWithType:NSEventTypeFlagsChanged
                                              location:NSZeroPoint
                                         modifierFlags:NSEventModifierFlagCommand
                                             timestamp:NSProcessInfo.processInfo.systemUptime
                                          windowNumber:_window.windowNumber
                                               context:nil
                                            characters:@""
                           charactersIgnoringModifiers:@""
                                             isARepeat:NO
                                               keyCode:55]];
    check(std::abs(_world.bodies[0].position.x - 0.53) < 2e-3,
          "⌘ must release snapping without moving the mouse");
    [self send:NSEventTypeLeftMouseUp at:arrow(0.53) flags:NSEventModifierFlagCommand];
    [_undo undo];
    _scene.moveSnap = 0;

    // Undo keeps the view still, so handles stay where they were on screen.
    Vec3 cameraBefore = [_scene cameraTarget];
    [self dragFrom:arrow(0) to:arrow(0.5) flags:0];
    [_undo undo];
    check(([_scene cameraTarget] - cameraBefore).length() < 1e-9, "Drag and undo must not move the camera");

    // Right-drag pans; it must not open the context menu.
    NSPoint empty = NSMakePoint(40, 300);
    [self send:NSEventTypeRightMouseDown at:empty flags:0];
    [self send:NSEventTypeRightMouseDragged at:NSMakePoint(80, 320) flags:0];
    [self send:NSEventTypeRightMouseUp at:NSMakePoint(80, 320) flags:0];
    check(([_scene cameraTarget] - cameraBefore).length() > 1e-3, "Right-drag must still pan");

    // Plane handle: XZ square moves in X and Z, keeps Y.
    NSPoint square = [self viewPoint:start + Vec3{43 * unit, 0, 43 * unit}];
    NSPoint away = [self viewPoint:start + Vec3{43 * unit + 0.4, 0, 43 * unit - 0.3}];
    [self dragFrom:square to:away flags:0];
    Vec3 planar = _world.bodies[0].position - start;
    check(std::abs(planar.x - 0.4) < 2e-3 && std::abs(planar.z + 0.3) < 2e-3 && planar.y == 0,
          "Plane drag wrong");
    [_undo undo];

    // Preview in the middle of a drag.
    _scene.moveSnap = 0.05;
    handle = arrow(0);
    target = arrow(0.5);
    [self send:NSEventTypeLeftMouseDown at:handle flags:0];
    [self send:NSEventTypeLeftMouseDragged at:target flags:0];
    check([_scene.gizmoReadout containsString:@"snap 5 cm"], "Readout must show the move and the snap step");
    [self writeScenePreview:@"build/gizmo-move-preview.png"];
    [self send:NSEventTypeLeftMouseUp at:target flags:0];
    check(_scene.gizmoReadout == nil, "Readout must hide after the drag");
    [_undo undo];

    // Empty space still orbits and a drag off the handles does not edit.
    Vec3 kept = _world.bodies[0].position;
    [self dragFrom:NSMakePoint(30, 200) to:NSMakePoint(90, 240) flags:0];
    check((_world.bodies[0].position - kept).length() == 0, "Orbiting moved the part");
    _scene.moveSnap = 0;

    // No handles while the simulation runs.
    [self toggleRun:nil];
    check(![self sceneGizmoAvailable], "Gizmo must hide while running");
    Vec3 running = _world.bodies[0].position;
    [_window makeFirstResponder:_scene];
    NSPoint here = [self viewPoint:running + Vec3{80 * unit, 0, 0}];
    [self dragFrom:here to:NSMakePoint(here.x + 60, here.y) flags:0];
    check((_world.bodies[0].position - running).length() < 1e-9, "Handle worked while running");
    [self toggleRun:nil];
    [self reset:nil];
}

- (void)checkRotateGizmo {
    [self loadForTest:makeRoverDemo() selected:0];
    [_scene viewPreset:0];
    [self pressKey:20]; // Key 3.
    check(_scene.gizmoTool == GizmoTool::Rotate, "Key 3 must choose the Rotate tool");
    _scene.rotateSnap = 15 * robotics::pi / 180;
    // Rotate the rover's deck (a child of the chassis, parent of the head) about Y by dragging the ring.
    [self selectBody:5];
    Vec3 center = _world.bodies[5].position;
    Vec3 head = _world.bodies[6].position;
    double unit = [_scene gizmoView].worldPerPoint(center);
    double radius = gizmoRingRadius * unit;
    auto ring = [&](double degrees) { // Y ring: e1 = Z, e2 = X.
        double a = degrees * robotics::pi / 180;
        return [self viewPoint:center + Vec3{std::sin(a), 0, std::cos(a)} * radius];
    };
    [self send:NSEventTypeLeftMouseDown at:ring(20) flags:0];
    for (int degrees = 25; degrees <= 110; degrees += 5) {
        [self send:NSEventTypeLeftMouseDragged at:ring(degrees) flags:0];
    }
    check([_scene.gizmoReadout containsString:@"+90.0°"], "Angle readout wrong");
    [self writeScenePreview:@"build/gizmo-preview.png"];
    [self send:NSEventTypeLeftMouseUp at:ring(110) flags:0];
    Quat expected = Quat::axisAngle({0, 1, 0}, robotics::pi / 2);
    check(rotationVector(_world.bodies[5].orientation * expected.conjugate()).length() < 1e-9,
          "Part did not turn 90° about Y");
    check(_world.maxAnchorError() < 1e-9, "Rotation broke a joint");
    check((_world.bodies[5].position - center).length() < 1e-12, "Rotation must keep the part's center");
    Vec3 arm = head - center;
    check((_world.bodies[6].position - (center + expected.rotate(arm))).length() < 1e-9,
          "Attached part did not rotate with the assembly");
    // The inspector shows the new orientation.
    check(std::abs(_rotation[1].doubleValue - 90) < 1e-6, "Inspector rotation not updated");
    // One undo step for the whole drag; physics stays calm afterwards.
    [_undo undo];
    check((_world.bodies[6].position - head).length() < 1e-9, "One undo must revert the rotation");
    [_undo redo];
    [self toggleRun:nil];
    [self runTicks:10];
    [self toggleRun:nil];
    check(_world.maxAnchorError() < 5e-3, "Joints drifted after rotating");
    [self reset:nil];

    // Drop to Floor: the toolbar button's action on an assembly in the air.
    [self loadForTest:makeDemo() selected:1];
    [_scene viewPreset:0];
    [self selectBody:1];
    double lowest = assemblyLowestPoint(_world, 1);
    [self dropToFloor:nil];
    check(std::abs(assemblyLowestPoint(_world, 1)) < 1e-9 && lowest != 0, "Drop to Floor");
    check(_world.maxAnchorError() < 1e-9, "Drop to Floor broke a joint");
    _scene.gizmoTool = GizmoTool::Select;
    _scene.rotateSnap = 0;
}

- (void)checkTimelineFlow {
    World rover = makeSensorRoverDemo();
    [self loadForTest:rover selected:0];
    TimelineBar* bar = _timeline;
    check(!bar->_play.enabled && [bar->_time.stringValue isEqualToString:@"Run to record"], "Empty timeline");
    [self pressKey:13]; // W: drive; starts the simulation and the recording.
    check(_running, "W must start the run");
    [self runTicks:20]; // 2 s.
    [_window makeFirstResponder:_scene];
    check(std::abs(_world.time - 2) < 0.01, "Two seconds of simulation expected");
    check(_recording.size() == 121, "60 frames per second expected");
    check([bar->_time.stringValue containsString:@"REC"], "Timeline must show it is recording");

    // Scrub to the middle with the mouse: pauses, shows the frame, leaves the live state alone.
    World live = _world;
    NSRect track = [bar->_track convertRect:bar->_track.bounds toView:nil];
    auto trackPoint = [&](double fraction) {
        return NSMakePoint(track.origin.x + 8 + (track.size.width - 16) * fraction, NSMidY(track));
    };
    [_window sendEvent:mouseEvent(_window, NSEventTypeLeftMouseDown, trackPoint(0.5), 0)];
    check(_reviewing && !_running, "Scrubbing must pause and enter review");
    check(std::abs([self sceneWorld]->time - 1) < 0.02, "Scrub to the middle must show t = 1 s");
    [_window sendEvent:mouseEvent(_window, NSEventTypeLeftMouseDragged, trackPoint(0.25), 0)];
    check(std::abs([self sceneWorld]->time - 0.5) < 0.02, "Dragging the timeline");
    [_window sendEvent:mouseEvent(_window, NSEventTypeLeftMouseDragged, trackPoint(0.5), 0)];
    [_window sendEvent:mouseEvent(_window, NSEventTypeLeftMouseUp, trackPoint(0.5), 0)];
    std::size_t index = _reviewIndex;
    const RecordingFrame& shown = _recording.frame(index);
    check((([self sceneWorld])->bodies[0].position - shown.bodies[0].position).length() == 0,
          "Frame not shown");
    check((_world.bodies[0].position - live.bodies[0].position).length() == 0 && _world.time == live.time,
          "Scrubbing changed the live simulation");
    check((_world.bodies[0].position - shown.bodies[0].position).length() > 0.01, "Rover should have moved");
    [_scene homeCamera];
    [self refresh];
    check(std::abs(_position[0].doubleValue - shown.bodies[0].position.x) < 1e-3 || _reviewIndex != index,
          "Inspector must show the frame on screen");
    [self writeScenePreview:@"build/timeline-preview.png"];

    // Frame stepping: buttons and arrow keys.
    [bar->_stepForward performClick:nil];
    check(_reviewIndex == index + 1, "Step forward");
    [self pressKey:123];
    check(_reviewIndex == index, "← must step back");
    // Editing is refused while viewing.
    bool refused = false;
    try {
        [self addBox:nil];
    } catch (const std::exception&) {
        refused = true;
    }
    check(refused && !_linkCard.subviews.firstObject.hidden, "Edits must wait for Live");
    check(![self canPerform:@selector(deleteBody:)], "Delete must be disabled while reviewing");

    // Playback at 2×.
    [bar->_speed selectItemAtIndex:3];
    [bar->_speed sendAction:bar->_speed.action to:bar->_speed.target];
    [self timelinePlay:nil];
    double t0 = _recording.frame(_reviewIndex).time;
    [self tickReview:0.1];
    check(std::abs(_recording.frame(_reviewIndex).time - t0 - 0.2) < 0.02, "2× playback");
    [self timelinePlay:nil];
    check(!_reviewPlaying, "Pause playback");
    [bar->_speed selectItemAtIndex:2];
    [bar->_speed sendAction:bar->_speed.action to:bar->_speed.target];

    // Esc returns to live without changing it.
    [self pressKey:53];
    check(!_reviewing && (_world.bodies[0].position - live.bodies[0].position).length() == 0, "Esc → live");

    // CSV of the whole recording.
    check([self writeRecordingCsv:@"build/recording.csv"], "Could not export the recording");
    std::string header;
    check(csvRows("build/recording.csv", &header) == _recording.size(), "CSV rows");
    check(header.find("rover_chassis · position X (m)") != std::string::npos, "CSV header");

    // Resume from t = 1 s: state equals the frame, the future is gone, simulation runs on from there.
    [self timelineScrubTo:0.5];
    index = _reviewIndex;
    Vec3 frame = _recording.frame(index).bodies[0].position;
    double time = _recording.frame(index).time;
    [bar->_resume performClick:nil];
    check(_running && !_reviewing, "Resume must run");
    check(std::abs(_world.time - time) < 1e-9, "Resumed at the wrong time");
    check((_world.bodies[0].position - frame).length() < 1e-9, "Resumed at the wrong pose");
    check(_recording.size() == index + 1, "Later frames must be dropped");
    [self runTicks:5];
    [self toggleRun:nil];
    check(std::abs(_world.time - time - 0.5) < 0.01, "Simulation must continue");
    check(_recording.size() == index + 1 + 30, "Recording must continue");
    // Continuing along the same path: determinism across resume (same command as before).
    check(_world.bodies[0].velocity.finite() && _world.maxAnchorError() < 0.02, "Resumed state unstable");

    // Reset keeps the recording for review; an edit clears it.
    [self reset:nil];
    check(_world.time == 0 && !_recording.empty(), "Reset");
    [self timelineScrubTo:1];
    [self timelineLive:nil];
    [self addBox:nil];
    check(_recording.empty(), "Editing must clear the recording");
    [self deleteBody:nil];
}

- (void)checkPlotsFlow {
    [self loadForTest:makeSensorRoverDemo() selected:0];
    [self openPlots:nil];
    PlotWindow* plots = _plots;
    [plots removeAll:nil];
    // Search filters the list.
    plots->_search.stringValue = @"chassis angular speed";
    [plots filterChanged];
    check(plots->_filtered.size() == 1, "Search must narrow the list to one channel");
    plots->_search.stringValue = @"";
    [plots filterChanged];
    check(plots->_filtered.size() > 100, "All channels listed");
    // A real right-click on a part: selects it and opens the menu (closed again from a timer, as the menu
    // runs its own event loop).
    [_window makeKeyAndOrderFront:nil];
    [self selectBody:-1];
    NSPoint part = [self viewPoint:_world.bodies[0].position + Vec3{0, 0.14, 0}];
    _contextMenu = nil;
    NSTimer* close = [NSTimer timerWithTimeInterval:0.2
                                            repeats:NO
                                              block:^(NSTimer* timer) {
                                                  (void)timer;
                                                  [self->_contextMenu cancelTracking];
                                              }];
    [NSRunLoop.currentRunLoop addTimer:close forMode:NSRunLoopCommonModes];
    [NSRunLoop.currentRunLoop addTimer:close forMode:NSEventTrackingRunLoopMode];
    [self send:NSEventTypeRightMouseDown at:part flags:0];
    [self send:NSEventTypeRightMouseUp at:part flags:0];
    [close invalidate];
    check(_contextMenu && [_contextMenu itemWithTitle:@"Plot Position"] != nil,
          "Right-click must open the plot menu");
    check(_selected >= 0, "Right-click must select the part");
    [self menuNeedsUpdate:_inspectorPlotMenu];
    check(_inspectorPlotMenu.numberOfItems > 3, "Inspector fields must offer plot shortcuts");
    [self menuNeedsUpdate:_outline.menu];
    check([_outline.menu itemWithTag:1].submenu != nil, "Parts list must offer plot shortcuts");
    // Shortcuts from the context menu of a part (the same menu right-click shows).
    NSMenu* menu = [self sceneContextMenuForBody:1];
    check([menu itemWithTitle:@"Plot Motor Torque"] != nil, "Wheel menu must offer joint channels");
    for (NSString* title in @[ @"Plot Position", @"Plot Speed", @"Plot Motor Torque" ]) {
        NSMenuItem* item = [menu itemWithTitle:title];
        [NSApp sendAction:item.action to:item.target from:item];
    }
    check(plots->_charts.size() == 3, "Units must separate charts: m, m/s, N·m");
    check(plots->_charts[0].series.size() == 3, "Position has three components");
    // Add from the list: energy, contact count, IMU channel, joint reaction.
    for (const char* query : {"scene total kinetic", "contact count", "imu accel_y"}) {
        plots->_search.stringValue = @(query);
        [plots filterChanged];
        check(plots->_filtered.size() == 1, std::string("Search must find one channel: ") + query);
        [plots->_available selectRowIndexes:[NSIndexSet indexSetWithIndex:0] byExtendingSelection:NO];
        [NSApp sendAction:@selector(addSelected:) to:plots from:nil];
    }
    plots->_search.stringValue = @"";
    [plots filterChanged];
    // Adding twice does not duplicate; a second part's speed joins the chart with the same unit.
    std::size_t count = [plots activeChannels].size();
    [plots addChannels:{bodyChannel(_world, 1, BodyQuantity::Speed)} newChart:NO];
    check([plots activeChannels].size() == count, "Duplicate series");
    [plots addChannels:{bodyChannel(_world, 0, BodyQuantity::Speed)} newChart:NO];
    check(plots->_charts[1].series.size() == 2 && plots->_charts[1].series[1].slot == 1,
          "Same unit, same chart");

    [self pressKey:13]; // Drive.
    [self runTicks:20];
    [self toggleRun:nil];

    // Cursor readout through the mouse handler.
    NSRect charts = [plots->_chartsView convertRect:plots->_chartsView.bounds toView:nil];
    [plots->_chartsView mouseMoved:mouseEvent(plots->_window, NSEventTypeMouseMoved,
                                              NSMakePoint(NSMidX(charts), NSMidY(charts)), 0)];
    check([plots.hoverSummary containsString:@" m"], "Hover must read out the value under the mouse");
    // Export.
    check([plots exportCsv:@"build/plots.csv"], "Could not export plots");
    std::string header;
    check(csvRows("build/plots.csv", &header) == _recording.size(), "Plot CSV rows");
    check(header.find("wheel_front_right · position X (m)") != std::string::npos
              && header.find("imu · accel_y") != std::string::npos,
          "Plot CSV columns");
    // Scrubbing shows a cursor; clicking a chart moves the timeline while reviewing.
    [self timelineScrubTo:0.4];
    [plots setHoverX:-1];
    std::size_t index = _reviewIndex;
    NSPoint late = NSMakePoint(NSMaxX(charts) - 40, NSMidY(charts));
    NSEvent* click = mouseEvent(plots->_window, NSEventTypeLeftMouseDown, late, 0);
    [plots->_chartsView mouseDown:click];
    check(_reviewIndex > index, "Clicking a chart must seek the timeline");
    [self timelineScrubTo:0.4];
    // Pause freezes the time span while new data arrives.
    check(writeWindowImage(plots->_window, @"build/plots-preview.png"), "Could not render plots");
    [self timelineLive:nil];
    [plots togglePause:nil];
    double frozen = plots->_pausedEnd;
    [self toggleRun:nil];
    [self runTicks:3];
    [self toggleRun:nil];
    check(plots->_paused && _recording.endTime() > frozen, "Pause keeps the charts frozen");
    [plots togglePause:nil];
    // Series follow parts by name through edits and disappear with them.
    [self reset:nil];
    [self selectBody:1];
    std::size_t before = [plots activeChannels].size();
    [self deleteBody:nil];
    check([plots activeChannels].size() == before - 5, "Series of the deleted part must go");
    [_undo undo];
    [plots->_window orderOut:nil];
}

- (void)checkLibraryFlow {
    NSString* folder = @"build/test-library";
    [NSFileManager.defaultManager removeItemAtPath:folder error:nil];
    _libraryFolder = folder;
    [self loadForTest:makeRoverDemo() selected:0];
    [self openLibrary:nil];
    LibraryPanel* library = _library;
    [library reload];
    check(library->_table.numberOfRows == 9, "Nine built-in items expected");
    check(writeWindowImage(library->_window, @"build/library-preview.png"), "Could not render library");

    // Every item, attached and free, through the panel; each run 1 s and undone in one step.
    for (NSInteger row = 0; row < 9; ++row) {
        const LibraryItem& item = library->_items[static_cast<std::size_t>(row)];
        for (NSInteger placement : {0, 1}) {
            [self selectBody:0];
            std::size_t bodies = _world.bodies.size();
            library->_placement.selectedSegment = placement;
            [library refresh];
            [library->_table selectRowIndexes:[NSIndexSet indexSetWithIndex:static_cast<NSUInteger>(row)]
                         byExtendingSelection:NO];
            [library insert:nil];
            check(_world.bodies.size() > bodies, "Nothing inserted: " + item.title);
            check(_selected == static_cast<int>(bodies), "Inserted item must be selected");
            check(_world.maxAnchorError() < 1e-9, "Invalid joints after inserting " + item.title);
            [self toggleRun:nil];
            [self runTicks:10];
            [self toggleRun:nil];
            check(_world.maxAnchorError() < 0.02, "Joints separated: " + item.title);
            for (const Body& body : _world.bodies) {
                check(body.position.finite() && body.velocity.length() < 10, "Exploded: " + item.title);
            }
            [self reset:nil];
            [_undo undo];
            check(_world.bodies.size() == bodies, "One undo must remove " + item.title);
        }
    }
    // Build a small robot from modules, then save and reuse it.
    [self newScene:nil];
    [self openLibrary:nil];
    check(library->_placement.selectedSegment == 1, "Empty scene must place on the floor");
    [library->_table selectRowIndexes:[NSIndexSet indexSetWithIndex:5]
                 byExtendingSelection:NO]; // Chassis plate.
    [library insert:nil];
    int chassis = _selected;
    library->_placement.selectedSegment = 0;
    [library refresh];
    for (NSUInteger row : {0u, 0u, 6u}) { // Two wheels and a lidar mast.
        [self selectBody:chassis];
        [library->_table selectRowIndexes:[NSIndexSet indexSetWithIndex:row] byExtendingSelection:NO];
        [library insert:nil];
    }
    check(_world.bodies.size() == 6 && _world.sensors.size() == 1, "Chassis, two wheels, mast (3 parts)");
    check(std::abs(assemblyLowestPoint(_world, chassis)) < 1e-9,
          "Robot must stand on its wheels, not in the floor");
    [self selectBody:chassis];
    [_scene homeCamera];
    [self refresh];
    [self writeScenePreview:@"build/library-robot-preview.png"];
    check([self saveAssemblyNamed:@"My robot"], "Could not save the assembly");
    [library reload];
    check(library->_table.numberOfRows == 10 && library->_items.back().custom, "Saved item must be listed");
    check(library->_items.back().assembly.sensors.size() == 1, "Saved item keeps its sensors");
    check(library->_items.back().title == "My robot", "Saved item title");
    [library->_table selectRowIndexes:[NSIndexSet indexSetWithIndex:9] byExtendingSelection:NO];
    library->_placement.selectedSegment = 1;
    [library refresh];
    check(library->_remove.enabled, "Saved items can be removed");
    [library insert:nil];
    check(_world.bodies.size() == 12 && _world.sensors.size() == 2, "Saved item not inserted");
    check(_world.bodies[6].name == "chassis_plate_1", "Inserted copy must get unique names");
    [self toggleRun:nil];
    [self runTicks:10];
    [self toggleRun:nil];
    check(_world.maxAnchorError() < 0.02, "Custom item unstable");
    [self reset:nil];
    [library->_window orderOut:nil];
    _libraryFolder = nil;
    [NSFileManager.defaultManager removeItemAtPath:folder error:nil];
}

- (void)checkEditorUX {
    [self checkMoveGizmo];
    [self checkRotateGizmo];
    [self checkTimelineFlow];
    [self checkPlotsFlow];
    [self checkLibraryFlow];
}

@end
