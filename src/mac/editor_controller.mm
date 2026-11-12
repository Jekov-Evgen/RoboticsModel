// EditorController core: window construction, state, undo, feedback, viewport host, run loop.

#import "mac/editor_controller.h"
#import "mac/app_icon.h"
#import "mac/control_lab.h"
#import "mac/library_panel.h"
#import "mac/plot_window.h"
#import "mac/sensor_panel.h"

#include "robotics/demo_scenes.hpp"
#include "robotics/editing/assembly.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>

using namespace robotics;
using namespace editor_ui;

namespace {
bool smokeTestMode = false;
} // namespace

bool isEditorSmokeTest() {
    return smokeTestMode;
}

void setEditorSmokeTest(bool enabled) {
    smokeTestMode = enabled;
}

namespace {
EditorRenderJob renderJobData;
} // namespace

const EditorRenderJob& editorRenderJob() {
    return renderJobData;
}

void setEditorRenderJob(const char* scene, const char* output, double seconds, int view, int labJoint) {
    renderJobData = {scene, output, seconds, view, labJoint};
}

@implementation EditorController

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    (void)notification;
    _world = makeRoverDemo();
    _selected = 0;
    _inspectorBody = -1;
    _shownSensor = -1;
    _cameraSensor = -1;
    NSApp.applicationIconImage = roboticsAppIcon();
    _undo = [[NSUndoManager alloc] init];
    _undo.levelsOfUndo = 100;
    _textUndo = [[NSUndoManager alloc] init];
    if (isEditorSmokeTest()) {
        _undo.groupsByEvent = NO;
    }
    [self buildMenu];
    _window =
        [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 1440, 900)
                                    styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                              | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable
                                      backing:NSBackingStoreBuffered
                                        defer:NO];
    _window.title = @"Robotics Model";
    _window.minSize = NSMakeSize(1100, 680);
    _window.delegate = self;
    _window.releasedWhenClosed = NO;
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    _window.tabbingMode = NSWindowTabbingModeDisallowed;
    NSToolbar* toolbar = [[NSToolbar alloc] initWithIdentifier:@"main"];
    toolbar.delegate = self;
    toolbar.displayMode = NSToolbarDisplayModeIconAndLabel;
    toolbar.allowsUserCustomization = NO;
    _window.toolbar = toolbar;
    _window.toolbarStyle = NSWindowToolbarStyleUnified;
    NSView* root = _window.contentView;
    NSRect b = root.bounds;
    const CGFloat side = 236;
    const CGFloat inspector = 324;
    const CGFloat status = 28;

    // Parts list (left)
    NSVisualEffectView* sidebar =
        [[NSVisualEffectView alloc] initWithFrame:NSMakeRect(0, 0, side, b.size.height)];
    sidebar.material = NSVisualEffectMaterialSidebar;
    sidebar.blendingMode = NSVisualEffectBlendingModeBehindWindow;
    sidebar.autoresizingMask = NSViewHeightSizable | NSViewMaxXMargin;
    [root addSubview:sidebar];
    NSTextField* partsTitle =
        label(sidebar, @"Parts", NSMakeRect(14, b.size.height - 34, 120, 20), 13, NSFontWeightSemibold);
    partsTitle.autoresizingMask = NSViewMinYMargin;
    _partsCount = label(sidebar, @"", NSMakeRect(110, b.size.height - 33, side - 124, 18), 11,
                        NSFontWeightRegular, NSColor.secondaryLabelColor);
    _partsCount.alignment = NSTextAlignmentRight;
    _partsCount.autoresizingMask = NSViewMinYMargin;
    NSScrollView* listScroll =
        [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 84, side, b.size.height - 124)];
    listScroll.autoresizingMask = NSViewHeightSizable;
    listScroll.drawsBackground = NO;
    listScroll.hasVerticalScroller = YES;
    _outline = [[NSOutlineView alloc] initWithFrame:listScroll.bounds];
    _outline.style = NSTableViewStyleSourceList;
    _outline.headerView = nil;
    _outline.rowHeight = 26;
    _outline.backgroundColor = NSColor.clearColor;
    _outline.indentationPerLevel = 14;
    _outline.floatsGroupRows = NO;
    NSTableColumn* column = [[NSTableColumn alloc] initWithIdentifier:@"part"];
    column.width = side - 24;
    column.resizingMask = NSTableColumnAutoresizingMask;
    [_outline addTableColumn:column];
    _outline.outlineTableColumn = column;
    _outline.columnAutoresizingStyle = NSTableViewUniformColumnAutoresizingStyle;
    _outline.dataSource = self;
    _outline.delegate = self;
    NSMenu* context = [[NSMenu alloc] initWithTitle:@"Part"];
    context.delegate = self;
    [context addItemWithTitle:@"Duplicate" action:@selector(duplicateBody:) keyEquivalent:@""];
    [context addItemWithTitle:@"Delete" action:@selector(deleteBody:) keyEquivalent:@""];
    [context addItemWithTitle:@"Drop to Floor" action:@selector(dropToFloor:) keyEquivalent:@""];
    [context addItemWithTitle:@"Save Assembly to Library…"
                       action:@selector(saveAssemblyToLibrary:)
                keyEquivalent:@""];
    for (NSMenuItem* item in context.itemArray) {
        item.target = self;
    }
    NSMenuItem* plotItem = [context addItemWithTitle:@"Plot" action:nil keyEquivalent:@""];
    plotItem.tag = 1; // Filled in by menuNeedsUpdate: for the clicked part.
    _outline.menu = context;
    listScroll.documentView = _outline;
    [sidebar addSubview:listScroll];
    label(sidebar, @"Add a part", NSMakeRect(14, 58, 200, 16), 10.5, NSFontWeightSemibold,
          NSColor.secondaryLabelColor);
    NSArray* adds = @[
        @[ @"Box", @"cube", NSStringFromSelector(@selector(addBox:)) ],
        @[ @"Ball", @"circle", NSStringFromSelector(@selector(addSphere:)) ],
        @[ @"Wheel", @"circle.circle", NSStringFromSelector(@selector(addCylinder:)) ]
    ];
    for (int k = 0; k < 3; ++k) {
        NSButton* add = pushButton(sidebar, adds[k][0], NSMakeRect(8 + k * 74, 18, 72, 32), self,
                                   NSSelectorFromString(adds[k][2]), adds[k][1]);
        add.toolTip =
            [NSString stringWithFormat:@"Add a %@ above the selected part", [adds[k][0] lowercaseString]];
    }

    // 3D viewport and status bar (center)
    try {
        _scene = [[SceneView alloc]
            initWithFrame:NSMakeRect(side, status, b.size.width - side - inspector, b.size.height - status)];
    } catch (const std::exception& e) {
        NSAlert* a = [[NSAlert alloc] init];
        a.messageText = @"3D view unavailable";
        a.informativeText = @(e.what());
        [a runModal];
        std::exit(1);
    }
    _scene.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    _scene.host = self;
    [root addSubview:_scene];
    [self buildTimeline];
    NSView* statusBar =
        [[NSView alloc] initWithFrame:NSMakeRect(side, 0, b.size.width - side - inspector, status)];
    statusBar.wantsLayer = YES;
    statusBar.layer.backgroundColor =
        [NSColor colorWithCalibratedRed:0.085 green:0.1 blue:0.125 alpha:1].CGColor;
    statusBar.autoresizingMask = NSViewWidthSizable | NSViewMaxYMargin;
    [root addSubview:statusBar];
    _statusLeft = label(statusBar, @"", NSMakeRect(12, 5, 420, 17), 11, NSFontWeightRegular,
                        NSColor.secondaryLabelColor);
    _statusLeft.font = [NSFont monospacedDigitSystemFontOfSize:11 weight:NSFontWeightRegular];
    _statusLeft.autoresizingMask = NSViewWidthSizable;
    _statusRight = label(statusBar, @"", NSMakeRect(statusBar.frame.size.width - 312, 5, 300, 17), 11,
                         NSFontWeightRegular, NSColor.secondaryLabelColor);
    _statusRight.alignment = NSTextAlignmentRight;
    _statusRight.autoresizingMask = NSViewMinXMargin;

    // Inspector (right)
    _inspectorScroll = [[NSScrollView alloc]
        initWithFrame:NSMakeRect(b.size.width - inspector, 0, inspector, b.size.height)];
    _inspectorScroll.autoresizingMask = NSViewHeightSizable | NSViewMinXMargin;
    _inspectorScroll.hasVerticalScroller = YES;
    _inspectorScroll.autohidesScrollers = YES;
    _inspectorScroll.backgroundColor = [NSColor colorWithCalibratedRed:0.105 green:0.115 blue:0.135 alpha:1];
    _inspector = [[FlippedView alloc] initWithFrame:NSMakeRect(0, 0, inspector, 1000)];
    _inspectorScroll.documentView = _inspector;
    [root addSubview:_inspectorScroll];
    [self buildInspector];

    [self refresh];
    [_window center];
    [_window makeKeyAndOrderFront:nil];
    [_window makeFirstResponder:_scene];
    if (isEditorSmokeTest()) {
        // Scripted checks must not race real keyboard/mouse input (e.g. Space toggles Run).
        const NSEventMask userInput = NSEventMaskKeyDown | NSEventMaskKeyUp | NSEventMaskLeftMouseDown
                                      | NSEventMaskLeftMouseUp | NSEventMaskRightMouseDown
                                      | NSEventMaskOtherMouseDown | NSEventMaskScrollWheel;
        [NSEvent addLocalMonitorForEventsMatchingMask:userInput
                                              handler:^NSEvent*(NSEvent* event) {
                                                  (void)event;
                                                  return nil;
                                              }];
    } else {
        [NSApp activateIgnoringOtherApps:YES];
    }
    _previous = std::chrono::steady_clock::now();
    _timer = [NSTimer scheduledTimerWithTimeInterval:1.0 / 60
                                              target:self
                                            selector:@selector(tick:)
                                            userInfo:nil
                                             repeats:YES];
    [[NSRunLoop currentRunLoop] addTimer:_timer forMode:NSEventTrackingRunLoopMode];
    if (!editorRenderJob().scene.empty()) {
        [self performSelector:@selector(renderJob) withObject:nil afterDelay:0.5];
    } else if (isEditorSmokeTest()) {
        [self performSelector:@selector(smokeCheck) withObject:nil afterDelay:0.3];
    } else {
        [_scene showToast:
                    @"Welcome! Click the 3D view and press W to drive the rover, or select a part to edit it."
                    error:NO];
    }
}

- (NSUndoManager*)windowWillReturnUndoManager:(NSWindow*)window {
    // Typing in a text field gets its own short-lived history; everything else is model undo.
    return [window.firstResponder isKindOfClass:NSTextView.class] ? _textUndo : _undo;
}

- (void)undo:(id)sender {
    (void)sender;
    [self undoAction:nil];
}

- (void)redo:(id)sender {
    (void)sender;
    [self redoAction:nil];
}

- (void)undoAction:(id)sender {
    (void)sender;
    [_window makeFirstResponder:_scene];
    [_textUndo removeAllActions];
    if (_undo.canUndo) {
        [_undo undo];
    }
}

- (void)redoAction:(id)sender {
    (void)sender;
    [_window makeFirstResponder:_scene];
    if (_undo.canRedo) {
        [_undo redo];
    }
}

- (void)recordUndo:(NSString*)name {
    World before = _world;
    int selected = _selected;
    if (!_undo.groupsByEvent) {
        [_undo beginUndoGrouping];
    }
    [_undo registerUndoWithTarget:self
                          handler:^(EditorController* target) {
                              [target restoreWorld:before selected:selected name:name];
                          }];
    [_undo setActionName:name];
    if (!_undo.groupsByEvent) {
        [_undo endUndoGrouping];
    }
    _window.documentEdited = YES;
}

- (void)restoreWorld:(const World&)world selected:(int)selected name:(NSString*)name {
    [self recordUndo:name];
    _running = NO;
    _world = world;
    _selected = selected;
    [self invalidateSnapshot];
    [self refresh];
    [_scene holdCamera];
    [_scene showToast:[NSString stringWithFormat:@"%@: %@", _undo.isUndoing ? @"Undone" : @"Redone", name]
                error:NO];
}

- (void)showError:(NSString*)message {
    if (isEditorSmokeTest()) {
        throw std::runtime_error(message.UTF8String);
    }
    NSBeep();
    [_scene showToast:message error:YES];
}

- (void)toast:(NSString*)message {
    [_scene showToast:message error:NO];
}

- (BOOL)canEdit {
    if (_reviewing) {
        [self showError:@"You are viewing a recording. Press Live (Esc) to edit the robot."];
        return NO;
    }
    if (_running) {
        [self showError:@"Pause the simulation (Space) before editing the robot."];
        return NO;
    }
    return YES;
}

- (void)showHelp:(id)sender {
    (void)sender;
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Quick Start";
    alert.icon = NSApp.applicationIconImage;
    alert.informativeText =
        @"1. Add parts with the Add buttons (Box, Ball, Wheel).\n"
         "2. Select a part and edit its position, size and mass in the Inspector on the right. Changes apply "
         "when you press Return.\n"
         "3. To connect parts, select the child part and choose its parent under \"Attach to\". Pick Fixed, "
         "Ball, Hinge or Slider and click Create connection.\n"
         "4. Give a hinge or slider a motor speed and torque/force to make it move. \"Limit range\" keeps "
         "it between Min and Max.\n"
         "5. Press Run (Space). Reset puts everything back.\n"
         "6. Library (⌘B) inserts ready-made modules: wheels, servos, grippers, sensor masts. With a part "
         "selected they attach to it.\n\n"
         "Camera: drag to orbit, Shift-drag or right-drag to pan, scroll to zoom, F to focus the robot.\n"
         "Move and rotate parts: keys 1 Select, 2 Move, 3 Rotate (or the buttons in the 3D view), then drag "
         "a "
         "handle while paused. Choose a Snap step next to the buttons; hold ⌘ while dragging to invert it. "
         "Everything attached below the part goes along. Option-drag also moves a part (add Shift to lift "
         "it).\n"
         "Recording: every run is recorded. Drag the timeline at the bottom to look back, ← → step one "
         "frame, Esc returns to live. Resume Here continues the simulation from the frame shown.\n"
         "Plots (⌘P): plot any quantity; right-click a part in 3D or the Parts list for shortcuts.\n"
         "Driving: W/S forward and back, A/D turn, X brake.\n"
         "Every change can be undone with ⌘Z.";
    [alert runModal];
}

- (void)invalidateSnapshot {
    _hasSnapshot = NO;
    [self discardRecording]; // Recorded frames no longer describe this robot.
    _world.time = 0;
    _world.contacts.clear();
    _world.resetControllers();
    resetSensors(_world);
}

- (void)selectBody:(int)index {
    if (index != _selected) {
        [_window makeFirstResponder:_scene]; // Commit pending edits to the previous part first.
    }
    _selected = index;
    [self refresh];
}

- (void)refresh {
    if (_selected >= static_cast<int>(_world.bodies.size())) {
        _selected = static_cast<int>(_world.bodies.size()) - 1;
    }
    [self rebuildTree];
    _syncingSelection = YES;
    [_outline reloadData];
    [_outline expandItem:nil expandChildren:YES];
    NSInteger row = _selected >= 0 ? [_outline rowForItem:@(_selected)] : -1;
    if (row >= 0) {
        [_outline selectRowIndexes:[NSIndexSet indexSetWithIndex:row] byExtendingSelection:NO];
        [_outline scrollRowToVisible:row];
    } else {
        [_outline deselectAll:nil];
    }
    _syncingSelection = NO;
    _partsCount.stringValue =
        [NSString stringWithFormat:@"%zu parts · %zu joints", _world.bodies.size(), _world.joints.size()];
    [self refreshInspector];
    [self layoutInspector];
    [self updateStatus];
    [_lab refresh];
    [_sensorPanel refresh];
    [_plots refresh];
    [_library refresh];
    [self updateSensorViews];
    [self updateTimeline];
    _scene.needsDisplay = YES;
}

- (void)updateStatus {
    NSString* state = _reviewing ? @"Viewing recording"
                                 : (_running ? @"Simulating" : (_hasSnapshot ? @"Paused" : @"Editing"));
    NSString* detail = [NSString stringWithFormat:@"t = %.2f s   ·   %zu parts   ·   %zu joints",
                                                  _reviewing ? _reviewWorld.time : _world.time,
                                                  _world.bodies.size(), _world.joints.size()];
    [_scene setStatus:state detail:detail running:_running];
    _statusLeft.stringValue =
        _running || _hasSnapshot
            ? [NSString stringWithFormat:@"Contacts: %d    Joint drift: %.2f mm",
                                         _reviewing ? _recording.frame(_reviewIndex).contacts
                                                    : static_cast<int>(_world.contacts.size()),
                                         [self sceneWorld]->maxAnchorError() * 1000]
            : @"Edit mode — press Run (Space) to start physics";
    _statusRight.stringValue =
        [NSString stringWithFormat:@"%@   ·   Physics 240 Hz",
                                   _fileURL ? _fileURL.lastPathComponent : @"Unsaved robot"];
    _window.subtitle = _fileURL ? _fileURL.lastPathComponent : @"Untitled robot";
    if (_runItem) {
        _runItem.label = _running ? @"Pause" : @"Run";
        _runItem.image = [NSImage imageWithSystemSymbolName:_running ? @"pause.fill" : @"play.fill"
                                   accessibilityDescription:nil];
    }
}

- (void)toggleRun:(id)sender {
    (void)sender;
    [_window makeFirstResponder:_scene];
    [self leaveReview]; // Run continues from the live state, not from the frame on screen.
    if (!_running) {
        if (!_hasSnapshot || _recording.empty()) {
            [self startRecording];
        }
        if (!_hasSnapshot) {
            _snapshot = _world;
            _hasSnapshot = YES;
        }
        _previous = std::chrono::steady_clock::now();
        _accumulator = 0;
    }
    _running = !_running;
    [self refresh];
}

- (void)reset:(id)sender {
    (void)sender;
    _running = NO;
    [self leaveReview]; // The recording stays for review until the next run or edit.
    if (_hasSnapshot) {
        _world = _snapshot;
        _hasSnapshot = NO;
    }
    _accumulator = 0;
    [self refresh];
    [self toast:@"Reset: all parts are back at their starting positions."];
}

- (void)tick:(NSTimer*)timer {
    (void)timer;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - _previous).count();
    _previous = now;
    [self tickReview:elapsed];
    if (!_running) {
        return;
    }
    _accumulator += std::min(elapsed, 0.1);
    int steps = 0;
    try {
        while (_accumulator >= 1.0 / 240 && steps < 24) {
            stepSimulation(_world); // Physics step + sensors (sensor feedback reaches the controllers).
            [self afterSimulationStep];
            _recording.afterStep(_world);
            _accumulator -= 1.0 / 240;
            ++steps;
        }
        _world.validate();
    } catch (const std::exception& e) {
        _running = NO;
        [self refresh];
        [self showError:[NSString stringWithFormat:@"Simulation stopped: %s. Press Reset.", e.what()]];
    }
    [self updateStatus];
    [self updateSensorViews];
    [self updateTimeline];
    _scene.needsDisplay = YES;
    [_lab tick];
    [_plots tick];
}

#pragma mark Viewport host

- (const World*)sceneWorld {
    return _reviewing ? &_reviewWorld : &_world;
}

- (int)sceneSelected {
    return _selected;
}

- (BOOL)sceneRunning {
    return _running;
}

- (void)scenePicked:(int)body {
    [self selectBody:body];
}

- (void)sceneKey:(NSString*)action {
    if ([action isEqual:@"run"]) {
        [self toggleRun:nil];
    } else if ([action isEqual:@"delete"]) {
        [self deleteBody:nil];
    } else if ([action isEqual:@"frame-back"]) {
        [self stepFrames:-1];
    } else if ([action isEqual:@"frame-forward"]) {
        [self stepFrames:1];
    } else if ([action isEqual:@"live"]) {
        [self timelineLive:nil];
    } else {
        [self drive:action];
    }
}

- (BOOL)sceneBeginMove:(int)body {
    if (![self canEdit]) {
        return NO;
    }
    [self selectBody:body];
    [self recordUndo:@"Move Part"];
    return YES;
}

- (void)sceneMoveBy:(Vec3)delta {
    if (_selected < 0) {
        return;
    }
    if (_world.bodies[_selected].position.y + delta.y < 0.02) {
        delta.y = 0.02 - _world.bodies[_selected].position.y;
    }
    moveAssembly(_world, _selected, delta);
    [self invalidateSnapshot];
    setVector(_position, _world.bodies[_selected].position);
    _shownLinkValues = [self linkFieldSignature];
    _scene.needsDisplay = YES;
}

- (void)sceneEndMove {
    [self refresh];
}

- (BOOL)sceneGizmoAvailable {
    return !_running && !_reviewing && _selected >= 0;
}

- (BOOL)sceneBeginRotate {
    if (![self canEdit] || _selected < 0) {
        return NO;
    }
    [self recordUndo:@"Rotate Part"];
    _rotateBodies = _world.bodies;
    _rotateJoints = _world.joints;
    return YES;
}

- (void)sceneRotateAxis:(Vec3)axis pivot:(Vec3)pivot angle:(double)angle {
    if (_selected < 0 || _rotateBodies.size() != _world.bodies.size()) {
        return;
    }
    _world.bodies = _rotateBodies;
    _world.joints = _rotateJoints;
    rotateAssembly(_world, _selected, pivot, axis, angle);
    [self invalidateSnapshot];
    setVector(_rotation, _world.bodies[_selected].orientation.eulerAngles() * (180 / robotics::pi));
    setVector(_position, _world.bodies[_selected].position);
    _shownLinkValues = [self linkFieldSignature];
    _scene.needsDisplay = YES;
}

- (void)sceneDropToFloor {
    [self dropToFloor:nil];
}

- (NSMenu*)sceneContextMenuForBody:(int)body {
    if (body >= 0 && !_reviewing && !_running) {
        [self selectBody:body];
    }
    _contextMenu = [self plotMenuForBody:body];
    return _contextMenu;
}

@end
