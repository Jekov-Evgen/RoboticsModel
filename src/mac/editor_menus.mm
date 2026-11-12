// EditorController (Menus): menu bar, toolbar and command validation.

#import "mac/editor_controller.h"

using namespace robotics;
using namespace editor_ui;

@implementation EditorController (Menus)

- (NSMenuItem*)item:(NSMenu*)menu
              title:(NSString*)title
             action:(SEL)action
                key:(NSString*)key
               mods:(NSEventModifierFlags)mods
             target:(id)target {
    NSMenuItem* item = [menu addItemWithTitle:title action:action keyEquivalent:key];
    item.keyEquivalentModifierMask = mods;
    item.target = target;
    return item;
}

- (void)buildMenu {
    NSMenu* bar = [[NSMenu alloc] init];
    auto submenu = [&](NSString* title) {
        NSMenuItem* top = [[NSMenuItem alloc] initWithTitle:title action:nil keyEquivalent:@""];
        NSMenu* m = [[NSMenu alloc] initWithTitle:title];
        top.submenu = m;
        [bar addItem:top];
        return m;
    };
    const NSEventModifierFlags cmd = NSEventModifierFlagCommand,
                               shiftCmd = NSEventModifierFlagCommand | NSEventModifierFlagShift,
                               optCmd = NSEventModifierFlagCommand | NSEventModifierFlagOption;
    NSMenu* app = submenu(@"Robotics Model");
    [self item:app
         title:@"About Robotics Model"
        action:@selector(orderFrontStandardAboutPanel:)
           key:@""
          mods:0
        target:NSApp];
    [app addItem:NSMenuItem.separatorItem];
    [self item:app title:@"Hide Robotics Model" action:@selector(hide:) key:@"h" mods:cmd target:NSApp];
    [self item:app title:@"Quit Robotics Model" action:@selector(terminate:) key:@"q" mods:cmd target:NSApp];
    NSMenu* file = submenu(@"File");
    [self item:file title:@"New Robot" action:@selector(newScene:) key:@"n" mods:cmd target:self];
    [self item:file title:@"Open…" action:@selector(load:) key:@"o" mods:cmd target:self];
    [self item:file title:@"Import URDF…" action:@selector(importUrdf:) key:@"i" mods:cmd target:self];
    [self item:file
         title:@"Import 3D Model as Visual…"
        action:@selector(importVisualModel:)
           key:@"i"
          mods:shiftCmd
        target:self];
    [self item:file title:@"Remove 3D Model" action:@selector(removeVisualModel:) key:@"" mods:0 target:self];
    [file addItem:NSMenuItem.separatorItem];
    [self item:file title:@"Save" action:@selector(save:) key:@"s" mods:cmd target:self];
    [self item:file title:@"Save As…" action:@selector(saveAs:) key:@"s" mods:shiftCmd target:self];
    NSMenu* edit = submenu(@"Edit");
    [self item:edit title:@"Undo" action:@selector(undo:) key:@"z" mods:cmd target:nil];
    [self item:edit title:@"Redo" action:@selector(redo:) key:@"z" mods:shiftCmd target:nil];
    [edit addItem:NSMenuItem.separatorItem];
    [self item:edit title:@"Cut" action:@selector(cut:) key:@"x" mods:cmd target:nil];
    [self item:edit title:@"Copy" action:@selector(copy:) key:@"c" mods:cmd target:nil];
    [self item:edit title:@"Paste" action:@selector(paste:) key:@"v" mods:cmd target:nil];
    [self item:edit title:@"Select All" action:@selector(selectAll:) key:@"a" mods:cmd target:nil];
    [edit addItem:NSMenuItem.separatorItem];
    [self item:edit title:@"Duplicate Part" action:@selector(duplicateBody:) key:@"d" mods:cmd target:self];
    [self item:edit title:@"Delete Part" action:@selector(deleteBody:) key:@"" mods:0 target:self];
    [self item:edit title:@"Drop to Floor" action:@selector(dropToFloor:) key:@"" mods:0 target:self];
    NSMenu* add = submenu(@"Add");
    [self item:add title:@"Box" action:@selector(addBox:) key:@"1" mods:cmd target:self];
    [self item:add title:@"Ball" action:@selector(addSphere:) key:@"2" mods:cmd target:self];
    [self item:add title:@"Wheel" action:@selector(addCylinder:) key:@"3" mods:cmd target:self];
    [add addItem:NSMenuItem.separatorItem];
    [self item:add title:@"Part Library…" action:@selector(openLibrary:) key:@"b" mods:cmd target:self];
    [self item:add
         title:@"Save Assembly to Library…"
        action:@selector(saveAssemblyToLibrary:)
           key:@""
          mods:0
        target:self];
    NSMenu* sim = submenu(@"Simulation");
    [self item:sim title:@"Run / Pause" action:@selector(toggleRun:) key:@"r" mods:cmd target:self];
    [self item:sim title:@"Reset to Start" action:@selector(reset:) key:@"r" mods:shiftCmd target:self];
    [sim addItem:NSMenuItem.separatorItem];
    [self item:sim
         title:@"Control Lab (PID & Plots)…"
        action:@selector(openControlLab:)
           key:@"l"
          mods:cmd
        target:self];
    [self item:sim
         title:@"Sensors (Live Readings)…"
        action:@selector(openSensors:)
           key:@"k"
          mods:cmd
        target:self];
    [self item:sim title:@"Plots…" action:@selector(openPlots:) key:@"p" mods:cmd target:self];
    [sim addItem:NSMenuItem.separatorItem];
    [self item:sim
         title:@"Export Recording as CSV…"
        action:@selector(exportRecording:)
           key:@""
          mods:0
        target:self];
    [self item:sim title:@"Clear Recording" action:@selector(clearRecording:) key:@"" mods:0 target:self];
    NSMenu* view = submenu(@"View");
    [self item:view title:@"Focus on Robot" action:@selector(homeCamera:) key:@"0" mods:cmd target:self];
    [self item:view
         title:@"Show Collision Shapes"
        action:@selector(toggleCollisionShapes:)
           key:@"c"
          mods:optCmd
        target:self];
    [view addItem:NSMenuItem.separatorItem];
    // Plain 1/2/3 are handled by the 3D view: as menu shortcuts they would block typing digits in fields.
    NSArray* tools = @[ @"Select Tool (1)", @"Move Tool (2)", @"Rotate Tool (3)" ];
    for (NSInteger k = 0; k < 3; ++k) {
        [self item:view title:tools[k] action:@selector(selectTool:) key:@"" mods:0 target:self].tag = k;
    }
    [view addItem:NSMenuItem.separatorItem];
    NSArray* presets = @[ @"Perspective", @"Top", @"Front", @"Side" ];
    for (NSInteger k = 0; k < 4; ++k) {
        NSMenuItem* p = [self item:view
                             title:presets[k]
                            action:@selector(cameraPreset:)
                               key:[NSString stringWithFormat:@"%ld", k + 1]
                              mods:optCmd
                            target:self];
        p.tag = k;
    }
    NSMenu* examples = submenu(@"Examples");
    [self item:examples title:@"Starter Rover" action:@selector(rover:) key:@"" mods:0 target:self];
    [self item:examples title:@"Robot Arm" action:@selector(demo:) key:@"" mods:0 target:self];
    [self item:examples title:@"PID Test Bench" action:@selector(pidDemo:) key:@"" mods:0 target:self];
    [self item:examples title:@"Sensor Rover" action:@selector(sensorRover:) key:@"" mods:0 target:self];
    [self item:examples
         title:@"Noisy PID (encoder feedback)"
        action:@selector(noisyPidDemo:)
           key:@""
          mods:0
        target:self];
    [self item:examples
         title:@"Linear Actuator & Limits"
        action:@selector(actuatorDemo:)
           key:@""
          mods:0
        target:self];
    [examples addItem:NSMenuItem.separatorItem];
    [self item:examples
         title:@"Diff-Drive Robot (URDF)"
        action:@selector(diffDriveExample:)
           key:@""
          mods:0
        target:self];
    [self item:examples
         title:@"Mesh Arm (URDF)"
        action:@selector(meshArmExample:)
           key:@""
          mods:0
        target:self];
    [self item:examples
         title:@"Two-Link Arm (URDF)"
        action:@selector(urdfArmExample:)
           key:@""
          mods:0
        target:self];
    [self item:examples title:@"Gripper (URDF)" action:@selector(gripperExample:) key:@"" mods:0 target:self];
    NSMenu* window = submenu(@"Window");
    [self item:window title:@"Minimize" action:@selector(performMiniaturize:) key:@"m" mods:cmd target:nil];
    NSApp.windowsMenu = window;
    NSMenu* help = submenu(@"Help");
    [self item:help title:@"Quick Start Guide" action:@selector(showHelp:) key:@"?" mods:cmd target:self];
    NSApp.helpMenu = help;
    NSApp.mainMenu = bar;
}

- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)toolbar {
    (void)toolbar;
    return @[
        @"run", @"reset", NSToolbarSpaceItemIdentifier, @"undo", @"redo", NSToolbarSpaceItemIdentifier,
        @"add", @"delete", NSToolbarFlexibleSpaceItemIdentifier, @"examples", @"open", @"save",
        NSToolbarSpaceItemIdentifier, @"library", @"plots", @"lab", @"help"
    ];
}

- (NSArray<NSToolbarItemIdentifier>*)toolbarAllowedItemIdentifiers:(NSToolbar*)toolbar {
    return [self toolbarDefaultItemIdentifiers:toolbar];
}

- (NSToolbarItem*)toolbar:(NSToolbar*)toolbar
        itemForItemIdentifier:(NSToolbarItemIdentifier)identifier
    willBeInsertedIntoToolbar:(BOOL)flag {
    (void)toolbar;
    (void)flag;
    auto make = [&](NSString* title, NSString* symbol, SEL action, NSString* tip) {
        NSToolbarItem* item = [[NSToolbarItem alloc] initWithItemIdentifier:identifier];
        item.label = title;
        item.paletteLabel = title;
        item.toolTip = tip;
        item.image = [NSImage imageWithSystemSymbolName:symbol accessibilityDescription:title];
        item.target = self;
        item.action = action;
        item.bordered = YES;
        return item;
    };
    auto menuItem = [&](NSString* title, NSString* symbol, NSString* tip, NSArray* entries) {
        NSMenuToolbarItem* item = [[NSMenuToolbarItem alloc] initWithItemIdentifier:identifier];
        item.label = title;
        item.paletteLabel = title;
        item.toolTip = tip;
        item.image = [NSImage imageWithSystemSymbolName:symbol accessibilityDescription:title];
        item.showsIndicator = YES;
        NSMenu* menu = [[NSMenu alloc] init];
        for (NSArray* e in entries) {
            NSMenuItem* m = [menu addItemWithTitle:e[0] action:NSSelectorFromString(e[1]) keyEquivalent:@""];
            m.target = self;
            if ([e count] > 2) {
                m.image = [NSImage imageWithSystemSymbolName:e[2] accessibilityDescription:nil];
            }
        }
        item.menu = menu;
        return (NSToolbarItem*)item;
    };
    if ([identifier isEqual:@"run"]) {
        return _runItem = make(_running ? @"Pause" : @"Run", _running ? @"pause.fill" : @"play.fill",
                               @selector(toggleRun:), @"Start or pause the physics simulation (Space)");
    }
    if ([identifier isEqual:@"reset"]) {
        return make(@"Reset", @"backward.end.fill", @selector(reset:),
                    @"Return every part to where it was before you pressed Run (⇧⌘R)");
    }
    if ([identifier isEqual:@"undo"]) {
        return make(@"Undo", @"arrow.uturn.backward", @selector(undoAction:), @"Undo the last change (⌘Z)");
    }
    if ([identifier isEqual:@"redo"]) {
        return make(@"Redo", @"arrow.uturn.forward", @selector(redoAction:), @"Redo (⇧⌘Z)");
    }
    if ([identifier isEqual:@"add"]) {
        return menuItem(@"Add", @"plus", @"Add a new part", @[
            @[ @"Box", NSStringFromSelector(@selector(addBox:)), @"cube" ],
            @[ @"Ball", NSStringFromSelector(@selector(addSphere:)), @"circle" ],
            @[ @"Wheel", NSStringFromSelector(@selector(addCylinder:)), @"circle.circle" ]
        ]);
    }
    if ([identifier isEqual:@"delete"]) {
        return make(@"Delete", @"trash", @selector(deleteBody:),
                    @"Delete the selected part (⌫ in the 3D view)");
    }
    if ([identifier isEqual:@"examples"]) {
        return menuItem(@"Examples", @"square.grid.2x2", @"Load a ready-made robot", @[
            @[ @"Starter Rover", NSStringFromSelector(@selector(rover:)), @"car" ],
            @[ @"Robot Arm", NSStringFromSelector(@selector(demo:)), @"wrench.and.screwdriver" ],
            @[ @"PID Test Bench", NSStringFromSelector(@selector(pidDemo:)), @"waveform.path.ecg" ],
            @[ @"Sensor Rover", NSStringFromSelector(@selector(sensorRover:)), @"sensor" ],
            @[ @"Noisy PID", NSStringFromSelector(@selector(noisyPidDemo:)), @"waveform.path.badge.minus" ],
            @[
                @"Linear Actuator & Limits", NSStringFromSelector(@selector(actuatorDemo:)),
                @"arrow.left.and.right.square"
            ],
            @[ @"Diff-Drive Robot (URDF)", NSStringFromSelector(@selector(diffDriveExample:)), @"car.side" ],
            @[ @"Mesh Arm (URDF)", NSStringFromSelector(@selector(meshArmExample:)), @"cube.transparent" ],
            @[ @"Empty Scene", NSStringFromSelector(@selector(newScene:)), @"doc" ]
        ]);
    }
    if ([identifier isEqual:@"open"]) {
        return make(@"Open", @"folder", @selector(load:), @"Open a saved .robot file (⌘O)");
    }
    if ([identifier isEqual:@"save"]) {
        return make(@"Save", @"square.and.arrow.down", @selector(save:), @"Save this robot (⌘S)");
    }
    if ([identifier isEqual:@"library"]) {
        return make(@"Library", @"shippingbox", @selector(openLibrary:),
                    @"Insert ready-made modules: wheels, servos, grippers, sensor masts (⌘B)");
    }
    if ([identifier isEqual:@"plots"]) {
        return make(@"Plots", @"chart.xyaxis.line", @selector(openPlots:),
                    @"Plot any quantity of the simulation (⌘P)");
    }
    if ([identifier isEqual:@"lab"]) {
        return make(@"Control Lab", @"waveform.path.ecg", @selector(openControlLab:),
                    @"Tune PID controllers and see response plots (⌘L)");
    }
    if ([identifier isEqual:@"help"]) {
        return make(@"Help", @"questionmark.circle", @selector(showHelp:), @"Quick start guide");
    }
    return nil;
}

- (BOOL)canPerform:(SEL)action {
    if (action == @selector(undoAction:)) {
        return _undo.canUndo;
    }
    if (action == @selector(redoAction:)) {
        return _undo.canRedo;
    }
    const BOOL editable = !_running && !_reviewing;
    if (action == @selector(deleteBody:) || action == @selector(duplicateBody:)
        || action == @selector(importVisualModel:) || action == @selector(dropToFloor:)
        || action == @selector(saveAssemblyToLibrary:)) {
        return editable && _selected >= 0;
    }
    if (action == @selector(exportRecording:) || action == @selector(clearRecording:)) {
        return !_recording.empty();
    }
    if (action == @selector(removeVisualModel:)) {
        return editable && _selected >= 0
               && (!_world.bodies[static_cast<std::size_t>(_selected)].visuals.empty()
                   || _world.bodies[static_cast<std::size_t>(_selected)].primitiveHidden);
    }
    if (action == @selector(importUrdf:)) {
        return editable;
    }
    if (action == @selector(addBox:) || action == @selector(addSphere:) || action == @selector(addCylinder:)
        || action == @selector(save:) || action == @selector(saveAs:)) {
        return editable;
    }
    return YES;
}

- (BOOL)validateToolbarItem:(NSToolbarItem*)item {
    return [self canPerform:item.action];
}

- (BOOL)validateMenuItem:(NSMenuItem*)item {
    if (item.action == @selector(toggleCollisionShapes:)) {
        item.state = _scene.showCollision ? NSControlStateValueOn : NSControlStateValueOff;
    }
    if (item.action == @selector(setRecordingLength:)) {
        item.state = item.tag == static_cast<NSInteger>(_recording.duration()) ? NSControlStateValueOn
                                                                               : NSControlStateValueOff;
    }
    if (item.action == @selector(selectTool:)) {
        item.state = static_cast<NSInteger>(_scene.gizmoTool) == item.tag ? NSControlStateValueOn
                                                                          : NSControlStateValueOff;
    }
    return [self canPerform:item.action];
}

- (void)menuNeedsUpdate:(NSMenu*)menu {
    if (menu == _inspectorPlotMenu) { // Right-click on the inspector's fields.
        [menu removeAllItems];
        NSMenu* items = [self plotMenuForBody:_selected];
        for (NSMenuItem* item in items.itemArray.copy) {
            [items removeItem:item];
            [menu addItem:item];
        }
        return;
    }
    if (menu != _outline.menu) {
        return;
    }
    NSInteger row = _outline.clickedRow;
    if (row >= 0) {
        [self selectBody:[[_outline itemAtRow:row] intValue]];
    }
    [menu itemWithTag:1].submenu = _selected >= 0 ? [self plotMenuForBody:_selected] : nil;
}

@end
