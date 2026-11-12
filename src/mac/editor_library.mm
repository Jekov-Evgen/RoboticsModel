// EditorController (Library): inserting and saving library items, Drop to Floor, tool menu and the plot
// shortcuts of the context menus.

#import "mac/editor_controller.h"
#import "mac/library_panel.h"
#import "mac/plot_window.h"

#include "robotics/editing/assembly.hpp"
#include "robotics/io/robot_format.hpp"

#include <algorithm>
#include <cmath>

using namespace robotics;
using namespace editor_ui;

namespace {

// Tags of the "plot a part" menu items.
enum PlotGroup {
    PlotPosition,
    PlotSpeed,
    PlotVelocity,
    PlotAngularVelocity,
    PlotOrientation,
    PlotKineticEnergy
};

std::vector<BodyQuantity> quantities(NSInteger group) {
    switch (group) {
    case PlotPosition:
        return {BodyQuantity::PositionX, BodyQuantity::PositionY, BodyQuantity::PositionZ};
    case PlotSpeed:
        return {BodyQuantity::Speed};
    case PlotVelocity:
        return {BodyQuantity::VelocityX, BodyQuantity::VelocityY, BodyQuantity::VelocityZ};
    case PlotAngularVelocity:
        return {BodyQuantity::AngularVelocityX, BodyQuantity::AngularVelocityY,
                BodyQuantity::AngularVelocityZ};
    case PlotOrientation:
        return {BodyQuantity::Roll, BodyQuantity::Pitch, BodyQuantity::Yaw};
    default:
        return {BodyQuantity::KineticEnergy};
    }
}

// Point on the floor near `target` that no part stands on.
Vec3 clearFloorPoint(const World& world, Vec3 target) {
    Vec3 point{target.x, 0, target.z};
    for (int attempt = 0; attempt < 40; ++attempt) {
        bool blocked = false;
        for (const Body& body : world.bodies) {
            double reach = body.size.length() / 2 + 0.4;
            blocked = blocked || std::hypot(body.position.x - point.x, body.position.z - point.z) < reach;
        }
        if (!blocked) {
            break;
        }
        point.z += 0.4;
    }
    return point;
}

} // namespace

@implementation EditorController (Library)

- (void)openLibrary:(id)sender {
    (void)sender;
    if (!_library) {
        _library = [[LibraryPanel alloc] initWithEditor:self];
    }
    [_library show];
}

- (void)openPlots:(id)sender {
    (void)sender;
    if (!_plots) {
        _plots = [[PlotWindow alloc] initWithEditor:self];
    }
    [_plots show];
}

- (NSString*)customLibraryFolder {
    if (_libraryFolder) {
        return _libraryFolder;
    }
    NSURL* support = [NSFileManager.defaultManager URLForDirectory:NSApplicationSupportDirectory
                                                          inDomain:NSUserDomainMask
                                                 appropriateForURL:nil
                                                            create:YES
                                                             error:nil];
    return [support URLByAppendingPathComponent:@"RoboticsModel/Library"].path;
}

- (std::vector<LibraryItem>)libraryItems {
    std::vector<LibraryItem> items = builtinLibrary();
    // Saved items: the user's library, then a Library folder beside the open robot file.
    NSMutableArray<NSString*>* folders = [NSMutableArray arrayWithObject:[self customLibraryFolder]];
    if (_fileURL) {
        [folders
            addObject:[_fileURL.URLByDeletingLastPathComponent URLByAppendingPathComponent:@"Library"].path];
    }
    for (NSString* folder in folders) {
        NSArray<NSString*>* files = [NSFileManager.defaultManager contentsOfDirectoryAtPath:folder error:nil];
        for (NSString* file in [files sortedArrayUsingSelector:@selector(localizedStandardCompare:)]) {
            if (![file.pathExtension.lowercaseString isEqualToString:@"robot"]) {
                continue;
            }
            NSString* path = [folder stringByAppendingPathComponent:file];
            try {
                LibraryItem item = customLibraryItem(file.stringByDeletingPathExtension.UTF8String,
                                                     loadScene(path.fileSystemRepresentation));
                item.id = path.UTF8String;
                items.push_back(std::move(item));
            } catch (const std::exception& e) {
                NSLog(@"Skipping library item %@: %s", path, e.what());
            }
        }
    }
    return items;
}

- (BOOL)insertItem:(const LibraryItem&)item attach:(BOOL)attach {
    if (![self canEdit]) {
        return NO;
    }
    [_window makeFirstResponder:_scene];
    int host = attach ? _selected : -1;
    World candidate = _world;
    LibraryInsertion insertion;
    try {
        insertion = insertLibraryItem(candidate, item, host, clearFloorPoint(_world, [_scene cameraTarget]));
    } catch (const std::exception& e) {
        [self showError:[NSString
                            stringWithFormat:@"Could not insert %@: %s", @(item.title.c_str()), e.what()]];
        return NO;
    }
    [self recordUndo:[NSString stringWithFormat:@"Insert %@", @(item.title.c_str())]];
    _world = std::move(candidate);
    _selected = insertion.root;
    [self invalidateSnapshot];
    [self refresh];
    [_scene holdCamera];
    NSString* where = host >= 0
                          ? [NSString stringWithFormat:@"attached to %@", @(_world.bodies[host].name.c_str())]
                          : @"on the floor";
    [self toast:[NSString stringWithFormat:@"Inserted %@ %@ — undo with ⌘Z", @(item.title.c_str()), where]];
    return YES;
}

- (BOOL)saveAssemblyNamed:(NSString*)name {
    if (_selected < 0) {
        return NO;
    }
    NSCharacterSet* unsafe = [NSCharacterSet characterSetWithCharactersInString:@"/:\\"];
    NSString* safe = [[name componentsSeparatedByCharactersInSet:unsafe] componentsJoinedByString:@"-"];
    safe = [safe stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
    if (safe.length == 0 || [safe hasPrefix:@"."]) {
        [self showError:@"Please enter a name for the library item."];
        return NO;
    }
    NSString* folder = [self customLibraryFolder];
    [NSFileManager.defaultManager createDirectoryAtPath:folder
                            withIntermediateDirectories:YES
                                             attributes:nil
                                                  error:nil];
    NSString* path = [folder stringByAppendingPathComponent:[safe stringByAppendingPathExtension:@"robot"]];
    for (int n = 2; [NSFileManager.defaultManager fileExistsAtPath:path]; ++n) {
        path = [folder stringByAppendingPathComponent:[[NSString stringWithFormat:@"%@ %d", safe, n]
                                                          stringByAppendingPathExtension:@"robot"]];
    }
    saveScene(extractAssembly(_world, _selected), path.fileSystemRepresentation);
    [_library reload];
    [self toast:[NSString stringWithFormat:@"Saved “%@” to the library.",
                                           path.lastPathComponent.stringByDeletingPathExtension]];
    return YES;
}

- (void)saveAssemblyToLibrary:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    [_window makeFirstResponder:_scene];
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Save to Library";
    alert.informativeText =
        [NSString stringWithFormat:@"Saves “%@” and everything attached below it as a reusable item.",
                                   @(_world.bodies[static_cast<std::size_t>(_selected)].name.c_str())];
    NSTextField* field = [[NSTextField alloc] initWithFrame:NSMakeRect(0, 0, 260, 24)];
    field.stringValue = @(_world.bodies[static_cast<std::size_t>(_selected)].name.c_str());
    alert.accessoryView = field;
    alert.window.initialFirstResponder = field;
    [alert addButtonWithTitle:@"Save"];
    [alert addButtonWithTitle:@"Cancel"];
    if ([alert runModal] != NSAlertFirstButtonReturn) {
        return;
    }
    try {
        [self saveAssemblyNamed:field.stringValue];
    } catch (const std::exception& e) {
        [self showError:[NSString stringWithFormat:@"Could not save: %s", e.what()]];
    }
}

- (void)dropToFloor:(id)sender {
    (void)sender;
    if (![self canEdit]) {
        return;
    }
    if (_selected < 0) {
        [self showError:@"Select a part first."];
        return;
    }
    [_window makeFirstResponder:_scene];
    [self recordUndo:@"Drop to Floor"];
    double shift = dropAssemblyToFloor(_world, _selected);
    [self invalidateSnapshot];
    [self refresh];
    [_scene holdCamera];
    [self toast:[NSString stringWithFormat:@"Dropped to the floor (moved %.3f m).", std::abs(shift)]];
}

- (void)selectTool:(NSMenuItem*)sender {
    _scene.gizmoTool = static_cast<GizmoTool>(sender.tag);
}

- (NSMenu*)plotMenuForBody:(int)body {
    if (body < 0 || body >= static_cast<int>(_world.bodies.size())) {
        return nil;
    }
    NSMenu* menu = [[NSMenu alloc] initWithTitle:@"Plot"];
    auto add = [&](NSString* title, SEL action, NSInteger tag, int index) {
        NSMenuItem* item = [menu addItemWithTitle:title action:action keyEquivalent:@""];
        item.target = self;
        item.tag = tag;
        item.representedObject = @(index);
    };
    NSMenuItem* header = [menu addItemWithTitle:@(_world.bodies[static_cast<std::size_t>(body)].name.c_str())
                                         action:nil
                                  keyEquivalent:@""];
    header.enabled = NO;
    add(@"Plot Position", @selector(plotBodyQuantity:), PlotPosition, body);
    add(@"Plot Orientation", @selector(plotBodyQuantity:), PlotOrientation, body);
    if (!_world.bodies[static_cast<std::size_t>(body)].isStatic()) {
        add(@"Plot Speed", @selector(plotBodyQuantity:), PlotSpeed, body);
        add(@"Plot Velocity", @selector(plotBodyQuantity:), PlotVelocity, body);
        add(@"Plot Angular Velocity", @selector(plotBodyQuantity:), PlotAngularVelocity, body);
        add(@"Plot Kinetic Energy", @selector(plotBodyQuantity:), PlotKineticEnergy, body);
    }
    for (int j = 0; j < static_cast<int>(_world.joints.size()); ++j) {
        const Joint& joint = _world.joints[static_cast<std::size_t>(j)];
        if (joint.b != body) {
            continue;
        }
        [menu addItem:NSMenuItem.separatorItem];
        if (joint.isActuated()) {
            BOOL linear = joint.type == JointType::Prismatic;
            add(linear ? @"Plot Joint Position" : @"Plot Joint Angle", @selector(plotJointQuantity:),
                static_cast<NSInteger>(JointQuantity::Coordinate), j);
            add(@"Plot Joint Speed", @selector(plotJointQuantity:),
                static_cast<NSInteger>(JointQuantity::Speed), j);
            add(linear ? @"Plot Motor Force" : @"Plot Motor Torque", @selector(plotJointQuantity:),
                static_cast<NSInteger>(JointQuantity::Effort), j);
        }
        add(@"Plot Joint Reaction Force", @selector(plotJointQuantity:),
            static_cast<NSInteger>(JointQuantity::ReactionForce), j);
    }
    return menu;
}

- (void)plotBodyQuantity:(NSMenuItem*)sender {
    int body = [sender.representedObject intValue];
    std::vector<Channel> channels;
    for (BodyQuantity quantity : quantities(sender.tag)) {
        channels.push_back(bodyChannel(_world, body, quantity));
    }
    [self openPlots:nil];
    [_plots addChannels:channels newChart:NO];
}

- (void)plotJointQuantity:(NSMenuItem*)sender {
    int joint = [sender.representedObject intValue];
    [self openPlots:nil];
    [_plots addChannels:{jointChannel(_world, joint, static_cast<JointQuantity>(sender.tag))} newChart:NO];
}

@end
