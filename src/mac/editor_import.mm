// EditorController (Import): URDF import, 3D models as part visuals, collision-shape toggle, URDF examples.

#import "mac/editor_controller.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include "robotics/io/mesh.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/io/urdf.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace robotics;
using namespace editor_ui;

namespace {

// Visual model sizing choices in "Import 3D Model as Visual…".
enum VisualFit : NSInteger { FitToPart = 0, Metres = 1, Millimetres = 2 };

NSArray<UTType*>* typesForExtensions(NSArray<NSString*>* extensions) {
    NSMutableArray<UTType*>* types = [NSMutableArray array];
    for (NSString* extension in extensions) {
        if (UTType* type = [UTType typeWithFilenameExtension:extension]) {
            [types addObject:type];
        }
    }
    return types;
}

NSButton* checkbox(NSString* title, BOOL on) {
    NSButton* box = [NSButton checkboxWithTitle:title target:nil action:nil];
    box.state = on ? NSControlStateValueOn : NSControlStateValueOff;
    [box sizeToFit];
    return box;
}

} // namespace

@implementation EditorController (Import)

#pragma mark URDF

// Folder with the bundled examples: app resources, the source tree the editor was built from, or ./examples.
- (NSString*)examplesFolder {
    NSMutableArray<NSString*>* candidates = [NSMutableArray array];
    if (NSString* resources = NSBundle.mainBundle.resourcePath) {
        [candidates addObject:[resources stringByAppendingPathComponent:@"examples"]];
    }
#ifdef ROBOTICS_SOURCE_DIR
    [candidates addObject:[@ROBOTICS_SOURCE_DIR stringByAppendingPathComponent:@"examples"]];
#endif
    [candidates addObject:[NSFileManager.defaultManager.currentDirectoryPath
                              stringByAppendingPathComponent:@"examples"]];
    for (NSString* path in candidates) {
        BOOL directory = NO;
        if ([NSFileManager.defaultManager fileExistsAtPath:[path stringByAppendingPathComponent:@"urdf"]
                                               isDirectory:&directory]
            && directory) {
            return path;
        }
    }
    return nil;
}

- (std::vector<std::string>)importUrdfAtPath:(NSString*)path fixBase:(BOOL)fixBase {
    UrdfImportOptions options;
    options.fixBase = fixBase;
    UrdfImportResult result = loadUrdf(path.fileSystemRepresentation, options);
    NSString* name = result.robotName.empty() ? path.lastPathComponent.stringByDeletingPathExtension
                                              : @(result.robotName.c_str());
    [self loadWorld:std::move(result.world) selected:-1 name:[NSString stringWithFormat:@"Import %@", name]];
    _selected = _world.bodies.empty() ? -1 : 0;
    // _fileURL is kept, as with the built-in examples, so undoing the import leaves the document consistent.
    [_scene reloadMeshes];
    [_scene frameWorld];
    [self refresh];
    return result.warnings;
}

- (void)showImportWarnings:(const std::vector<std::string>&)warnings robot:(NSString*)robot {
    if (warnings.empty() || isEditorSmokeTest()) {
        return;
    }
    NSMutableString* text = [NSMutableString string];
    for (const auto& warning : warnings) {
        [text appendFormat:@"•  %s\n\n", warning.c_str()];
    }
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = [NSString stringWithFormat:@"%@ was imported with %zu note%@", robot, warnings.size(),
                                                   warnings.size() == 1 ? @"" : @"s"];
    alert.informativeText = @"The robot is ready to simulate. Some parts of the URDF were approximated or "
                            @"skipped:";
    NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 0, 460, 180)];
    scroll.hasVerticalScroller = YES;
    scroll.borderType = NSBezelBorder;
    NSTextView* list = [[NSTextView alloc] initWithFrame:scroll.bounds];
    list.editable = NO;
    list.string = text;
    list.font = [NSFont systemFontOfSize:12];
    list.textContainerInset = NSMakeSize(6, 6);
    list.autoresizingMask = NSViewWidthSizable;
    scroll.documentView = list;
    alert.accessoryView = scroll;
    [alert addButtonWithTitle:@"OK"];
    [alert beginSheetModalForWindow:_window completionHandler:nil];
}

- (void)importUrdfFile:(NSString*)path fixBase:(BOOL)fixBase {
    try {
        std::vector<std::string> warnings = [self importUrdfAtPath:path fixBase:fixBase];
        [self showImportWarnings:warnings robot:path.lastPathComponent];
    } catch (const std::exception& e) {
        if (isEditorSmokeTest()) {
            throw;
        }
        NSAlert* alert = [[NSAlert alloc] init];
        alert.alertStyle = NSAlertStyleWarning;
        alert.messageText = [NSString stringWithFormat:@"Could not import %@", path.lastPathComponent];
        alert.informativeText = @(e.what());
        [alert beginSheetModalForWindow:_window completionHandler:nil];
    }
}

- (void)importUrdf:(id)sender {
    (void)sender;
    if (![self canEdit]) {
        return;
    }
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = NO;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Choose a URDF robot description (.urdf). Meshes are found next to it or in its ROS "
                    @"package folders.";
    panel.allowedContentTypes = typesForExtensions(@[ @"urdf", @"xml" ]);
    NSButton* fix = checkbox(@"Fix the base to the ground (for robot arms)", NO);
    NSView* accessory = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, fix.frame.size.width + 32, 36)];
    fix.frameOrigin = NSMakePoint(16, 9);
    [accessory addSubview:fix];
    panel.accessoryView = accessory;
    panel.accessoryViewDisclosed = YES;
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    [self importUrdfFile:panel.URL.path fixBase:fix.state == NSControlStateValueOn];
}

- (void)loadUrdfExample:(NSString*)relative fixBase:(BOOL)fixBase {
    NSString* folder = [self examplesFolder];
    if (!folder) {
        [self showError:@"The examples folder was not found next to the app."];
        return;
    }
    [self importUrdfFile:[[folder stringByAppendingPathComponent:@"urdf"]
                             stringByAppendingPathComponent:relative]
                 fixBase:fixBase];
}

- (void)diffDriveExample:(id)sender {
    (void)sender;
    if ([self canEdit]) {
        [self loadUrdfExample:@"diff_drive.urdf" fixBase:NO];
    }
}

- (void)meshArmExample:(id)sender {
    (void)sender;
    if ([self canEdit]) {
        [self loadUrdfExample:@"demo_meshes/urdf/mesh_arm.urdf" fixBase:YES];
    }
}

- (void)urdfArmExample:(id)sender {
    (void)sender;
    if ([self canEdit]) {
        [self loadUrdfExample:@"two_link_arm.urdf" fixBase:YES];
    }
}

- (void)gripperExample:(id)sender {
    (void)sender;
    if ([self canEdit]) {
        [self loadUrdfExample:@"gripper.urdf" fixBase:YES];
    }
}

#pragma mark Visual models

// Replaces the part's visual with a mesh file. Fit: scale uniformly into the part's box and centre it.
- (void)attachVisualModel:(NSString*)path fit:(NSInteger)fit {
    if (_selected < 0) {
        return;
    }
    TriangleMesh mesh = loadMesh(path.fileSystemRepresentation); // Throws with a readable reason.
    MeshBounds bounds = mesh.bounds();
    Body& body = _world.bodies[static_cast<std::size_t>(_selected)];
    VisualMesh visual;
    visual.path = path.fileSystemRepresentation;
    double scale = fit == Millimetres ? 0.001 : 1;
    if (fit == FitToPart) {
        Vec3 extent = bounds.size();
        scale = 1e30;
        for (int axis = 0; axis < 3; ++axis) {
            if (extent[axis] > 1e-12) {
                scale = std::min(scale, body.size[axis] / extent[axis]);
            }
        }
        if (scale == 1e30) {
            throw std::runtime_error("The model has no extent");
        }
    }
    visual.scale = {scale, scale, scale};
    visual.position = fit == FitToPart ? bounds.center() * -scale : Vec3{};
    [self recordUndo:@"Set 3D Model"];
    body.visuals = {visual};
    [self invalidateSnapshot];
    [self refresh];
}

- (void)importVisualModel:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = NO;
    panel.allowsMultipleSelection = NO;
    panel.message =
        [NSString stringWithFormat:@"Choose an STL or OBJ model to show for “%s”. Physics keeps "
                                   @"using the part's shape.",
                                   _world.bodies[static_cast<std::size_t>(_selected)].name.c_str()];
    panel.allowedContentTypes = typesForExtensions(@[ @"stl", @"obj" ]);
    NSPopUpButton* size = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(110, 6, 250, 26) pullsDown:NO];
    [size
        addItemsWithTitles:@[ @"Fit to the part", @"File units are metres", @"File units are millimetres" ]];
    NSTextField* caption = [NSTextField labelWithString:@"Model size:"];
    caption.frame = NSMakeRect(16, 11, 90, 17);
    NSView* accessory = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 380, 38)];
    [accessory addSubview:caption];
    [accessory addSubview:size];
    panel.accessoryView = accessory;
    panel.accessoryViewDisclosed = YES;
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    try {
        [self attachVisualModel:panel.URL.path fit:size.indexOfSelectedItem];
        [self toast:[NSString
                        stringWithFormat:@"%@ now shows %@ — physics still uses its %@ shape",
                                         @(_world.bodies[static_cast<std::size_t>(_selected)].name.c_str()),
                                         panel.URL.lastPathComponent,
                                         shapeName(_world.bodies[static_cast<std::size_t>(_selected)].shape)
                                             .lowercaseString]];
    } catch (const std::exception& e) {
        [self showError:[NSString stringWithFormat:@"Could not use this model: %s", e.what()]];
    }
}

- (void)removeVisualModel:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    Body& body = _world.bodies[static_cast<std::size_t>(_selected)];
    if (body.visuals.empty() && !body.primitiveHidden) {
        return;
    }
    [self recordUndo:@"Remove 3D Model"];
    body.visuals.clear();
    body.primitiveHidden = false;
    [self invalidateSnapshot];
    [self refresh];
}

- (void)toggleCollisionShapes:(id)sender {
    (void)sender;
    _scene.showCollision = !_scene.showCollision;
}

#pragma mark Smoke test

// Imports the URDF examples through the editor, checks counts, persists visuals and renders
// build/urdf-preview.png (mesh arm) and build/urdf-diff-drive-preview.png.
- (void)checkUrdfImport {
    auto check = [](bool ok, const char* message) {
        if (!ok) {
            throw std::runtime_error(message);
        }
    };
    NSString* folder = [self examplesFolder];
    check(folder != nil, "Examples folder not found");
    [self diffDriveExample:nil];
    check(_world.bodies.size() == 4 && _world.joints.size() == 3, "Diff-drive import: body/joint counts");
    [self drive:@"w"];
    for (int i = 0; i < 360; ++i) {
        _world.step();
    }
    [self drive:@"x"];
    [self toggleRun:nil];
    check(_world.bodies[0].position.x > 0.1, "W must drive the imported diff-drive robot forward");
    [self reset:nil];
    [_scene homeCamera];
    NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:@"build/urdf-diff-drive-preview.png" atomically:YES],
          "Could not write diff-drive preview");

    [self meshArmExample:nil];
    check(_world.bodies.size() == 2 && _world.bodies[0].visuals.size() == 1
              && _world.bodies[1].visuals.size() == 2,
          "Mesh arm import: visuals");
    _world.joints[0].motorSpeed = 0.5; // Turn the arm a bit so the preview is not edge-on.
    for (int i = 0; i < 240; ++i) {
        _world.step();
    }
    _world.joints[0].motorSpeed = 0;
    _world.time = 0;
    _selected = 1;
    [self refresh];
    [_scene frameWorld];
    png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:@"build/urdf-preview.png" atomically:YES], "Could not write URDF preview");
    _scene.showCollision = YES;
    png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    check([png writeToFile:@"build/urdf-collision-preview.png" atomically:YES],
          "Could not write collision preview");
    _scene.showCollision = NO;

    // Visual models on an ordinary part persist through save/load.
    [self addBox:nil];
    NSString* obj = [folder stringByAppendingPathComponent:@"urdf/demo_meshes/meshes/arm.obj"];
    [self attachVisualModel:obj fit:0];
    const Body& box = _world.bodies.back();
    check(box.visuals.size() == 1 && std::abs(box.visuals[0].scale.x * 0.4 - box.size.x) < 1e-9,
          "Fit-to-part scale");
    saveScene(_world, "build/urdf-import.robot");
    World loaded = loadScene("build/urdf-import.robot");
    check(loaded.bodies.back().visuals.size() == 1 && loaded.bodies[1].visuals.size() == 2,
          "Visual meshes lost on save/load");
    [_undo undo];
    check(_world.bodies.back().visuals.empty(), "Undo did not remove the visual model");
    [self removeVisualModel:nil];
    bool rejected = false;
    try {
        [self importUrdfFile:@"tests/data/malformed_unclosed.urdf" fixBase:NO];
    } catch (const std::exception& e) {
        rejected = std::string(e.what()).find("line 9") != std::string::npos;
    }
    check(rejected, "Malformed URDF must be rejected with its line number");
}

@end
