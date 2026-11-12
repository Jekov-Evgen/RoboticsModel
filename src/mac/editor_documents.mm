// EditorController (Documents): open, save, save as and unsaved-changes handling.

#import "mac/editor_controller.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include "robotics/io/robot_format.hpp"

using namespace robotics;
using namespace editor_ui;

@implementation EditorController (Documents)

- (BOOL)writeTo:(NSURL*)url {
    try {
        saveScene(_world, url.fileSystemRepresentation);
    } catch (const std::exception& e) {
        [self showError:[NSString stringWithUTF8String:e.what()]];
        return NO;
    }
    _fileURL = url;
    _window.documentEdited = NO;
    [self updateStatus];
    [self toast:[NSString stringWithFormat:@"Saved %@", url.lastPathComponent]];
    return YES;
}

- (BOOL)saveAsReturningSuccess {
    if (![self canEdit]) {
        return NO;
    }
    [_window makeFirstResponder:_scene];
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = _fileURL ? _fileURL.lastPathComponent : @"my_robot.robot";
    UTType* type = [UTType typeWithFilenameExtension:@"robot"];
    if (type) {
        panel.allowedContentTypes = @[ type ];
    }
    if ([panel runModal] != NSModalResponseOK) {
        return NO;
    }
    return [self writeTo:panel.URL];
}

- (void)saveAs:(id)sender {
    (void)sender;
    [self saveAsReturningSuccess];
}

- (void)save:(id)sender {
    (void)sender;
    if (![self canEdit]) {
        return;
    }
    [_window makeFirstResponder:_scene];
    if (_fileURL) {
        [self writeTo:_fileURL];
    } else {
        [self saveAsReturningSuccess];
    }
}

- (void)load:(id)sender {
    (void)sender;
    if (![self canEdit]) {
        return;
    }
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = NO;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Choose a .robot file (or a .urdf robot description to import)";
    if ([panel runModal] != NSModalResponseOK) {
        return;
    }
    if ([panel.URL.pathExtension.lowercaseString isEqualToString:@"urdf"]) {
        [self importUrdfFile:panel.URL.path fixBase:NO];
        return;
    }
    try {
        World candidate = loadScene(panel.URL.fileSystemRepresentation);
        [self loadWorld:std::move(candidate) selected:_world.bodies.empty() ? -1 : 0 name:@"Open Robot"];
        _fileURL = panel.URL;
        [_scene reloadMeshes]; // Mesh files may have changed on disk since they were cached.
        _window.documentEdited = NO;
        [self updateStatus];
    } catch (const std::exception& e) {
        [self showError:[NSString stringWithFormat:@"Could not open the file: %s", e.what()]];
    }
}

- (BOOL)confirmDiscard {
    if (isEditorSmokeTest() || !_window.documentEdited || _closing) {
        return YES;
    }
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Save changes to this robot?";
    alert.informativeText = @"Your changes will be lost if you don't save them.";
    [alert addButtonWithTitle:@"Save…"];
    [alert addButtonWithTitle:@"Cancel"];
    [alert addButtonWithTitle:@"Don't Save"];
    NSModalResponse r = [alert runModal];
    if (r == NSAlertSecondButtonReturn) {
        return NO;
    }
    if (r == NSAlertFirstButtonReturn) {
        _running = NO;
        if (_fileURL ? ![self writeTo:_fileURL] : ![self saveAsReturningSuccess]) {
            return NO;
        }
    }
    _closing = YES;
    return YES;
}

- (BOOL)windowShouldClose:(NSWindow*)sender {
    (void)sender;
    return [self confirmDiscard];
}

- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication*)sender {
    (void)sender;
    return [self confirmDiscard] ? NSTerminateNow : NSTerminateCancel;
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
    (void)sender;
    return YES;
}

@end
