// macOS editor entry point. `robotics_editor --smoke-test` runs the automated UI self check;
// `robotics_editor --render scene.robot out.png [seconds [view [lab_joint]]]` renders one editor screenshot and exits.

#import <Cocoa/Cocoa.h>

#import "mac/editor_controller.h"

#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--smoke-test") {
        setEditorSmokeTest(true);
    } else if (argc > 3 && std::string(argv[1]) == "--render") {
        setEditorSmokeTest(true); // Scripted run: no dialogs, user input is ignored.
        setEditorRenderJob(argv[2], argv[3], argc > 4 ? std::atof(argv[4]) : 0, argc > 5 ? std::atoi(argv[5]) : 0,
                           argc > 6 ? std::atoi(argv[6]) : -1);
    }
    @autoreleasepool {
        NSApplication* app = [NSApplication sharedApplication];
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];
        EditorController* delegate = [[EditorController alloc] init];
        app.delegate = delegate;
        [app run];
    }
    return 0;
}
