// EditorController (RenderJob): `robotics_editor --render scene.robot out.png [seconds [view]]`.
// Loads a scene, advances the physics, renders the real editor window to a PNG and exits. Used for
// documentation screenshots; shares windowImage with the smoke test.

#import "mac/control_lab.h"
#import "mac/editor_controller.h"

#include "robotics/io/robot_format.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>

using namespace robotics;

@implementation EditorController (RenderJob)

- (void)renderJob {
    const EditorRenderJob& job = editorRenderJob();
    try {
        World world = loadScene(job.scene);
        [self loadWorld:std::move(world) selected:-1 name:@"Render"];
        [_window makeKeyAndOrderFront:nil];
        int steps = static_cast<int>(job.seconds / World::defaultTimeStep + 0.5);
        for (int i = 0; i < steps; ++i) {
            stepSimulation(_world);
        }
        [_scene viewPreset:job.view];
        [self refresh];
        [_scene homeCamera];
        if (job.view != 0) {
            [_scene viewPreset:job.view];
        }
        NSData* png = [[self windowImage] representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
        if (![png writeToFile:[NSString stringWithUTF8String:job.output.c_str()] atomically:YES]) {
            throw std::runtime_error("Could not write " + job.output);
        }
        if (job.labJoint >= 0) {
            [self openControlLab:nil];
            [_lab->_joint selectItemAtIndex:job.labJoint];
            [_lab loadFields:nil];
            [_lab refresh];
            NSView* content = _lab->_window.contentView;
            [content display];
            NSBitmapImageRep* bitmap = [content bitmapImageRepForCachingDisplayInRect:content.bounds];
            [_lab->_window.effectiveAppearance performAsCurrentDrawingAppearance:^{
                [content cacheDisplayInRect:content.bounds toBitmapImageRep:bitmap];
            }];
            NSData* lab = [bitmap representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
            std::string labPath = job.output.substr(0, job.output.size() - 4) + "-lab.png";
            if (![lab writeToFile:[NSString stringWithUTF8String:labPath.c_str()] atomically:YES]) {
                throw std::runtime_error("Could not write " + labPath);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "render failed: " << e.what() << '\n';
        std::exit(1);
    }
    std::exit(0);
}

@end
