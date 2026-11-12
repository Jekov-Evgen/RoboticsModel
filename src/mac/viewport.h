#pragma once
#import <Cocoa/Cocoa.h>
#import <MetalKit/MetalKit.h>
#include "robotics/editing/gizmo.hpp"
#include "robotics/world.hpp"

#include <vector>

// The editor owns the world; the viewport only reads it and reports user intent.
@protocol SceneViewHost <NSObject>
- (const robotics::World*)sceneWorld;
- (int)sceneSelected;
- (BOOL)sceneRunning;
- (void)scenePicked:(int)body;      // -1: empty space
- (void)sceneKey:(NSString*)action; // run, w, a, s, d, x, delete, frame-back, frame-forward, live
- (BOOL)sceneBeginMove:(int)body;
- (void)sceneMoveBy:(robotics::Vec3)delta;
- (void)sceneEndMove;
// Transform gizmos. Gizmos show only while this is YES (paused, a part selected, no recording on screen).
- (BOOL)sceneGizmoAvailable;
- (BOOL)sceneBeginRotate;
// Total rotation since sceneBeginRotate about the line through pivot.
- (void)sceneRotateAxis:(robotics::Vec3)axis pivot:(robotics::Vec3)pivot angle:(double)angle;
- (void)sceneDropToFloor;
- (NSMenu*)sceneContextMenuForBody:(int)body; // Right-click in 3D; body -1 for empty space. May return nil.
@end

// GPU viewport: Metal, reversed-Z depth buffer, 4x MSAA, planar shadows.
@interface SceneView : MTKView <MTKViewDelegate>
@property (nonatomic, weak) id<SceneViewHost> host;
@property (nonatomic) BOOL follow;
// Draws collision primitives as translucent overlays where visual meshes (or hidden placeholders) replace
// them.
@property (nonatomic) BOOL showCollision;
// Transform gizmo tool and snapping (0 = off; metres and radians).
@property (nonatomic) robotics::GizmoTool gizmoTool;
@property (nonatomic) double moveSnap;
@property (nonatomic) double rotateSnap;
@property (nonatomic, readonly) robotics::GizmoHandle hoveredHandle;
@property (nonatomic, readonly) NSString* gizmoReadout; // Text shown beside the mouse during a drag.
- (void)homeCamera;                                     // Default view; small robots are framed to fit.
- (void)frameWorld;                                     // Fits the whole scene into the view.
- (void)reloadMeshes; // Forgets cached visual meshes (and reported load errors), e.g. after files changed.
- (void)viewPreset:(NSInteger)preset; // 0 perspective, 1 top, 2 front, 3 side
- (void)setStatus:(NSString*)state detail:(NSString*)detail running:(BOOL)running;
- (void)showToast:(NSString*)text error:(BOOL)error;
- (NSBitmapImageRep*)snapshot;    // Offscreen render at backing resolution.
- (robotics::GizmoView)gizmoView; // The current camera, for gizmo math.
- (robotics::Vec3)cameraTarget;
// Keeps the view where it is after the followed robot was edited: only a run moves the camera along.
- (void)holdCamera;
// Small render of a standalone world (library thumbnails), framed to fit, without gizmos.
- (NSImage*)thumbnailOfWorld:(const robotics::World&)world width:(int)width height:(int)height;
// Renders a camera sensor (pose, vertical FOV, width x height) without gizmos. rgba: width*height*4 bytes,
// top row first; depth: metric distance along the optical axis (m), maxRange where nothing was drawn.
- (BOOL)renderSensorCamera:(const robotics::Sensor&)sensor
                     color:(std::vector<unsigned char>&)rgba
                     depth:(std::vector<float>&)depth;
// Shows (or with nil hides) the camera picture-in-picture panel.
- (void)showCameraColor:(NSImage*)color depth:(NSImage*)depth title:(NSString*)title;
@end
