#pragma once
// Main editor window controller. The implementation is split by concern into categories:
//   editor_controller.mm   lifecycle, state, undo, feedback, viewport host, simulation loop
//   editor_actions.mm      add/duplicate/delete parts, examples, driving, camera
//   editor_documents.mm    open/save/close
//   editor_inspector.mm    inspector panel (part and joint editing)
//   editor_menus.mm        menu bar, toolbar, validation
//   editor_parts_list.mm   parts outline (assembly tree)
//   editor_sensors.mm      sensors: inspector card, camera picture-in-picture, sensor log, sensor demos
//   editor_timeline.mm     recording, timeline bar, reviewing and resuming frames
//   editor_library.mm      part library actions, plot shortcuts, gizmo host, context menus
//   editor_smoke_test.mm   --smoke-test self check
//   editor_smoke_ux.mm     --smoke-test: gizmos, timeline, plots, library

#import <Cocoa/Cocoa.h>

#import "mac/ui_helpers.h"
#import "mac/viewport.h"

#include "robotics/editing/library.hpp"
#include "robotics/replay/channels.hpp"
#include "robotics/replay/recording.hpp"
#include "robotics/world.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

@class ControlLab;
@class LibraryPanel;
@class PlotWindow;
@class SensorPanel;
@class TimelineBar;

// True when started with --smoke-test: errors throw instead of showing toasts, no modal dialogs.
bool isEditorSmokeTest();
void setEditorSmokeTest(bool enabled);
// --render: load `scene`, simulate `seconds`, show camera preset `view` (0 perspective, 1 top, 2 front,
// 3 side), write the window to `output` (PNG) and exit. Active when `scene` is non-empty.
struct EditorRenderJob {
    std::string scene;
    std::string output;
    double seconds{};
    int view{};
    int labJoint{-1}; // >= 0: also write <output>-lab.png with the Control Lab plots of this joint.
};
void setEditorRenderJob(const char* scene, const char* output, double seconds, int view, int labJoint);
const EditorRenderJob& editorRenderJob();

// Instance variables are @public so the editor categories, Control Lab and smoke test can reach them.
@interface EditorController : NSObject <NSApplicationDelegate, NSWindowDelegate, SceneViewHost> {
  @public
    // Model
    robotics::World _world;
    robotics::World _snapshot; // State before Run; restored by Reset.
    int _selected;
    int _inspectorBody;
    BOOL _running;
    BOOL _hasSnapshot;
    BOOL _closing;
    // Window and services
    NSWindow* _window;
    SceneView* _scene;
    ControlLab* _lab;
    NSUndoManager* _undo;
    NSUndoManager* _textUndo;
    NSURL* _fileURL;
    // Parts list
    NSOutlineView* _outline;
    NSTextField* _partsCount;
    std::vector<int> _roots;
    std::vector<std::vector<int>> _children;
    BOOL _syncingSelection;
    // Inspector
    FlippedView* _inspector;
    NSScrollView* _inspectorScroll;
    CardView* _placeholderCard;
    CardView* _linkCard;
    CardView* _jointCard;
    CardView* _simCard;
    NSImageView* _linkIcon;
    NSTextField* _name;
    NSTextField* _linkKind;
    NSArray<NSTextField*>* _position;
    NSArray<NSTextField*>* _rotation;
    NSArray<NSTextField*>* _size;
    NSArray<NSTextField*>* _sizeLetters;
    NSTextField* _sizeTitle;
    NSTextField* _mass;
    NSButton* _static;
    NSTextField* _linkHint;
    NSString* _shownLinkValues;
    NSPopUpButton* _parentList;
    NSSegmentedControl* _jointType;
    NSTextField* _jointTypeHelp;
    NSArray<NSTextField*>* _anchor;
    NSArray<NSTextField*>* _axis;
    NSTextField* _axisTitle;
    NSSegmentedControl* _axisPreset;
    NSButton* _midpoint;
    NSTextField* _speedLabel;
    NSTextField* _torqueLabel;
    NSTextField* _speed;
    NSTextField* _torque;
    NSButton* _limitsEnabled;
    NSTextField* _lowerLabel;
    NSTextField* _upperLabel;
    NSTextField* _lowerLimit;
    NSTextField* _upperLimit;
    robotics::JointType _shownJointType; // Joint type whose units the limit fields currently use.
    NSTextField* _jointHint;
    NSButton* _jointApply;
    NSButton* _jointRemove;
    // Sensors (inspector card)
    CardView* _sensorCard;
    NSPopUpButton* _sensorAdd;
    NSPopUpButton* _sensorList;
    NSButton* _sensorRemove;
    NSTextField* _sensorName;
    NSArray<NSTextField*>* _sensorFields; // Generic number fields; meaning depends on the sensor type.
    NSArray<NSTextField*>* _sensorLabels;
    NSArray<NSTextField*>* _sensorMount;
    NSArray<NSTextField*>* _sensorMountRotation;
    NSButton* _sensorFeeds;
    NSTextField* _sensorHint;
    NSString* _shownSensorValues;
    int _shownSensor; // Index into _world.sensors shown in the card, or -1.
    // Sensors (live data)
    SensorPanel* _sensorPanel;
    std::unique_ptr<std::ofstream> _sensorLog; // Recording to CSV while running (Sensors window).
    std::vector<unsigned char> _cameraColor;   // Last rendered camera sensor image (RGBA) and depth (m).
    std::vector<float> _cameraDepth;
    int _cameraSensor; // Sensor index of the last rendered camera, or -1.
    // Recording and timeline
    robotics::Recording _recording;
    robotics::World _reviewWorld; // Shown instead of _world while reviewing a recorded frame.
    BOOL _reviewing;
    BOOL _reviewPlaying;
    std::size_t _reviewIndex;
    double _reviewClock; // Playback time (s).
    double _reviewSpeed;
    TimelineBar* _timeline;
    PlotWindow* _plots;
    // Rotate gizmo: pose at the start of the drag; every update rotates from here.
    std::vector<robotics::Body> _rotateBodies;
    std::vector<robotics::Joint> _rotateJoints;
    NSMenu* _inspectorPlotMenu;
    NSMenu* _contextMenu; // Last right-click menu of the 3D view.
    // Part library
    LibraryPanel* _library;
    NSString* _libraryFolder; // Where custom items are saved.
    // Status
    NSTextField* _statusLeft;
    NSTextField* _statusRight;
    NSToolbarItem* _runItem;
    NSTimer* _timer;
    double _accumulator;
    std::chrono::steady_clock::time_point _previous;
}
// Undo
- (void)undo:(id)sender;
- (void)redo:(id)sender;
- (void)undoAction:(id)sender;
- (void)redoAction:(id)sender;
- (void)recordUndo:(NSString*)name;
- (void)restoreWorld:(const robotics::World&)world selected:(int)selected name:(NSString*)name;
// Feedback
- (void)showError:(NSString*)message;
- (void)toast:(NSString*)message;
- (BOOL)canEdit;
- (void)showHelp:(id)sender;
// State
- (void)invalidateSnapshot;
- (void)selectBody:(int)index;
- (void)refresh;
- (void)updateStatus;
// Simulation
- (void)toggleRun:(id)sender;
- (void)reset:(id)sender;
- (void)tick:(NSTimer*)timer;
@end

@interface EditorController (Actions)
- (void)openControlLab:(id)sender;
- (void)homeCamera:(id)sender;
- (void)cameraPreset:(NSMenuItem*)sender;
- (void)loadWorld:(robotics::World)world selected:(int)selected name:(NSString*)name;
- (void)newScene:(id)sender;
- (void)demo:(id)sender;
- (void)rover:(id)sender;
- (void)pidDemo:(id)sender;
- (void)actuatorDemo:(id)sender;
- (void)drive:(NSString*)key;
- (void)addShape:(robotics::Shape)shape;
- (void)addBox:(id)sender;
- (void)addSphere:(id)sender;
- (void)addCylinder:(id)sender;
- (void)duplicateBody:(id)sender;
- (void)deleteBody:(id)sender;
@end

@interface EditorController (Documents)
- (BOOL)writeTo:(NSURL*)url;
- (BOOL)saveAsReturningSuccess;
- (void)saveAs:(id)sender;
- (void)save:(id)sender;
- (void)load:(id)sender;
- (BOOL)confirmDiscard;
@end

@interface EditorController (Inspector)
- (void)buildInspector;
- (void)layoutInspector;
- (int)parentIndex;
- (void)refreshInspector;
- (NSString*)linkFieldSignature;
- (void)updateEnabledState;
- (int)existingJoint;
- (void)toggleStatic:(id)sender;
- (void)applyBody:(id)sender;
- (void)chooseParent:(id)sender;
- (void)jointTypeChanged:(id)sender;
- (void)toggleLimits:(id)sender;
- (void)axisPreset:(NSSegmentedControl*)sender;
- (void)useMidpoint:(id)sender;
- (void)applyJoint:(id)sender;
- (void)removeJoint:(id)sender;
@end

@interface EditorController (Menus) <NSToolbarDelegate, NSMenuDelegate, NSMenuItemValidation,
                                     NSToolbarItemValidation>
- (NSMenuItem*)item:(NSMenu*)menu
              title:(NSString*)title
             action:(SEL)action
                key:(NSString*)key
               mods:(NSEventModifierFlags)mods
             target:(id)target;
- (void)buildMenu;
- (BOOL)canPerform:(SEL)action;
@end

@interface EditorController (PartsList) <NSOutlineViewDataSource, NSOutlineViewDelegate>
- (void)rebuildTree;
@end

@interface EditorController (Import)
- (NSString*)examplesFolder;
- (std::vector<std::string>)importUrdfAtPath:(NSString*)path fixBase:(BOOL)fixBase; // Throws on errors.
- (void)importUrdfFile:(NSString*)path fixBase:(BOOL)fixBase; // Shows warnings/errors to the user.
- (void)importUrdf:(id)sender;
- (void)diffDriveExample:(id)sender;
- (void)meshArmExample:(id)sender;
- (void)urdfArmExample:(id)sender;
- (void)gripperExample:(id)sender;
- (void)attachVisualModel:(NSString*)path fit:(NSInteger)fit; // fit: 0 to part, 1 metres, 2 mm. Throws.
- (void)importVisualModel:(id)sender;
- (void)removeVisualModel:(id)sender;
- (void)toggleCollisionShapes:(id)sender;
- (void)checkUrdfImport;
@end

@interface EditorController (Sensors)
- (void)buildSensorCard;
- (void)refreshSensorCard;
- (void)addSensor:(NSMenuItem*)sender;
- (void)chooseSensor:(id)sender;
- (void)applySensor:(id)sender;
- (void)removeSensor:(id)sender;
- (void)openSensors:(id)sender;
- (void)sensorRover:(id)sender;
- (void)noisyPidDemo:(id)sender;
- (void)afterSimulationStep; // Sensor log rows; called after every physics step while running.
- (void)updateSensorViews;   // Camera picture-in-picture and Sensors window, once per frame.
- (BOOL)startSensorLog:(NSString*)path;
- (void)stopSensorLog;
@end

@interface EditorController (Timeline)
- (void)buildTimeline;
- (void)startRecording;   // Clears the recording and stores the first frame.
- (void)discardRecording; // After model edits: frames no longer match the robot.
- (void)leaveReview;
- (void)updateTimeline;
- (void)showFrame:(std::size_t)index;
- (void)timelineScrubTo:(double)fraction; // 0..1 of the recording.
- (void)seekRecordingToTime:(double)time;
- (void)stepFrames:(int)delta;
- (void)timelineStepBack:(id)sender;
- (void)timelineStepForward:(id)sender;
- (void)timelinePlay:(id)sender;
- (void)timelineSpeed:(NSPopUpButton*)sender;
- (void)timelineLive:(id)sender;
- (void)timelineResume:(id)sender;
- (void)clearRecording:(id)sender;
- (void)setRecordingLength:(NSMenuItem*)sender;
- (BOOL)writeRecordingCsv:(NSString*)path;
- (void)exportRecording:(id)sender;
- (void)tickReview:(double)elapsed;
@end

@interface EditorController (Library)
- (void)openLibrary:(id)sender;
- (void)openPlots:(id)sender;
- (NSString*)customLibraryFolder;
- (std::vector<robotics::LibraryItem>)libraryItems; // Built-in, then saved items.
- (BOOL)insertItem:(const robotics::LibraryItem&)item attach:(BOOL)attach;
- (BOOL)saveAssemblyNamed:(NSString*)name; // Throws on file errors.
- (void)saveAssemblyToLibrary:(id)sender;
- (void)dropToFloor:(id)sender;
- (void)selectTool:(NSMenuItem*)sender;
- (NSMenu*)plotMenuForBody:(int)body;
- (void)plotBodyQuantity:(NSMenuItem*)sender;
- (void)plotJointQuantity:(NSMenuItem*)sender;
@end

@interface EditorController (SmokeTest)
- (NSBitmapImageRep*)windowImage;
- (void)smokeCheck;
- (BOOL)writeLabImage:(NSString*)path;
- (void)checkSliderInspector;
- (void)checkLinearActuatorDemo;
- (void)checkSensors;
@end

// Gizmos, timeline, plots and library: driven through real NSEvents where possible (editor_smoke_ux.mm).
@interface EditorController (SmokeTestUX)
- (void)checkEditorUX;
@end

// `--render` screenshot job (editor_render_job.mm).
@interface EditorController (RenderJob)
- (void)renderJob;
@end
