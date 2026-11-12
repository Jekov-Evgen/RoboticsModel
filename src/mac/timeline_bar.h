#pragma once
// Timeline bar at the bottom of the 3D view: transport buttons, a scrub track and recording actions.
// Holds no state of its own beyond what it shows; the editor drives it (see editor_timeline.mm).

#import <Cocoa/Cocoa.h>

@class EditorController;

// The scrub track: reports mouse positions as a fraction of the recording.
@interface TimelineTrack : NSView
@property (nonatomic, weak) EditorController* editor;
@property (nonatomic) double position; // 0..1, thumb position.
@property (nonatomic) BOOL hasRecording;
@property (nonatomic) BOOL live; // The thumb stands for the running simulation.
@end

// Instance variables are @public for the editor smoke test (src/mac/editor_smoke_test.mm).
@interface TimelineBar : NSVisualEffectView {
  @public
    TimelineTrack* _track;
    NSButton* _stepBack;
    NSButton* _play;
    NSButton* _stepForward;
    NSTextField* _time;
    NSPopUpButton* _speed;
    NSButton* _resume;
    NSButton* _live;
    NSPopUpButton* _more;
}
- (instancetype)initWithFrame:(NSRect)frame editor:(EditorController*)editor;
@end
