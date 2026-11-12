#pragma once
// Sensors window: live readings of one sensor, a scrolling plot of a chosen channel (polar scan for lidars,
// images for cameras), CSV recording and export.

#import <Cocoa/Cocoa.h>

@class EditorController;
@class SensorPlotView;

// Instance variables are @public for the editor smoke test (src/mac/editor_smoke_test.mm).
@interface SensorPanel : NSObject {
  @public
    __weak EditorController* _editor;
    NSWindow* _window;
    NSPopUpButton* _sensor;
    NSPopUpButton* _channel;
    NSTextField* _readings;
    NSButton* _record;
    SensorPlotView* _plot;
}
- (instancetype)initWithEditor:(EditorController*)editor;
- (int)sensorIndex; // Selected index into World::sensors, or -1.
- (int)channelIndex;
- (void)show;
- (void)refresh;                       // Rebuilds the sensor list (after edits or loading another robot).
- (void)tick;                          // Updates numbers and plot while visible.
- (BOOL)exportHistory:(NSString*)path; // Writes the plot history of every sensor as CSV.
@end
