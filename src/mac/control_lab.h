#pragma once
// Control Lab window: per-joint (hinge or slider) controller settings and live response plots.

#import <Cocoa/Cocoa.h>

@class EditorController;
@class ControlPlotView;

// Instance variables are @public for the editor smoke test (src/mac/editor_smoke_test.mm).
@interface ControlLab : NSObject {
  @public
    __weak EditorController* _editor;
    NSWindow* _window;
    ControlPlotView* _plot;
    NSPopUpButton* _joint;
    NSPopUpButton* _kind;
    NSPopUpButton* _variable;
    NSPopUpButton* _signal;
    NSArray<NSTextField*>* _gains;
    NSTextField* _target;
    NSTextField* _amplitude;
    NSTextField* _frequency;
    NSTextField* _start;
    NSTextField* _limit;
    NSTextField* _limitLabel;
    NSTextField* _integralLimit;
    NSTextField* _integralLimitLabel;
    NSTextField* _filter;
    NSTextField* _deadband;
    NSTextField* _units;
    NSTextField* _info;
    // Feedback source: ideal joint measurement or the joint's encoder sensor (noise, latency, counts, rate).
    NSPopUpButton* _feedback;
    NSTextField* _encoderNoise;
    NSTextField* _encoderLatency;
    NSTextField* _encoderCounts;
    NSTextField* _encoderRate;
}
- (instancetype)initWithEditor:(EditorController*)editor;
- (int)jointIndex; // Selected joint index in the world, or -1.
- (void)show;
- (void)refresh;
- (void)loadFields:(id)sender;
- (void)updateEnabled:(id)sender;
- (void)apply:(id)sender;
- (void)demo:(id)sender;
- (void)tick;
@end
