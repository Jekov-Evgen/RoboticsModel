#pragma once
// Plots window: pick any quantity of the simulation from a searchable list and see it in stacked charts.
// Reads from the editor's recording, so plots cover everything the timeline can show; while reviewing a
// recording a cursor line follows the frame on screen.

#import <Cocoa/Cocoa.h>

#import "mac/ui_helpers.h"

#include "robotics/replay/channels.hpp"

#include <vector>

@class EditorController;
@class PlotWindow;

// The stacked charts.
@interface PlotChartsView : FlippedView
@property (nonatomic, weak) PlotWindow* owner;
@property (nonatomic) CGFloat hoverX; // Negative: no hover.
- (double)timeAtX:(CGFloat)x;
- (void)visibleStart:(double*)start end:(double*)end;
@end

struct PlotSeries {
    robotics::Channel channel;
    int slot{}; // Color slot; stays with the series while others are added or removed.
};

// One chart has one y axis, so its series share a unit.
struct PlotChart {
    std::vector<PlotSeries> series;
};

// Instance variables are @public for the editor smoke test (src/mac/editor_smoke_test.mm).
@interface PlotWindow : NSObject <NSTableViewDataSource, NSTableViewDelegate, NSSearchFieldDelegate> {
  @public
    __weak EditorController* _editor;
    NSWindow* _window;
    NSSearchField* _search;
    NSTableView* _available;
    NSTableView* _active;
    PlotChartsView* _chartsView;
    NSButton* _pause;
    NSPopUpButton* _range;
    std::vector<robotics::Channel> _all;
    std::vector<robotics::Channel> _filtered;
    std::vector<PlotChart> _charts;
    BOOL _paused;
    double _pausedEnd;
}
- (instancetype)initWithEditor:(EditorController*)editor;
- (void)show;
- (void)refresh; // Channels after edits or loading another robot.
- (void)tick;    // New data or a moved cursor.
- (void)filterChanged;
// Adds series: to the last chart with the same unit, or to a new chart. newChart forces one new chart for
// all.
- (void)addChannels:(const std::vector<robotics::Channel>&)channels newChart:(BOOL)newChart;
- (std::vector<robotics::Channel>)activeChannels;
- (void)removeAll:(id)sender;
- (void)togglePause:(id)sender;
- (BOOL)exportCsv:(NSString*)path;
- (void)setHoverX:(CGFloat)x; // View x of the mouse in the charts, or negative for none.
- (NSString*)hoverSummary;    // Text of the cursor readout, for tests.
@end
