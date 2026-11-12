// Plots window: see plot_window.h.

#import "mac/plot_window.h"
#import "mac/editor_controller.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <algorithm>
#include <cmath>
#include <fstream>

using namespace robotics;
using namespace editor_ui;

namespace {

constexpr int maxSeriesPerChart = 8;
constexpr CGFloat axisWidth = 64;
constexpr CGFloat rightMargin = 14;

// Categorical slots in fixed order, validated for color vision deficiency against the chart surface.
NSColor* seriesColor(int slot) {
    static const unsigned colors[maxSeriesPerChart] = {0x3987e5, 0xd95926, 0x199e70, 0xc98500,
                                                       0xd55181, 0x008300, 0x9085e9, 0xe66767};
    unsigned c = colors[slot % maxSeriesPerChart];
    return [NSColor colorWithSRGBRed:((c >> 16) & 255) / 255.0
                               green:((c >> 8) & 255) / 255.0
                                blue:(c & 255) / 255.0
                               alpha:1];
}

NSColor* chartSurface() {
    return [NSColor colorWithSRGBRed:0x1a / 255.0 green:0x1a / 255.0 blue:0x19 / 255.0 alpha:1];
}

NSString* text(const std::string& value) {
    return @(value.c_str());
}

NSString* formatValue(double value, const std::string& unit) {
    if (!std::isfinite(value)) {
        return @"–";
    }
    return unit.empty() ? [NSString stringWithFormat:@"%.4g", value]
                        : [NSString stringWithFormat:@"%.4g %@", value, text(unit)];
}

// Round grid spacing (1, 2 or 5 times a power of ten) near `rough`.
double niceStep(double rough) {
    double power = std::pow(10, std::floor(std::log10(rough)));
    double scaled = rough / power;
    return power * (scaled < 1.5 ? 1 : (scaled < 3.5 ? 2 : (scaled < 7.5 ? 5 : 10)));
}

NSDictionary* textStyle(CGFloat size, NSColor* color) {
    return @{
        NSFontAttributeName : [NSFont monospacedDigitSystemFontOfSize:size weight:NSFontWeightRegular],
        NSForegroundColorAttributeName : color
    };
}

} // namespace

@implementation PlotChartsView {
    NSTrackingArea* _tracking;
}

- (instancetype)initWithFrame:(NSRect)frame {
    self = [super initWithFrame:frame];
    _hoverX = -1;
    return self;
}

- (void)updateTrackingAreas {
    [super updateTrackingAreas];
    if (_tracking) {
        [self removeTrackingArea:_tracking];
    }
    _tracking = [[NSTrackingArea alloc] initWithRect:NSZeroRect
                                             options:NSTrackingMouseMoved | NSTrackingMouseEnteredAndExited
                                                     | NSTrackingActiveAlways | NSTrackingInVisibleRect
                                               owner:self
                                            userInfo:nil];
    [self addTrackingArea:_tracking];
}

- (void)mouseMoved:(NSEvent*)event {
    self.hoverX = [self convertPoint:event.locationInWindow fromView:nil].x;
    self.needsDisplay = YES;
}

- (void)mouseExited:(NSEvent*)event {
    (void)event;
    self.hoverX = -1;
    self.needsDisplay = YES;
}

- (void)seekTo:(NSEvent*)event {
    EditorController* editor = self.owner ? self.owner->_editor : nil;
    if (editor && editor->_reviewing) {
        CGFloat x = [self convertPoint:event.locationInWindow fromView:nil].x;
        [editor seekRecordingToTime:[self timeAtX:x]];
    }
}

- (void)mouseDown:(NSEvent*)event {
    [self seekTo:event];
}

- (void)mouseDragged:(NSEvent*)event {
    [self seekTo:event];
}

// Time span on screen. A review cursor outside it pulls the span along.
- (void)visibleStart:(double*)start end:(double*)end {
    PlotWindow* owner = self.owner;
    EditorController* editor = owner->_editor;
    const Recording& recording = editor->_recording;
    double last = owner->_paused ? owner->_pausedEnd : recording.endTime();
    double first = recording.startTime();
    NSInteger span = owner->_range.selectedTag;
    if (span > 0) {
        if (editor->_reviewing) {
            double cursor = recording.frame(editor->_reviewIndex).time;
            if (cursor < last - static_cast<double>(span) || cursor > last) {
                last = std::min(recording.endTime(), cursor + static_cast<double>(span) / 2);
            }
        }
        first = std::max(first, last - static_cast<double>(span));
    }
    *start = first;
    *end = std::max(last, first + 1e-3);
}

- (double)timeAtX:(CGFloat)x {
    double start;
    double end;
    [self visibleStart:&start end:&end];
    double width = self.bounds.size.width - axisWidth - rightMargin;
    return start + std::clamp((x - axisWidth) / width, 0.0, 1.0) * (end - start);
}

- (CGFloat)xAtTime:(double)time start:(double)start end:(double)end {
    return axisWidth + (time - start) / (end - start) * (self.bounds.size.width - axisWidth - rightMargin);
}

// Legend: a color key and the series name with its value at the cursor (or the newest value). Wraps.
- (CGFloat)drawLegend:(const PlotChart&)chart
                frame:(NSRect)frame
                frame:(const RecordingFrame*)sample
                model:(const World&)model {
    NSDictionary* style = textStyle(11, NSColor.labelColor);
    CGFloat x = frame.origin.x + axisWidth;
    CGFloat y = frame.origin.y + 6;
    for (const PlotSeries& series : chart.series) {
        double value = sample ? channelValue(series.channel, model, *sample) : NAN;
        NSString* entry = [NSString stringWithFormat:@"%@  %@", text(series.channel.label()),
                                                     formatValue(value, series.channel.unit)];
        CGFloat width = [entry sizeWithAttributes:style].width + 26;
        if (x + width > NSMaxX(frame) - rightMargin && x > frame.origin.x + axisWidth) {
            x = frame.origin.x + axisWidth;
            y += 16;
        }
        [seriesColor(series.slot) setFill];
        [[NSBezierPath bezierPathWithRoundedRect:NSMakeRect(x, y + 5, 14, 4) xRadius:2 yRadius:2] fill];
        [entry drawAtPoint:NSMakePoint(x + 19, y) withAttributes:style];
        x += width;
    }
    return y + 18 - frame.origin.y;
}

- (void)drawChart:(const PlotChart&)chart
            frame:(NSRect)frame
            start:(double)start
              end:(double)end
           cursor:(double)cursor
         showTime:(BOOL)showTime {
    EditorController* editor = self.owner->_editor;
    const Recording& recording = editor->_recording;
    const World& model = editor->_world;
    const RecordingFrame* sample = nullptr;
    if (!recording.empty()) {
        sample = &recording.frame(recording.indexAtTime(cursor));
    }
    CGFloat legend = [self drawLegend:chart frame:frame frame:sample model:model];
    NSRect area = NSMakeRect(frame.origin.x + axisWidth, frame.origin.y + legend + 2,
                             frame.size.width - axisWidth - rightMargin,
                             frame.size.height - legend - (showTime ? 24 : 8));
    if (area.size.height < 12 || recording.empty()) {
        return;
    }
    [chartSurface() setFill];
    NSRectFill(area);
    std::size_t first = recording.indexAtTime(start);
    std::size_t last = recording.indexAtTime(end);
    // At most about two samples per point of width: more cannot be seen.
    std::size_t stride = std::max<std::size_t>(
        1, (last - first) / std::max<std::size_t>(1, static_cast<std::size_t>(area.size.width * 2)));
    double lowest = INFINITY;
    double highest = -INFINITY;
    std::vector<std::vector<double>> values(chart.series.size());
    std::vector<double> times;
    for (std::size_t f = first; f <= last; f += stride) {
        const RecordingFrame& frameData = recording.frame(f);
        times.push_back(frameData.time);
        for (std::size_t s = 0; s < chart.series.size(); ++s) {
            double value = channelValue(chart.series[s].channel, model, frameData);
            values[s].push_back(value);
            if (std::isfinite(value)) {
                lowest = std::min(lowest, value);
                highest = std::max(highest, value);
            }
        }
    }
    if (!std::isfinite(lowest)) {
        return;
    }
    double padding = std::max((highest - lowest) * 0.08, std::max(std::abs(highest), 1.0) * 1e-3);
    lowest -= padding;
    highest += padding;
    auto yAt = [&](double value) {
        return NSMaxY(area) - (value - lowest) / (highest - lowest) * area.size.height;
    };
    NSDictionary* axisStyle = textStyle(10, NSColor.secondaryLabelColor);
    double step = niceStep((highest - lowest) / 4);
    for (double value = std::ceil(lowest / step) * step; value <= highest; value += step) {
        CGFloat y = std::floor(yAt(value)) + 0.5;
        [[NSColor colorWithCalibratedWhite:1 alpha:0.08] setStroke];
        [NSBezierPath strokeLineFromPoint:NSMakePoint(area.origin.x, y) toPoint:NSMakePoint(NSMaxX(area), y)];
        NSString* tick = [NSString stringWithFormat:@"%.4g", std::abs(value) < step * 1e-6 ? 0.0 : value];
        NSSize size = [tick sizeWithAttributes:axisStyle];
        [tick drawAtPoint:NSMakePoint(area.origin.x - size.width - 6, y - size.height / 2)
            withAttributes:axisStyle];
    }
    if (showTime) {
        for (int k = 0; k <= 5; ++k) {
            double time = start + (end - start) * k / 5;
            NSString* tick = [NSString stringWithFormat:@"%.1f s", time];
            NSSize size = [tick sizeWithAttributes:axisStyle];
            CGFloat x = [self xAtTime:time start:start end:end];
            [tick drawAtPoint:NSMakePoint(std::min(x - size.width / 2, NSMaxX(area) - size.width),
                                          NSMaxY(area) + 4)
                withAttributes:axisStyle];
        }
    }
    [NSGraphicsContext saveGraphicsState];
    NSRectClip(area);
    for (std::size_t s = 0; s < chart.series.size(); ++s) {
        NSBezierPath* line = [NSBezierPath bezierPath];
        line.lineWidth = 2;
        line.lineJoinStyle = NSLineJoinStyleRound;
        BOOL pen = NO;
        for (std::size_t k = 0; k < times.size(); ++k) {
            if (!std::isfinite(values[s][k])) {
                pen = NO; // A sensor without a reading leaves a gap.
                continue;
            }
            NSPoint p = NSMakePoint([self xAtTime:times[k] start:start end:end], yAt(values[s][k]));
            if (pen) {
                [line lineToPoint:p];
            } else {
                [line moveToPoint:p];
                pen = YES;
            }
        }
        [seriesColor(chart.series[s].slot) setStroke];
        [line stroke];
    }
    // Review cursor follows the timeline; the hover line follows the mouse.
    if (editor->_reviewing) {
        CGFloat x =
            std::floor([self xAtTime:recording.frame(editor->_reviewIndex).time start:start end:end]) + 0.5;
        [NSColor.systemYellowColor setStroke];
        NSBezierPath* line = [NSBezierPath bezierPath];
        line.lineWidth = 1.5;
        [line moveToPoint:NSMakePoint(x, area.origin.y)];
        [line lineToPoint:NSMakePoint(x, NSMaxY(area))];
        [line stroke];
    }
    if (self.hoverX >= axisWidth && sample) {
        CGFloat x = std::floor(self.hoverX) + 0.5;
        [[NSColor colorWithCalibratedWhite:1 alpha:0.55] setStroke];
        [NSBezierPath strokeLineFromPoint:NSMakePoint(x, area.origin.y) toPoint:NSMakePoint(x, NSMaxY(area))];
        for (const PlotSeries& series : chart.series) {
            double value = channelValue(series.channel, model, *sample);
            if (std::isfinite(value)) {
                NSRect dot = NSMakeRect(x - 4, yAt(value) - 4, 8, 8);
                [seriesColor(series.slot) setFill];
                NSBezierPath* marker = [NSBezierPath bezierPathWithOvalInRect:dot];
                [marker fill];
                [chartSurface() setStroke];
                marker.lineWidth = 2;
                [marker stroke];
            }
        }
    }
    [NSGraphicsContext restoreGraphicsState];
}

- (void)drawRect:(NSRect)dirty {
    (void)dirty;
    PlotWindow* owner = self.owner;
    NSRect b = self.bounds;
    [[NSColor colorWithCalibratedRed:0.105 green:0.115 blue:0.135 alpha:1] setFill];
    NSRectFill(b);
    EditorController* editor = owner->_editor;
    const std::vector<PlotChart>& charts = owner->_charts;
    if (charts.empty() || editor->_recording.size() < 2) {
        NSString* message = charts.empty() ? @"Choose quantities on the left and press Add."
                                           : @"Press Run in the main window to collect data.";
        NSDictionary* style = @{
            NSFontAttributeName : [NSFont systemFontOfSize:14],
            NSForegroundColorAttributeName : NSColor.secondaryLabelColor
        };
        NSSize size = [message sizeWithAttributes:style];
        [message drawAtPoint:NSMakePoint((b.size.width - size.width) / 2, b.size.height / 2 - size.height)
              withAttributes:style];
        if (charts.empty()) {
            return;
        }
    }
    double start;
    double end;
    [self visibleStart:&start end:&end];
    double cursor = end;
    if (editor->_reviewing) {
        cursor = editor->_recording.frame(editor->_reviewIndex).time;
    }
    if (self.hoverX >= axisWidth) {
        cursor = [self timeAtX:self.hoverX];
    }
    CGFloat height = b.size.height / static_cast<CGFloat>(charts.size());
    for (std::size_t k = 0; k < charts.size(); ++k) {
        [self drawChart:charts[k]
                  frame:NSMakeRect(0, height * static_cast<CGFloat>(k), b.size.width, height)
                  start:start
                    end:end
                 cursor:cursor
               showTime:k + 1 == charts.size()];
    }
}

@end

@implementation PlotWindow

- (instancetype)initWithEditor:(EditorController*)editor {
    self = [super init];
    if (!self) {
        return nil;
    }
    _editor = editor;
    _window =
        [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 1040, 680)
                                    styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                              | NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable
                                      backing:NSBackingStoreBuffered
                                        defer:NO];
    _window.title = @"Plots";
    _window.minSize = NSMakeSize(760, 460);
    _window.releasedWhenClosed = NO;
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    _window.acceptsMouseMovedEvents = YES;
    NSView* root = _window.contentView;
    NSRect b = root.bounds;
    const CGFloat side = 300;

    // Left column: search, available channels, plotted channels.
    NSView* left = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, side, b.size.height)];
    left.autoresizingMask = NSViewHeightSizable | NSViewMaxXMargin;
    [root addSubview:left];
    _search = [[NSSearchField alloc] initWithFrame:NSMakeRect(12, b.size.height - 40, side - 24, 28)];
    _search.placeholderString = @"Search, e.g. chassis speed";
    _search.delegate = self;
    _search.autoresizingMask = NSViewMinYMargin;
    _search.toolTip = @"Type parts of a name: every word must match";
    [left addSubview:_search];
    auto table = [&](NSRect frame, NSAutoresizingMaskOptions mask, NSString* identifier) {
        NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:frame];
        scroll.hasVerticalScroller = YES;
        scroll.autoresizingMask = mask;
        NSTableView* view = [[NSTableView alloc] initWithFrame:scroll.bounds];
        view.headerView = nil;
        view.style = NSTableViewStylePlain;
        view.rowHeight = 20;
        view.usesAlternatingRowBackgroundColors = NO;
        view.allowsMultipleSelection = YES;
        NSTableColumn* column = [[NSTableColumn alloc] initWithIdentifier:identifier];
        column.width = frame.size.width - 20;
        column.resizingMask = NSTableColumnAutoresizingMask;
        [view addTableColumn:column];
        view.dataSource = self;
        view.delegate = self;
        scroll.documentView = view;
        [left addSubview:scroll];
        return view;
    };
    const CGFloat activeHeight = 170;
    _available = table(NSMakeRect(12, activeHeight + 88, side - 24, b.size.height - activeHeight - 136),
                       NSViewHeightSizable, @"available");
    _available.target = self;
    _available.doubleAction = @selector(addSelected:);
    _available.toolTip = @"Double-click to plot";
    NSButton* add = pushButton(left, @"Add", NSMakeRect(12, activeHeight + 52, 134, 28), self,
                               @selector(addSelected:), @"plus");
    add.toolTip = @"Add to the chart with the same unit, or to a new chart";
    NSButton* addNew = pushButton(left, @"New Chart", NSMakeRect(154, activeHeight + 52, 134, 28), self,
                                  @selector(addSelectedToNewChart:), @"rectangle.split.1x2");
    addNew.toolTip = @"Put the selected quantities in a chart of their own";
    label(left, @"Plotted", NSMakeRect(14, activeHeight + 28, 200, 16), 11, NSFontWeightSemibold,
          NSColor.secondaryLabelColor);
    _active = table(NSMakeRect(12, 50, side - 24, activeHeight - 26), 0, @"active");
    NSButton* remove =
        pushButton(left, @"Remove", NSMakeRect(12, 12, 134, 28), self, @selector(removeSelected:), @"minus");
    remove.toolTip = @"Remove the selected quantities from the charts";
    NSButton* clear =
        pushButton(left, @"Remove All", NSMakeRect(154, 12, 134, 28), self, @selector(removeAll:), @"trash");
    clear.toolTip = @"Remove every chart";

    // Right: controls above the charts.
    NSView* bar =
        [[NSView alloc] initWithFrame:NSMakeRect(side, b.size.height - 48, b.size.width - side, 48)];
    bar.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
    [root addSubview:bar];
    _pause =
        pushButton(bar, @"Pause", NSMakeRect(4, 10, 96, 28), self, @selector(togglePause:), @"pause.fill");
    _pause.toolTip = @"Freeze the charts to inspect them. The simulation keeps running.";
    label(bar, @"Show", NSMakeRect(112, 15, 36, 18), 12);
    _range = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(148, 11, 116, 26) pullsDown:NO];
    for (NSArray* entry in
         @[ @[ @"Last 5 s", @5 ], @[ @"Last 10 s", @10 ], @[ @"Last 30 s", @30 ], @[ @"Everything", @0 ] ]) {
        [_range addItemWithTitle:entry[0]];
        _range.lastItem.tag = [entry[1] integerValue];
    }
    [_range selectItemAtIndex:3];
    _range.target = self;
    _range.action = @selector(rangeChanged:);
    _range.toolTip = @"Time span shown in the charts";
    [bar addSubview:_range];
    NSButton* csv = pushButton(bar, @"Export CSV…", NSMakeRect(bar.frame.size.width - 140, 10, 128, 28), self,
                               @selector(exportClicked:), @"square.and.arrow.up");
    csv.autoresizingMask = NSViewMinXMargin;
    csv.toolTip = @"Save the plotted quantities of the whole recording as a CSV table";
    _chartsView =
        [[PlotChartsView alloc] initWithFrame:NSMakeRect(side, 0, b.size.width - side, b.size.height - 48)];
    _chartsView.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    _chartsView.owner = self;
    _chartsView.toolTip = nil;
    [root addSubview:_chartsView];
    [self refresh];
    return self;
}

- (void)show {
    if (!_window.visible) {
        [_window center];
    }
    [self refresh];
    [_window makeKeyAndOrderFront:nil];
}

- (void)filterChanged {
    std::string query = _search.stringValue.UTF8String;
    _filtered.clear();
    for (const Channel& channel : _all) {
        if (channelMatches(channel, query)) {
            _filtered.push_back(channel);
        }
    }
    [_available reloadData];
}

- (void)controlTextDidChange:(NSNotification*)notification {
    (void)notification;
    [self filterChanged];
}

- (void)refresh {
    EditorController* editor = _editor;
    if (!editor) {
        return;
    }
    std::vector<Channel> channels = listChannels(editor->_world);
    bool same = channels.size() == _all.size();
    for (std::size_t k = 0; same && k < channels.size(); ++k) {
        same = channels[k].index == _all[k].index && channels[k].label() == _all[k].label();
    }
    if (same) {
        _chartsView.needsDisplay = YES;
        return;
    }
    _all = std::move(channels);
    [self filterChanged];
    for (PlotChart& chart : _charts) {
        std::erase_if(chart.series, [&](PlotSeries& s) { return !rebindChannel(s.channel, editor->_world); });
    }
    std::erase_if(_charts, [](const PlotChart& chart) { return chart.series.empty(); });
    [_active reloadData];
    _chartsView.needsDisplay = YES;
}

- (void)tick {
    if (_window.visible) {
        _chartsView.needsDisplay = YES;
    }
}

- (void)addChannels:(const std::vector<Channel>&)channels newChart:(BOOL)newChart {
    std::size_t target = _charts.size(); // Chart receiving series when forced; none exists yet.
    for (const Channel& channel : channels) {
        auto same = [&](const PlotSeries& s) {
            return s.channel.source == channel.source && s.channel.index == channel.index
                   && s.channel.quantity == channel.quantity;
        };
        bool plotted = std::any_of(_charts.begin(), _charts.end(), [&](const PlotChart& chart) {
            return std::any_of(chart.series.begin(), chart.series.end(), same);
        });
        if (plotted) {
            continue;
        }
        std::size_t index = _charts.size();
        if (newChart) {
            index = target;
            if (index < _charts.size() && _charts[index].series.size() >= maxSeriesPerChart) {
                index = target = _charts.size();
            }
        } else {
            for (std::size_t k = _charts.size(); k-- > 0;) {
                if (_charts[k].series.front().channel.unit == channel.unit
                    && _charts[k].series.size() < maxSeriesPerChart) {
                    index = k;
                    break;
                }
            }
        }
        if (index == _charts.size()) {
            _charts.emplace_back();
        }
        PlotChart& chart = _charts[index];
        int slot = 0;
        while (std::any_of(chart.series.begin(), chart.series.end(),
                           [&](const PlotSeries& s) { return s.slot == slot; })) {
            ++slot;
        }
        chart.series.push_back({channel, slot});
    }
    [_active reloadData];
    _chartsView.needsDisplay = YES;
}

- (std::vector<Channel>)chosenChannels {
    std::vector<Channel> chosen;
    NSIndexSet* rows = _available.selectedRowIndexes;
    for (NSUInteger row = rows.firstIndex; row != NSNotFound; row = [rows indexGreaterThanIndex:row]) {
        chosen.push_back(_filtered[row]);
    }
    return chosen;
}

- (void)addSelected:(id)sender {
    (void)sender;
    [self addChannels:[self chosenChannels] newChart:NO];
}

- (void)addSelectedToNewChart:(id)sender {
    (void)sender;
    [self addChannels:[self chosenChannels] newChart:YES];
}

- (std::vector<Channel>)activeChannels {
    std::vector<Channel> channels;
    for (const PlotChart& chart : _charts) {
        for (const PlotSeries& series : chart.series) {
            channels.push_back(series.channel);
        }
    }
    return channels;
}

- (void)removeSelected:(id)sender {
    (void)sender;
    NSIndexSet* rows = _active.selectedRowIndexes;
    NSInteger row = 0;
    for (PlotChart& chart : _charts) {
        std::vector<PlotSeries> kept;
        for (const PlotSeries& series : chart.series) {
            if (![rows containsIndex:static_cast<NSUInteger>(row++)]) {
                kept.push_back(series);
            }
        }
        chart.series = std::move(kept);
    }
    std::erase_if(_charts, [](const PlotChart& chart) { return chart.series.empty(); });
    [_active reloadData];
    _chartsView.needsDisplay = YES;
}

- (void)removeAll:(id)sender {
    (void)sender;
    _charts.clear();
    [_active reloadData];
    _chartsView.needsDisplay = YES;
}

- (void)togglePause:(id)sender {
    (void)sender;
    _paused = !_paused;
    EditorController* editor = _editor;
    _pausedEnd = editor->_recording.endTime();
    _pause.title = _paused ? @"Resume" : @"Pause";
    _pause.image = [NSImage imageWithSystemSymbolName:_paused ? @"play.fill" : @"pause.fill"
                             accessibilityDescription:nil];
    _chartsView.needsDisplay = YES;
}

- (void)rangeChanged:(id)sender {
    (void)sender;
    _chartsView.needsDisplay = YES;
}

- (BOOL)exportCsv:(NSString*)path {
    EditorController* editor = _editor;
    std::ofstream out(path.fileSystemRepresentation);
    writeChannelCsv(out, editor->_recording, editor->_world, [self activeChannels]);
    out.close();
    return static_cast<bool>(out);
}

- (void)exportClicked:(id)sender {
    (void)sender;
    EditorController* editor = _editor;
    if (_charts.empty() || editor->_recording.empty()) {
        NSBeep();
        return;
    }
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"plots.csv";
    panel.allowedContentTypes = @[ UTTypeCommaSeparatedText ];
    [panel beginSheetModalForWindow:_window
                  completionHandler:^(NSModalResponse response) {
                      if (response == NSModalResponseOK && ![self exportCsv:panel.URL.path]) {
                          NSBeep();
                      }
                  }];
}

- (void)setHoverX:(CGFloat)x {
    _chartsView.hoverX = x;
    _chartsView.needsDisplay = YES;
}

- (NSString*)hoverSummary {
    EditorController* editor = _editor;
    if (_chartsView.hoverX < 0 || editor->_recording.empty() || _charts.empty()) {
        return @"";
    }
    double time = [_chartsView timeAtX:_chartsView.hoverX];
    const RecordingFrame& frame = editor->_recording.frame(editor->_recording.indexAtTime(time));
    const PlotSeries& series = _charts.front().series.front();
    return [NSString stringWithFormat:@"%.2f s: %@", frame.time,
                                      formatValue(channelValue(series.channel, editor->_world, frame),
                                                  series.channel.unit)];
}

#pragma mark Tables

- (NSInteger)numberOfRowsInTableView:(NSTableView*)tableView {
    if (tableView == _available) {
        return static_cast<NSInteger>(_filtered.size());
    }
    return static_cast<NSInteger>([self activeChannels].size());
}

- (NSView*)tableView:(NSTableView*)tableView viewForTableColumn:(NSTableColumn*)column row:(NSInteger)row {
    (void)column;
    NSTextField* field = [NSTextField labelWithString:@""];
    field.font = [NSFont systemFontOfSize:11.5];
    field.lineBreakMode = NSLineBreakByTruncatingMiddle;
    if (tableView == _available) {
        const Channel& channel = _filtered[static_cast<std::size_t>(row)];
        field.stringValue =
            channel.unit.empty()
                ? text(channel.label())
                : [NSString stringWithFormat:@"%@ (%@)", text(channel.label()), text(channel.unit)];
        field.toolTip = field.stringValue;
        return field;
    }
    NSInteger index = row;
    NSView* cell = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 260, 20)];
    for (std::size_t c = 0; c < _charts.size(); ++c) {
        for (const PlotSeries& series : _charts[c].series) {
            if (index-- == 0) {
                NSView* key = [[NSView alloc] initWithFrame:NSMakeRect(4, 8, 14, 4)];
                key.wantsLayer = YES;
                key.layer.cornerRadius = 2;
                key.layer.backgroundColor = seriesColor(series.slot).CGColor;
                [cell addSubview:key];
                field.stringValue =
                    [NSString stringWithFormat:@"%zu: %@", c + 1, text(series.channel.label())];
                field.frame = NSMakeRect(24, 2, 236, 16);
                field.autoresizingMask = NSViewWidthSizable;
                field.toolTip = field.stringValue;
                [cell addSubview:field];
                return cell;
            }
        }
    }
    return cell;
}

@end
