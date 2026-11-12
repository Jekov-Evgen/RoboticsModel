// EditorController (Timeline): recording while simulating, reviewing frames, resuming from a frame.
//
// _world is never touched while reviewing: the viewport shows _reviewWorld (see -sceneWorld) until the user
// returns to live or resumes from a frame.

#import "mac/control_lab.h"
#import "mac/editor_controller.h"
#import "mac/plot_window.h"
#import "mac/timeline_bar.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include "robotics/replay/channels.hpp"

#include <algorithm>
#include <fstream>

using namespace robotics;
using namespace editor_ui;

@implementation EditorController (Timeline)

- (void)buildTimeline {
    NSRect b = _scene.bounds;
    _timeline = [[TimelineBar alloc] initWithFrame:NSMakeRect(14, 12, b.size.width - 28, 40) editor:self];
    _timeline.autoresizingMask = NSViewWidthSizable | NSViewMaxYMargin;
    [_scene addSubview:_timeline];
    _reviewSpeed = 1;
    [self updateTimeline];
}

- (void)startRecording {
    [self leaveReview];
    _recording.start(_world);
    [self updateTimeline];
}

- (void)discardRecording {
    if (_recording.empty() && !_reviewing) {
        return;
    }
    _recording.clear();
    _reviewing = NO;
    _reviewPlaying = NO;
    [self updateTimeline];
}

- (void)leaveReview {
    _reviewing = NO;
    _reviewPlaying = NO;
}

- (void)updateTimeline {
    TimelineBar* bar = _timeline;
    BOOL has = _recording.size() > 1;
    double span = _recording.endTime() - _recording.startTime();
    bar->_track.hasRecording = has;
    bar->_track.live = !_reviewing;
    bar->_track.position =
        _reviewing && span > 0 ? (_recording.frame(_reviewIndex).time - _recording.startTime()) / span : 1;
    if (_reviewing) {
        bar->_time.stringValue = [NSString
            stringWithFormat:@"%.2f / %.2f s", _recording.frame(_reviewIndex).time, _recording.endTime()];
    } else if (has) {
        bar->_time.stringValue =
            [NSString stringWithFormat:@"%@ %.1f s", _running ? @"● REC" : @"Recorded", span];
    } else {
        bar->_time.stringValue = @"Run to record";
    }
    bar->_stepBack.enabled = has;
    bar->_stepForward.enabled = has;
    bar->_play.enabled = has;
    bar->_play.image = [NSImage imageWithSystemSymbolName:_reviewPlaying ? @"pause.fill" : @"play.fill"
                                 accessibilityDescription:_reviewPlaying ? @"Pause" : @"Play"];
    bar->_resume.enabled = _reviewing;
    bar->_live.enabled = _reviewing;
}

// Shows a frame, entering review (and pausing the simulation) if needed.
- (void)showFrame:(std::size_t)index {
    if (_recording.empty()) {
        return;
    }
    BOOL entering = !_reviewing;
    if (entering) {
        _running = NO;
        _accumulator = 0;
        _reviewing = YES;
    }
    _reviewIndex = std::min(index, _recording.size() - 1);
    _reviewClock = _recording.frame(_reviewIndex).time;
    _reviewWorld = _recording.view(_reviewIndex);
    if (entering) {
        [self refresh]; // Inspector locks, toolbar shows Run.
    } else {
        [self updateStatus];
        [self updateTimeline];
        [self updateSensorViews];
        _scene.needsDisplay = YES;
    }
    if (_selected >= 0 && _selected < static_cast<int>(_reviewWorld.bodies.size())) {
        const Body& body = _reviewWorld.bodies[static_cast<std::size_t>(_selected)];
        setVector(_position, body.position); // Locked fields follow the frame on screen.
        setVector(_rotation, body.orientation.eulerAngles() * (180 / robotics::pi));
    }
    [_plots tick];
}

- (void)timelineScrubTo:(double)fraction {
    if (_recording.size() < 2) {
        return;
    }
    _reviewPlaying = NO;
    double time = _recording.startTime() + fraction * (_recording.endTime() - _recording.startTime());
    [self showFrame:_recording.indexAtTime(time)];
}

- (void)seekRecordingToTime:(double)time {
    if (!_recording.empty()) {
        _reviewPlaying = NO;
        [self showFrame:_recording.indexAtTime(time)];
    }
}

- (void)stepFrames:(int)delta {
    if (_recording.size() < 2) {
        return;
    }
    _reviewPlaying = NO;
    std::size_t last = _recording.size() - 1;
    std::size_t current = _reviewing ? _reviewIndex : last;
    [self showFrame:delta < 0 ? (current > 0 ? current - 1 : 0) : std::min(current + 1, last)];
}

- (void)timelineStepBack:(id)sender {
    (void)sender;
    [self stepFrames:-1];
}

- (void)timelineStepForward:(id)sender {
    (void)sender;
    [self stepFrames:1];
}

- (void)timelinePlay:(id)sender {
    (void)sender;
    if (_recording.size() < 2) {
        return;
    }
    if (_running) { // Stop at the newest frame; the next click plays.
        [self showFrame:_recording.size() - 1];
        return;
    }
    if (_reviewPlaying) {
        _reviewPlaying = NO;
    } else {
        BOOL atEnd = !_reviewing || _reviewIndex + 1 >= _recording.size();
        [self showFrame:atEnd ? 0 : _reviewIndex];
        _reviewPlaying = YES;
    }
    [self updateTimeline];
}

- (void)timelineSpeed:(NSPopUpButton*)sender {
    constexpr double speeds[] = {0.25, 0.5, 1, 2, 4};
    _reviewSpeed = speeds[sender.indexOfSelectedItem];
    [_window makeFirstResponder:_scene];
}

- (void)timelineLive:(id)sender {
    (void)sender;
    if (!_reviewing) {
        return;
    }
    [self leaveReview];
    [self refresh];
    [_plots tick];
}

- (void)timelineResume:(id)sender {
    (void)sender;
    if (!_reviewing) {
        return;
    }
    try {
        double time = _recording.frame(_reviewIndex).time;
        World restored = _recording.restore(_reviewIndex, _world);
        _recording.truncateAfter(_reviewIndex);
        _world = std::move(restored);
        [self leaveReview];
        _hasSnapshot = YES; // Reset still returns to the start of the run.
        _running = YES;
        _accumulator = 0;
        _previous = std::chrono::steady_clock::now();
        [self refresh];
        [_scene showToast:[NSString
                              stringWithFormat:@"Resumed from t = %.2f s; later frames were discarded.", time]
                    error:NO];
    } catch (const std::exception& e) {
        [self showError:[NSString stringWithFormat:@"Could not resume: %s", e.what()]];
    }
}

- (void)clearRecording:(id)sender {
    (void)sender;
    BOOL reviewing = _reviewing;
    [self discardRecording];
    if (_running) {
        _recording.start(_world);
    }
    if (reviewing) {
        [self refresh];
    }
    [_plots tick];
}

- (void)setRecordingLength:(NSMenuItem*)sender {
    _reviewPlaying = NO;
    if (_reviewing) { // Trimming may remove the shown frame.
        [self leaveReview];
        [self refresh];
    }
    _recording.setDuration(static_cast<double>(sender.tag));
    [self updateTimeline];
    [self toast:[NSString stringWithFormat:@"Keeping the last %ld s of the simulation.", sender.tag]];
}

- (BOOL)writeRecordingCsv:(NSString*)path {
    std::ofstream out(path.fileSystemRepresentation);
    writeChannelCsv(out, _recording, _world, listChannels(_world));
    out.close();
    return static_cast<bool>(out);
}

- (void)exportRecording:(id)sender {
    (void)sender;
    if (_recording.empty()) {
        [self showError:@"Nothing recorded yet. Press Run first."];
        return;
    }
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"recording.csv";
    panel.allowedContentTypes = @[ UTTypeCommaSeparatedText ];
    if ([panel runModal] == NSModalResponseOK) {
        if ([self writeRecordingCsv:panel.URL.path]) {
            [self toast:[NSString stringWithFormat:@"Exported %zu frames to %@", _recording.size(),
                                                   panel.URL.lastPathComponent]];
        } else {
            [self showError:@"Could not write the CSV file."];
        }
    }
}

// Advances playback; called from the run loop while reviewing.
- (void)tickReview:(double)elapsed {
    if (!_reviewing || !_reviewPlaying) {
        return;
    }
    _reviewClock += elapsed * _reviewSpeed;
    double clock = _reviewClock;
    if (clock >= _recording.endTime()) {
        _reviewPlaying = NO;
        [self showFrame:_recording.size() - 1];
        return;
    }
    [self showFrame:_recording.indexAtTime(clock)];
    _reviewClock = clock; // Keep the fractions of a frame that showFrame rounded away.
}

@end
