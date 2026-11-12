// Timeline bar: see timeline_bar.h.

#import "mac/timeline_bar.h"
#import "mac/editor_controller.h"

#include <algorithm>

@implementation TimelineTrack

- (BOOL)acceptsFirstMouse:(NSEvent*)event {
    (void)event;
    return YES;
}

- (BOOL)mouseDownCanMoveWindow {
    return NO;
}

- (void)setPosition:(double)position {
    _position = position;
    self.needsDisplay = YES;
}

- (double)fractionAt:(NSEvent*)event {
    NSPoint p = [self convertPoint:event.locationInWindow fromView:nil];
    return std::clamp((p.x - 8) / std::max<CGFloat>(self.bounds.size.width - 16, 1), 0.0, 1.0);
}

- (void)mouseDown:(NSEvent*)event {
    [self.editor timelineScrubTo:[self fractionAt:event]];
}

- (void)mouseDragged:(NSEvent*)event {
    [self.editor timelineScrubTo:[self fractionAt:event]];
}

- (void)drawRect:(NSRect)dirty {
    (void)dirty;
    NSRect b = self.bounds;
    CGFloat y = b.size.height / 2;
    CGFloat width = b.size.width - 16;
    NSRect rail = NSMakeRect(8, y - 2, width, 4);
    [[NSColor colorWithCalibratedWhite:1 alpha:0.16] setFill];
    [[NSBezierPath bezierPathWithRoundedRect:rail xRadius:2 yRadius:2] fill];
    if (!self.hasRecording) {
        return;
    }
    CGFloat x = 8 + width * self.position;
    NSColor* accent = self.live ? NSColor.systemRedColor : NSColor.controlAccentColor;
    [accent setFill];
    [[NSBezierPath bezierPathWithRoundedRect:NSMakeRect(8, y - 2, x - 8, 4) xRadius:2 yRadius:2] fill];
    [NSColor.whiteColor setFill];
    [[NSBezierPath bezierPathWithOvalInRect:NSMakeRect(x - 7, y - 7, 14, 14)] fill];
    [accent setFill];
    [[NSBezierPath bezierPathWithOvalInRect:NSMakeRect(x - 3, y - 3, 6, 6)] fill];
}

@end

@implementation TimelineBar

- (void)mouseDown:(NSEvent*)event {
    (void)event; // A click between the controls must not reach the 3D view below.
}

- (NSButton*)button:(NSString*)symbol action:(SEL)action x:(CGFloat)x target:(id)target tip:(NSString*)tip {
    NSImage* image = [NSImage imageWithSystemSymbolName:symbol accessibilityDescription:tip];
    NSButton* button = [NSButton buttonWithImage:image target:target action:action];
    button.bordered = NO;
    button.frame = NSMakeRect(x, 8, 26, 24);
    button.toolTip = tip;
    button.autoresizingMask = NSViewMaxXMargin;
    [self addSubview:button];
    return button;
}

- (NSButton*)textButton:(NSString*)title
                 action:(SEL)action
                  right:(CGFloat)right
                  width:(CGFloat)width
                 target:(id)target
                    tip:(NSString*)tip {
    NSButton* button = [NSButton buttonWithTitle:title target:target action:action];
    button.controlSize = NSControlSizeSmall;
    button.font = [NSFont systemFontOfSize:11];
    button.frame = NSMakeRect(self.bounds.size.width - right - width, 9, width, 22);
    button.toolTip = tip;
    button.autoresizingMask = NSViewMinXMargin;
    [self addSubview:button];
    return button;
}

- (instancetype)initWithFrame:(NSRect)frame editor:(EditorController*)editor {
    self = [super initWithFrame:frame];
    if (!self) {
        return nil;
    }
    self.material = NSVisualEffectMaterialHUDWindow;
    self.blendingMode = NSVisualEffectBlendingModeWithinWindow;
    self.state = NSVisualEffectStateActive;
    self.wantsLayer = YES;
    self.layer.cornerRadius = 9;
    self.layer.masksToBounds = YES;
    _stepBack = [self button:@"backward.frame.fill"
                      action:@selector(timelineStepBack:)
                           x:8
                      target:editor
                         tip:@"One frame back (← in the 3D view)"];
    _play = [self button:@"play.fill"
                  action:@selector(timelinePlay:)
                       x:34
                  target:editor
                     tip:@"Play the recording at the chosen speed. The simulation itself stays paused."];
    _stepForward = [self button:@"forward.frame.fill"
                         action:@selector(timelineStepForward:)
                              x:60
                         target:editor
                            tip:@"One frame forward (→ in the 3D view)"];

    CGFloat right = 8;
    _more = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(frame.size.width - right - 36, 9, 36, 22)
                                       pullsDown:YES];
    _more.controlSize = NSControlSizeSmall;
    _more.bordered = NO;
    _more.autoresizingMask = NSViewMinXMargin;
    _more.toolTip = @"Recording options: length, CSV export, clear";
    NSMenuItem* icon = [[NSMenuItem alloc] initWithTitle:@"" action:nil keyEquivalent:@""];
    icon.image = [NSImage imageWithSystemSymbolName:@"ellipsis.circle" accessibilityDescription:@"Options"];
    [_more.menu addItem:icon];
    [_more.menu addItemWithTitle:@"Export Recording as CSV…"
                          action:@selector(exportRecording:)
                   keyEquivalent:@""];
    [_more.menu addItemWithTitle:@"Clear Recording" action:@selector(clearRecording:) keyEquivalent:@""];
    [_more.menu addItem:NSMenuItem.separatorItem];
    for (NSNumber* seconds in @[ @30, @60, @120, @300 ]) {
        NSMenuItem* item = [_more.menu addItemWithTitle:[NSString stringWithFormat:@"Keep Last %@ s", seconds]
                                                 action:@selector(setRecordingLength:)
                                          keyEquivalent:@""];
        item.tag = seconds.integerValue;
    }
    for (NSMenuItem* item in _more.menu.itemArray) {
        item.target = editor;
    }
    [self addSubview:_more];
    right += 40;

    _live = [self textButton:@"Live"
                      action:@selector(timelineLive:)
                       right:right
                       width:50
                      target:editor
                         tip:@"Leave the recording and return to the simulation's current state (Esc)"];
    right += 54;
    _resume = [self textButton:@"Resume Here"
                        action:@selector(timelineResume:)
                         right:right
                         width:96
                        target:editor
                           tip:@"Restart the simulation from the moment shown. Later frames are discarded."];
    right += 100;
    _speed = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(frame.size.width - right - 66, 9, 66, 22)
                                        pullsDown:NO];
    _speed.controlSize = NSControlSizeSmall;
    _speed.font = [NSFont systemFontOfSize:11];
    [_speed addItemsWithTitles:@[ @"0.25×", @"0.5×", @"1×", @"2×", @"4×" ]];
    [_speed selectItemAtIndex:2];
    _speed.target = editor;
    _speed.action = @selector(timelineSpeed:);
    _speed.autoresizingMask = NSViewMinXMargin;
    _speed.toolTip = @"Playback speed of the recording";
    [self addSubview:_speed];
    right += 70;
    _time = [NSTextField labelWithString:@""];
    _time.frame = NSMakeRect(frame.size.width - right - 118, 11, 118, 16);
    _time.alignment = NSTextAlignmentRight;
    _time.font = [NSFont monospacedDigitSystemFontOfSize:11 weight:NSFontWeightRegular];
    _time.textColor = NSColor.secondaryLabelColor;
    _time.lineBreakMode = NSLineBreakByTruncatingTail;
    _time.autoresizingMask = NSViewMinXMargin;
    [self addSubview:_time];
    right += 124;
    _track = [[TimelineTrack alloc] initWithFrame:NSMakeRect(90, 6, frame.size.width - 90 - right, 28)];
    _track.editor = editor;
    _track.autoresizingMask = NSViewWidthSizable;
    _track.toolTip =
        @"Drag to look through the recording. The simulation pauses and its state is not changed.";
    [self addSubview:_track];
    return self;
}

@end
