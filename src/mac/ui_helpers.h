#pragma once
// Shared Cocoa building blocks for the editor windows (labels, number fields, XYZ rows, cards).

#import <Cocoa/Cocoa.h>

#include "robotics/body.hpp"
#include "robotics/joint.hpp"

// NSView with a top-left origin, matching the layout coordinates used throughout the editor.
@interface FlippedView : NSView
@end

// Rounded translucent panel used for inspector sections.
@interface CardView : FlippedView
@end

namespace editor_ui {

NSTextField* label(NSView* parent, NSString* text, NSRect frame, CGFloat size = 12,
                   NSFontWeight weight = NSFontWeightRegular, NSColor* color = nil);
NSTextField* wrapping(NSView* parent, NSString* text, NSRect frame, CGFloat size = 11,
                      NSColor* color = NSColor.secondaryLabelColor);
NSTextField* heading(NSView* parent, NSString* text, CGFloat y);
NSTextField* numberField(NSView* parent, NSRect frame, id target = nil, SEL action = nil,
                         NSString* tip = nil);
NSButton* pushButton(NSView* parent, NSString* title, NSRect frame, id target, SEL action,
                     NSString* symbol = nil);

// Title plus three colored X/Y/Z number fields; returns the fields.
NSArray<NSTextField*>* xyzRow(NSView* card, NSString* title, CGFloat y, id target, SEL action, NSString* tip,
                              NSArray<NSTextField*>** letters = nullptr, NSTextField** titleField = nullptr);

void setVector(NSArray<NSTextField*>* fields, robotics::Vec3 value);

// Parses a number field. On failure remembers the field (see markInvalidNumberField) and throws.
double number(NSTextField* field);
robotics::Vec3 getVector(NSArray<NSTextField*>* fields);
void clearInvalidNumberField();
void markInvalidNumberField(); // Colors the last field rejected by number() red, if any.

void setEnabledRecursive(NSView* view, BOOL enabled);

NSString* shapeName(robotics::Shape shape);
NSString* shapeSymbol(robotics::Shape shape);
NSString* jointName(robotics::JointType type); // UI name: Fixed, Ball, Hinge, Slider.
NSColor* bodyTint(const robotics::Body& body, int index);

} // namespace editor_ui
