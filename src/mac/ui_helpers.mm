// Shared Cocoa building blocks for the editor windows.

#import "mac/ui_helpers.h"

#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace robotics;

@implementation FlippedView

- (BOOL)isFlipped {
    return YES;
}

@end

@implementation CardView

- (instancetype)initWithFrame:(NSRect)frame {
    self = [super initWithFrame:frame];
    if (!self) {
        return nil;
    }
    self.wantsLayer = YES;
    self.layer.cornerRadius = 10;
    self.layer.borderWidth = 1;
    self.layer.backgroundColor = [NSColor colorWithCalibratedWhite:1 alpha:0.045].CGColor;
    self.layer.borderColor = [NSColor colorWithCalibratedWhite:1 alpha:0.07].CGColor;
    return self;
}

@end

namespace editor_ui {

namespace {
NSTextField* invalidNumberField = nil; // Last field rejected by number().
} // namespace

NSTextField* label(NSView* parent, NSString* text, NSRect frame, CGFloat size, NSFontWeight weight,
                   NSColor* color) {
    NSTextField* field = [NSTextField labelWithString:text];
    field.frame = frame;
    field.font = [NSFont systemFontOfSize:size weight:weight];
    if (color) {
        field.textColor = color;
    }
    field.lineBreakMode = NSLineBreakByTruncatingTail;
    [parent addSubview:field];
    return field;
}

NSTextField* wrapping(NSView* parent, NSString* text, NSRect frame, CGFloat size, NSColor* color) {
    NSTextField* field = [NSTextField wrappingLabelWithString:text];
    field.frame = frame;
    field.font = [NSFont systemFontOfSize:size];
    field.textColor = color;
    field.selectable = NO;
    [parent addSubview:field];
    return field;
}

NSTextField* heading(NSView* parent, NSString* text, CGFloat y) {
    return label(parent, text, NSMakeRect(14, y, 272, 16), 10.5, NSFontWeightSemibold,
                 NSColor.secondaryLabelColor);
}

NSTextField* numberField(NSView* parent, NSRect frame, id target, SEL action, NSString* tip) {
    NSTextField* field = [[NSTextField alloc] initWithFrame:frame];
    field.stringValue = @"0";
    field.bezelStyle = NSTextFieldRoundedBezel;
    field.font = [NSFont monospacedDigitSystemFontOfSize:12 weight:NSFontWeightRegular];
    field.toolTip = tip;
    if (action) {
        field.target = target;
        field.action = action;
        field.cell.sendsActionOnEndEditing = YES;
    }
    [parent addSubview:field];
    return field;
}

NSButton* pushButton(NSView* parent, NSString* title, NSRect frame, id target, SEL action, NSString* symbol) {
    NSButton* b = [NSButton buttonWithTitle:title target:target action:action];
    b.frame = frame;
    b.bezelStyle = NSBezelStyleRounded;
    if (symbol) {
        b.image = [NSImage imageWithSystemSymbolName:symbol accessibilityDescription:title];
        b.imagePosition = NSImageLeading;
    }
    [parent addSubview:b];
    return b;
}

NSArray<NSTextField*>* xyzRow(NSView* card, NSString* title, CGFloat y, id target, SEL action, NSString* tip,
                              NSArray<NSTextField*>** letters, NSTextField** titleField) {
    NSTextField* t = label(card, title, NSMakeRect(14, y, 180, 16), 11.5, NSFontWeightMedium);
    if (titleField) {
        *titleField = t;
    }
    NSArray<NSColor*>* colors =
        @[ NSColor.systemRedColor, NSColor.systemGreenColor, NSColor.systemBlueColor ];
    NSMutableArray* fields = [NSMutableArray array];
    NSMutableArray* marks = [NSMutableArray array];
    for (int k = 0; k < 3; ++k) {
        CGFloat x = 14 + k * 92;
        [marks addObject:label(card, @[ @"X", @"Y", @"Z" ][k], NSMakeRect(x, y + 23, 13, 16), 10.5,
                               NSFontWeightBold, colors[k])];
        [fields addObject:numberField(card, NSMakeRect(x + 13, y + 19, 76, 24), target, action, tip)];
    }
    if (letters) {
        *letters = marks;
    }
    return fields;
}

void setVector(NSArray<NSTextField*>* fields, Vec3 v) {
    for (int i = 0; i < 3; ++i) {
        fields[i].stringValue = [NSString stringWithFormat:@"%.6g", std::abs(v[i]) < 1e-12 ? 0.0 : v[i]];
    }
}

double number(NSTextField* field) {
    std::string value = field.stringValue.UTF8String;
    std::size_t used = 0;
    double result = 0;
    try {
        result = std::stod(value, &used);
    } catch (...) {
        used = 0;
    }
    while (used < value.size() && std::isspace(static_cast<unsigned char>(value[used]))) {
        ++used;
    }
    if (used == 0 || used != value.size() || !std::isfinite(result)) {
        invalidNumberField = field;
        throw std::runtime_error("\"" + value + "\" is not a number. Please enter a value like 0.5");
    }
    return result;
}

Vec3 getVector(NSArray<NSTextField*>* fields) {
    return {number(fields[0]), number(fields[1]), number(fields[2])};
}

void setEnabledRecursive(NSView* view, BOOL enabled) {
    for (NSView* sub in view.subviews) {
        if ([sub isKindOfClass:NSControl.class]) {
            ((NSControl*)sub).enabled = enabled;
        }
        setEnabledRecursive(sub, enabled);
    }
}

NSString* shapeName(Shape s) {
    return s == Shape::Box ? @"Box" : (s == Shape::Sphere ? @"Ball" : @"Wheel");
}

NSString* shapeSymbol(Shape s) {
    return s == Shape::Box ? @"cube.fill" : (s == Shape::Sphere ? @"circle.fill" : @"circle.circle.fill");
}

NSColor* bodyTint(const Body& b, int index) {
    if (b.mass == 0) {
        return NSColor.secondaryLabelColor;
    }
    if (b.name == "rover_chassis") {
        return [NSColor colorWithCalibratedHue:0.51 saturation:0.78 brightness:0.9 alpha:1];
    }
    if (b.shape == Shape::Cylinder) {
        return [NSColor colorWithCalibratedWhite:0.75 alpha:1];
    }
    return [NSColor colorWithCalibratedHue:std::fmod(0.48 + index * 0.13, 1.0)
                                saturation:0.55
                                brightness:0.95
                                     alpha:1];
}

NSString* jointName(JointType type) {
    // The editor calls prismatic joints "sliders".
    return type == JointType::Prismatic ? @"Slider" : @(std::string(jointTypeName(type)).c_str());
}

void clearInvalidNumberField() {
    invalidNumberField = nil;
}

void markInvalidNumberField() {
    if (invalidNumberField) {
        invalidNumberField.textColor = NSColor.systemRedColor;
    }
}

} // namespace editor_ui