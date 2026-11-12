// EditorController (Inspector): the right-hand panel for editing the selected part and its joint.

#import "mac/editor_controller.h"

#include "robotics/editing/assembly.hpp"

#include <algorithm>
#include <string>

using namespace robotics;
using namespace editor_ui;

namespace {

// Default limit range for a joint type, in UI units (degrees for hinges, metres for sliders).
void setDefaultLimits(NSTextField* lower, NSTextField* upper, JointType type) {
    bool slider = type == JointType::Prismatic;
    lower.stringValue = slider ? @"0" : @"-90";
    upper.stringValue = slider ? @"0.5" : @"90";
}

} // namespace

@implementation EditorController (Inspector)

- (void)buildInspector {
    const CGFloat w = 300;
    _placeholderCard = [[CardView alloc] initWithFrame:NSMakeRect(12, 12, w, 120)];
    [_inspector addSubview:_placeholderCard];
    NSImageView* hand =
        [NSImageView imageViewWithImage:[NSImage imageWithSystemSymbolName:@"hand.point.up.left"
                                                  accessibilityDescription:nil]];
    hand.frame = NSMakeRect(14, 16, 28, 28);
    hand.contentTintColor = NSColor.secondaryLabelColor;
    hand.symbolConfiguration = [NSImageSymbolConfiguration configurationWithPointSize:20
                                                                               weight:NSFontWeightRegular];
    [_placeholderCard addSubview:hand];
    label(_placeholderCard, @"Nothing selected", NSMakeRect(50, 20, 230, 20), 13, NSFontWeightSemibold);
    wrapping(_placeholderCard,
             @"Click a part in the 3D view or in the Parts list to edit its size, position, mass and "
             @"connections. Use the Add buttons to create new parts.",
             NSMakeRect(14, 52, 272, 60), 11.5);

    // Part card
    _linkCard = [[CardView alloc] initWithFrame:NSMakeRect(12, 12, w, 372)];
    [_inspector addSubview:_linkCard];
    heading(_linkCard, @"SELECTED PART", 12);
    _linkIcon = [[NSImageView alloc] initWithFrame:NSMakeRect(12, 34, 24, 24)];
    _linkIcon.symbolConfiguration =
        [NSImageSymbolConfiguration configurationWithPointSize:17 weight:NSFontWeightSemibold];
    [_linkCard addSubview:_linkIcon];
    _name = [[NSTextField alloc] initWithFrame:NSMakeRect(42, 33, 244, 26)];
    _name.font = [NSFont systemFontOfSize:14 weight:NSFontWeightSemibold];
    _name.bezelStyle = NSTextFieldRoundedBezel;
    _name.placeholderString = @"Part name";
    _name.target = self;
    _name.action = @selector(applyBody:);
    _name.cell.sendsActionOnEndEditing = YES;
    _name.toolTip = @"Name of the part. Press Return to rename.";
    [_linkCard addSubview:_name];
    _linkKind = label(_linkCard, @"", NSMakeRect(42, 62, 244, 16), 11, NSFontWeightRegular,
                      NSColor.secondaryLabelColor);
    _position = xyzRow(
        _linkCard, @"Position (m)", 86, self, @selector(applyBody:),
        @"Center of the part in meters. Y points up. Moving a part also moves everything attached below it.");
    _rotation = xyzRow(_linkCard, @"Rotation (degrees)", 136, self, @selector(applyBody:),
                       @"Rotation around X, Y and Z in degrees.");
    NSArray<NSTextField*>* letters = nil;
    NSTextField* sizeTitle = nil;
    _size = xyzRow(_linkCard, @"Size (m)", 186, self, @selector(applyBody:), @"Dimensions in meters.",
                   &letters, &sizeTitle);
    _sizeLetters = letters;
    _sizeTitle = sizeTitle;
    label(_linkCard, @"Mass (kg)", NSMakeRect(14, 236, 120, 16), 11.5, NSFontWeightMedium);
    _mass = numberField(_linkCard, NSMakeRect(14, 255, 96, 24), self, @selector(applyBody:),
                        @"Mass in kilograms. Heavier parts are harder to move.");
    _static = [NSButton checkboxWithTitle:@"Fixed in place" target:self action:@selector(toggleStatic:)];
    _static.frame = NSMakeRect(124, 257, 160, 20);
    _static.toolTip = @"A fixed part never moves (mass 0). Use it for bases and stands.";
    [_linkCard addSubview:_static];
    _linkHint = wrapping(_linkCard, @"", NSMakeRect(14, 288, 272, 30), 11);
    pushButton(_linkCard, @"Duplicate", NSMakeRect(10, 326, 138, 30), self, @selector(duplicateBody:),
               @"plus.square.on.square");
    NSButton* del = pushButton(_linkCard, @"Delete", NSMakeRect(152, 326, 138, 30), self,
                               @selector(deleteBody:), @"trash");
    del.contentTintColor = NSColor.systemRedColor;

    // Joint card
    _jointCard = [[CardView alloc] initWithFrame:NSMakeRect(12, 0, w, 514)];
    [_inspector addSubview:_jointCard];
    heading(_jointCard, @"CONNECTION TO PARENT", 12);
    label(_jointCard, @"Attach to", NSMakeRect(14, 38, 80, 18), 11.5, NSFontWeightMedium);
    _parentList = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(86, 33, 202, 26) pullsDown:NO];
    _parentList.target = self;
    _parentList.action = @selector(chooseParent:);
    _parentList.toolTip = @"The part this one is connected to.";
    [_jointCard addSubview:_parentList];
    // Segment index == JointType value.
    _jointType = [NSSegmentedControl segmentedControlWithLabels:@[ @"Fixed", @"Ball", @"Hinge", @"Slider" ]
                                                   trackingMode:NSSegmentSwitchTrackingSelectOne
                                                         target:self
                                                         action:@selector(jointTypeChanged:)];
    _jointType.frame = NSMakeRect(12, 68, 276, 26);
    _jointType.segmentDistribution = NSSegmentDistributionFillEqually;
    [_jointCard addSubview:_jointType];
    [_jointType setToolTip:@"Welded together, no movement" forSegment:0];
    [_jointType setToolTip:@"Free rotation around the anchor point" forSegment:1];
    [_jointType setToolTip:@"Rotation around one axis, can have a motor" forSegment:2];
    [_jointType setToolTip:@"Sliding along one axis, can have a motor" forSegment:3];
    _jointTypeHelp = wrapping(_jointCard, @"", NSMakeRect(14, 100, 272, 30), 11);
    _anchor = xyzRow(_jointCard, @"Anchor point (m)", 136, nil, nil,
                     @"World position where the two parts are connected.");
    _midpoint = [NSButton buttonWithTitle:@"Midpoint" target:self action:@selector(useMidpoint:)];
    _midpoint.controlSize = NSControlSizeSmall;
    _midpoint.bezelStyle = NSBezelStyleRounded;
    _midpoint.font = [NSFont systemFontOfSize:11];
    _midpoint.frame = NSMakeRect(196, 131, 92, 22);
    _midpoint.toolTip = @"Place the anchor halfway between the two parts";
    [_jointCard addSubview:_midpoint];
    NSTextField* axisTitle = nil;
    _axis = xyzRow(_jointCard, @"Rotation axis", 186, nil, nil,
                   @"Direction the hinge rotates around or the slider moves along (world space). Normalized "
                   @"automatically.",
                   nullptr, &axisTitle);
    _axisTitle = axisTitle;
    _axisPreset = [NSSegmentedControl segmentedControlWithLabels:@[ @"X", @"Y", @"Z" ]
                                                    trackingMode:NSSegmentSwitchTrackingMomentary
                                                          target:self
                                                          action:@selector(axisPreset:)];
    _axisPreset.controlSize = NSControlSizeSmall;
    _axisPreset.frame = NSMakeRect(196, 182, 92, 22);
    _axisPreset.toolTip = @"Quick-set the axis to X, Y or Z";
    [_jointCard addSubview:_axisPreset];
    _speedLabel =
        label(_jointCard, @"Motor speed (rad/s)", NSMakeRect(14, 236, 136, 16), 11.5, NSFontWeightMedium);
    _torqueLabel =
        label(_jointCard, @"Max torque (N·m)", NSMakeRect(152, 236, 136, 16), 11.5, NSFontWeightMedium);
    _speed = numberField(_jointCard, NSMakeRect(14, 255, 128, 24), nil, nil, @"");
    _torque = numberField(_jointCard, NSMakeRect(152, 255, 134, 24), nil, nil, @"");
    _limitsEnabled = [NSButton checkboxWithTitle:@"Limit range" target:self action:@selector(toggleLimits:)];
    _limitsEnabled.frame = NSMakeRect(12, 290, 272, 20);
    _limitsEnabled.toolTip = @"Keep the joint between Min and Max, measured from the pose where it was "
                             @"connected. Limits also stop the motor.";
    [_jointCard addSubview:_limitsEnabled];
    _lowerLabel = label(_jointCard, @"Min (°)", NSMakeRect(14, 315, 136, 16), 11.5, NSFontWeightMedium);
    _upperLabel = label(_jointCard, @"Max (°)", NSMakeRect(152, 315, 136, 16), 11.5, NSFontWeightMedium);
    _lowerLimit = numberField(_jointCard, NSMakeRect(14, 334, 128, 24), nil, nil,
                              @"Lowest allowed angle or position, relative to the connected pose.");
    _upperLimit = numberField(_jointCard, NSMakeRect(152, 334, 134, 24), nil, nil,
                              @"Highest allowed angle or position, relative to the connected pose.");
    _jointHint = wrapping(_jointCard, @"", NSMakeRect(14, 366, 272, 44), 11);
    _jointApply = pushButton(_jointCard, @"Create connection", NSMakeRect(10, 418, 280, 32), self,
                             @selector(applyJoint:), @"link");
    _jointApply.bezelColor = NSColor.controlAccentColor;
    _jointRemove = pushButton(_jointCard, @"Disconnect", NSMakeRect(10, 456, 280, 30), self,
                              @selector(removeJoint:), @"link.badge.plus");
    _jointRemove.image = [NSImage imageWithSystemSymbolName:@"scissors" accessibilityDescription:nil];

    // Simulation card
    _simCard = [[CardView alloc] initWithFrame:NSMakeRect(12, 0, w, 150)];
    [_inspector addSubview:_simCard];
    heading(_simCard, @"DRIVE & CONTROL", 12);
    wrapping(_simCard,
             @"Click the 3D view, then use W / S to drive, A / D to turn and X to brake. Space starts or "
             @"pauses the simulation.",
             NSMakeRect(14, 34, 272, 48), 11.5, NSColor.labelColor);
    pushButton(_simCard, @"Open Control Lab (PID & plots)…", NSMakeRect(10, 96, 280, 30), self,
               @selector(openControlLab:), @"waveform.path.ecg");
    [self buildSensorCard];
    // Right-click shortcuts to the Plots window.
    _inspectorPlotMenu = [[NSMenu alloc] initWithTitle:@"Plot"];
    _inspectorPlotMenu.delegate = self;
    _linkCard.menu = _inspectorPlotMenu;
    _jointCard.menu = _inspectorPlotMenu;
    for (NSArray<NSTextField*>* row in @[ _position, _rotation, _anchor, _axis ]) {
        for (NSTextField* field in row) {
            field.menu = _inspectorPlotMenu;
        }
    }
    _speed.menu = _inspectorPlotMenu;
    _torque.menu = _inspectorPlotMenu;
}

- (void)layoutInspector {
    CGFloat y = 12;
    auto place = [&](NSView* card, BOOL visible) {
        card.hidden = !visible;
        if (visible) {
            card.frame = NSMakeRect(12, y, card.frame.size.width, card.frame.size.height);
            y += card.frame.size.height + 12;
        }
    };
    BOOL has = _selected >= 0;
    place(_placeholderCard, !has);
    place(_linkCard, has);
    place(_jointCard, has);
    place(_sensorCard, has);
    place(_simCard, YES);
    _inspector.frame = NSMakeRect(0, 0, _inspectorScroll.contentSize.width,
                                  std::max<CGFloat>(y, _inspectorScroll.contentSize.height));
}

- (int)parentIndex {
    NSMenuItem* item = _parentList.selectedItem;
    return item ? static_cast<int>(item.tag) : -1;
}

- (void)refreshInspector {
    _inspectorBody = _selected;
    if (_selected < 0) {
        return;
    }
    const Body& b = [self sceneWorld]->bodies[_selected]; // While reviewing: the frame on screen.
    for (NSTextField* f in [[_position arrayByAddingObjectsFromArray:_rotation]
             arrayByAddingObjectsFromArray:[_size arrayByAddingObject:_mass]]) {
        f.textColor = NSColor.labelColor;
    }
    _linkIcon.image = [NSImage imageWithSystemSymbolName:shapeSymbol(b.shape) accessibilityDescription:nil];
    _linkIcon.contentTintColor = bodyTint(b, _selected);
    _name.stringValue = @(b.name.c_str());
    int connections = 0;
    for (const auto& j : _world.joints) {
        if (j.a == _selected || j.b == _selected) {
            ++connections;
        }
    }
    _linkKind.stringValue = [NSString stringWithFormat:@"%@ · %@ · %d connection%@", shapeName(b.shape),
                                                       b.mass == 0 ? @"fixed in place" : @"moves freely",
                                                       connections, connections == 1 ? @"" : @"s"];
    setVector(_position, b.position);
    setVector(_rotation, b.orientation.eulerAngles() * (180 / robotics::pi));
    setVector(_size, b.size);
    _mass.stringValue = [NSString stringWithFormat:@"%.6g", b.mass];
    _static.state = b.mass == 0 ? NSControlStateValueOn : NSControlStateValueOff;
    BOOL box = b.shape == Shape::Box;
    BOOL ball = b.shape == Shape::Sphere;
    _sizeTitle.stringValue = box ? @"Size (m)" : (ball ? @"Diameter (m)" : @"Diameter and width (m)");
    _size[1].hidden = _sizeLetters[1].hidden = !box;
    _size[2].hidden = _sizeLetters[2].hidden = ball;
    _sizeLetters[0].stringValue = box ? @"X" : @"⌀";
    _sizeLetters[2].stringValue = box ? @"Z" : @"W";
    _sizeLetters[0].textColor = box ? NSColor.systemRedColor : NSColor.secondaryLabelColor;
    _sizeLetters[2].textColor = box ? NSColor.systemBlueColor : NSColor.secondaryLabelColor;
    _shownLinkValues = [self linkFieldSignature];

    [_parentList removeAllItems];
    [_parentList addItemWithTitle:@"Nothing (free part)"];
    _parentList.lastItem.tag = -1;
    [_parentList.menu addItem:NSMenuItem.separatorItem];
    _parentList.lastItem.tag = -2; // Tag 0 is body 0: selectItemWithTag must not find the separator.
    for (int i = 0; i < static_cast<int>(_world.bodies.size()); ++i) {
        if (i != _selected) {
            [_parentList addItemWithTitle:[NSString stringWithFormat:@"%s", _world.bodies[i].name.c_str()]];
            _parentList.lastItem.tag = i;
            _parentList.lastItem.image =
                [NSImage imageWithSystemSymbolName:shapeSymbol(_world.bodies[i].shape)
                          accessibilityDescription:nil];
        }
    }
    [_parentList selectItemWithTag:-1];
    for (const auto& j : _world.joints) {
        if (j.b == _selected) {
            [_parentList selectItemWithTag:j.a];
            break;
        }
    }
    [self chooseParent:nil];
    [self updateEnabledState];
    [self refreshSensorCard];
}

- (NSString*)linkFieldSignature {
    NSMutableString* s = [NSMutableString stringWithString:_name.stringValue];
    for (NSTextField* f in [[_position arrayByAddingObjectsFromArray:_rotation]
             arrayByAddingObjectsFromArray:[_size arrayByAddingObject:_mass]]) {
        [s appendFormat:@"\x1f%@", f.stringValue];
    }
    return s;
}

- (void)updateEnabledState {
    BOOL edit = !_running && !_reviewing;
    setEnabledRecursive(_linkCard, edit);
    setEnabledRecursive(_jointCard, edit);
    if (edit) {
        _mass.enabled = _static.state != NSControlStateValueOn;
        BOOL parent = [self parentIndex] >= 0;
        BOOL actuated = isActuatedJointType(static_cast<JointType>(_jointType.selectedSegment));
        BOOL exists = [self existingJoint] >= 0;
        for (NSControl* c in [(NSArray<NSControl*>*)_anchor
                 arrayByAddingObjectsFromArray:@[ _jointType, _jointApply, _midpoint ]]) {
            c.enabled = parent;
        }
        for (NSControl* c in [(NSArray<NSControl*>*)_axis
                 arrayByAddingObjectsFromArray:@[ _axisPreset, _speed, _torque, _limitsEnabled ]]) {
            c.enabled = parent && actuated;
        }
        BOOL limits = parent && actuated && _limitsEnabled.state == NSControlStateValueOn;
        _lowerLimit.enabled = limits;
        _upperLimit.enabled = limits;
        _jointRemove.enabled = exists;
    }
    _linkHint.stringValue =
        _reviewing ? @"Viewing a recording. Press Live (Esc) to edit this part."
                   : (_running ? @"Pause the simulation (Space) to edit this part."
                               : @"Changes apply on Return. Move and rotate in 3D with keys 2 and 3.");
    _linkHint.textColor = edit ? NSColor.secondaryLabelColor : NSColor.systemOrangeColor;
}

- (int)existingJoint {
    int parent = [self parentIndex];
    if (parent < 0 || _selected < 0) {
        return -1;
    }
    for (int k = 0; k < static_cast<int>(_world.joints.size()); ++k) {
        const auto& j = _world.joints[k];
        if ((j.a == parent && j.b == _selected) || (j.b == parent && j.a == _selected)) {
            return k;
        }
    }
    return -1;
}

- (void)toggleStatic:(id)sender {
    (void)sender;
    if (_static.state == NSControlStateValueOn) {
        _mass.stringValue = @"0";
    } else if (number(_mass) == 0) {
        _mass.stringValue = @"1";
    }
    [self applyBody:nil];
}

- (void)applyBody:(id)sender {
    (void)sender;
    if (_running || _reviewing) {
        if (isEditorSmokeTest()) {
            [self canEdit]; // Throws in the smoke test so a silent no-op cannot hide a failure.
        }
        return;
    }
    if (_selected < 0 || _inspectorBody != _selected || _selected >= static_cast<int>(_world.bodies.size())) {
        return;
    }
    NSString* signature = [self linkFieldSignature];
    if ([signature isEqualToString:_shownLinkValues]) {
        return;
    }
    clearInvalidNumberField();
    try {
        World candidate = _world;
        Body& b = candidate.bodies[_selected];
        const Body& old = _world.bodies[_selected];
        std::string name = _name.stringValue.UTF8String;
        if (name.empty()) {
            throw std::runtime_error("A part needs a name");
        }
        Vec3 position = getVector(_position);
        Vec3 size = getVector(_size);
        Vec3 rotation = getVector(_rotation);
        double mass = number(_mass);
        if (mass < 0) {
            throw std::runtime_error("Mass cannot be negative (use 0 for a fixed part)");
        }
        NSArray<NSString*>* shown = [_shownLinkValues componentsSeparatedByString:@"\x1f"];
        bool rotationChanged = false;
        for (int k = 0; k < 3; ++k) {
            if (![shown[4 + k] isEqualToString:_rotation[k].stringValue]) {
                rotationChanged = true;
            }
        }
        b.name = name;
        b.size = size;
        b.mass = mass;
        if (rotationChanged) {
            b.orientation = Quat::euler(rotation * (robotics::pi / 180));
        }
        if (b.shape == Shape::Sphere) {
            b.size = {b.size.x, b.size.x, b.size.x};
        }
        if (b.shape == Shape::Cylinder) {
            b.size.y = b.size.x;
        }
        b.velocity = {};
        b.angularVelocity = {};
        if (rotationChanged) { // Rotating a part keeps its joints at their current world positions.
            for (auto& j : candidate.joints) {
                if (j.a == _selected || j.b == _selected) {
                    Vec3 anchor = _world.bodies[j.a].point(j.anchorA);
                    Vec3 axis = _world.bodies[j.a].orientation.rotate(j.axisA);
                    j.anchorA = candidate.bodies[j.a].localPoint(anchor);
                    j.anchorB = candidate.bodies[j.b].localPoint(anchor);
                    j.axisA = candidate.bodies[j.a].orientation.conjugate().rotate(axis);
                    j.axisB = candidate.bodies[j.b].orientation.conjugate().rotate(axis);
                    j.reference =
                        candidate.bodies[j.a].orientation.conjugate() * candidate.bodies[j.b].orientation;
                    j.angle = 0;
                    j.lastWrappedAngle = 0;
                    j.encoderInitialized = false;
                }
            }
        }
        Vec3 delta = position - old.position;
        bool positionChanged = false;
        for (int k = 0; k < 3; ++k) {
            if (![shown[1 + k] isEqualToString:_position[k].stringValue]) {
                positionChanged = true;
            }
        }
        if (positionChanged && delta.length() > 0) {
            moveAssembly(candidate, _selected, delta);
        }
        candidate.validate();
        [self recordUndo:@"Edit Part"];
        _world = std::move(candidate);
        [self invalidateSnapshot];
        [self refresh];
    } catch (const std::exception& e) {
        markInvalidNumberField();
        [self showError:[NSString stringWithUTF8String:e.what()]];
    }
}

- (void)chooseParent:(id)sender {
    int parent = [self parentIndex];
    setVector(_axis, {0, 0, 1});
    _speed.stringValue = @"0";
    _torque.stringValue = @"0";
    _limitsEnabled.state = NSControlStateValueOff;
    _lowerLimit.stringValue = @"";
    _upperLimit.stringValue = @"";
    [_jointType setSelected:YES forSegment:static_cast<int>(JointType::Hinge)];
    int existing = [self existingJoint];
    if (_selected >= 0 && parent >= 0) {
        setVector(_anchor, (_world.bodies[parent].position + _world.bodies[_selected].position) * 0.5);
    }
    if (existing >= 0) {
        const Joint& j = _world.joints[existing];
        [_jointType setSelected:YES forSegment:static_cast<int>(j.type)];
        setVector(_anchor, _world.bodies[j.a].point(j.anchorA));
        setVector(_axis, _world.bodies[j.a].orientation.rotate(j.axisA));
        _speed.stringValue =
            [NSString stringWithFormat:@"%.6g", j.a == parent ? j.motorSpeed : -j.motorSpeed];
        _torque.stringValue = [NSString stringWithFormat:@"%.6g", j.maxTorque];
        if (j.isActuated() && j.limitsEnabled) {
            // Shown in the parent -> part direction; hinge limits in degrees.
            double scale = j.type == JointType::Hinge ? 180 / robotics::pi : 1;
            double lower = j.a == parent ? j.lowerLimit : -j.upperLimit;
            double upper = j.a == parent ? j.upperLimit : -j.lowerLimit;
            _limitsEnabled.state = NSControlStateValueOn;
            _lowerLimit.stringValue = [NSString stringWithFormat:@"%.6g", lower * scale];
            _upperLimit.stringValue = [NSString stringWithFormat:@"%.6g", upper * scale];
        }
    } else if (sender && parent >= 0 && _world.bodies[_selected].shape == Shape::Cylinder) {
        // Sensible wheel defaults: motorized hinge on the wheel's own axle.
        setVector(_anchor, _world.bodies[_selected].position);
        setVector(_axis, _world.bodies[_selected].orientation.rotate({0, 0, 1}));
        _torque.stringValue = @"8";
    }
    [self jointTypeChanged:nil];
}

- (void)toggleLimits:(id)sender {
    (void)sender;
    if (_limitsEnabled.state == NSControlStateValueOn
        && (_lowerLimit.stringValue.length == 0 || _upperLimit.stringValue.length == 0)) {
        setDefaultLimits(_lowerLimit, _upperLimit, static_cast<JointType>(_jointType.selectedSegment));
    }
    [self updateEnabledState];
}

- (void)jointTypeChanged:(id)sender {
    JointType type = static_cast<JointType>(_jointType.selectedSegment);
    if (sender && type != _shownJointType) {
        // Degrees and metres do not convert: start from the new type's defaults, or leave the fields
        // empty so ticking "Limit range" later fills in the right defaults.
        if (isActuatedJointType(type) && _limitsEnabled.state == NSControlStateValueOn) {
            setDefaultLimits(_lowerLimit, _upperLimit, type);
        } else {
            _lowerLimit.stringValue = @"";
            _upperLimit.stringValue = @"";
        }
    }
    _shownJointType = type;
    bool slider = type == JointType::Prismatic;
    switch (type) {
    case JointType::Fixed:
        _jointTypeHelp.stringValue = @"Fixed: the parts are welded together and move as one.";
        break;
    case JointType::Ball:
        _jointTypeHelp.stringValue = @"Ball: the part can swing freely in every direction around the anchor.";
        break;
    case JointType::Hinge:
        _jointTypeHelp.stringValue =
            @"Hinge: the part rotates around one axis, like a wheel or an elbow. It can have a motor.";
        break;
    case JointType::Prismatic:
        _jointTypeHelp.stringValue = @"Slider: the part slides along one axis, like a drawer or a linear "
                                     @"actuator. It can have a motor.";
        break;
    }
    _axisTitle.stringValue = slider ? @"Slide axis" : @"Rotation axis";
    _speedLabel.stringValue = slider ? @"Motor speed (m/s)" : @"Motor speed (rad/s)";
    _torqueLabel.stringValue = slider ? @"Max force (N)" : @"Max torque (N·m)";
    _speed.toolTip =
        slider ? @"Target sliding speed. Positive = along the slide axis."
               : @"Target spin speed. Positive = counter-clockwise around the axis (right-hand rule).";
    _torque.toolTip = slider ? @"Motor strength. 0 turns the motor off (the part slides freely)."
                             : @"Motor strength. 0 turns the motor off (the hinge spins freely).";
    _lowerLabel.stringValue = slider ? @"Min (m)" : @"Min (°)";
    _upperLabel.stringValue = slider ? @"Max (m)" : @"Max (°)";
    int parent = [self parentIndex];
    int existing = [self existingJoint];
    int attached = 0;
    for (const auto& j : _world.joints) {
        if (j.a == _selected) {
            ++attached;
        }
    }
    if (parent < 0) {
        _jointHint.stringValue =
            attached > 0 ? [NSString stringWithFormat:@"This is a base part with %d part%@ attached to it. "
                                                      @"To edit those connections, select the attached part.",
                                                      attached, attached == 1 ? @"" : @"s"]
                         : @"This part is free. Choose a part under \"Attach to\" to connect it.";
    } else if (existing >= 0) {
        const Joint& j = _world.joints[existing];
        _jointHint.stringValue =
            j.isActuated() && j.controller.type != ControllerType::SpeedMotor
                ? @"Connected. This joint is driven by a feedback controller — tune it in the Control Lab."
                : @"Connected. Edit the values above and click Update connection to apply them.";
    } else {
        _jointHint.stringValue = @"Not connected yet. Check the anchor point, then click Create connection.";
    }
    _jointApply.title = existing >= 0 ? @"Update connection" : @"Create connection";
    [self updateEnabledState];
}

- (void)axisPreset:(NSSegmentedControl*)sender {
    Vec3 axis{};
    axis[static_cast<int>(sender.selectedSegment)] = 1;
    setVector(_axis, axis);
}

- (void)useMidpoint:(id)sender {
    (void)sender;
    int parent = [self parentIndex];
    if (parent < 0 || _selected < 0) {
        return;
    }
    setVector(_anchor, (_world.bodies[parent].position + _world.bodies[_selected].position) * 0.5);
}

- (void)applyJoint:(id)sender {
    (void)sender;
    if (![self canEdit] || _selected < 0) {
        return;
    }
    clearInvalidNumberField();
    try {
        int parent = [self parentIndex];
        if (parent < 0) {
            throw std::runtime_error("Choose a part under \"Attach to\" first");
        }
        World candidate = _world;
        int existing = [self existingJoint];
        bool existed = existing >= 0;
        Joint previous;
        if (existed) {
            previous = candidate.joints[existing];
            if (previous.a != parent) { // Express the old joint in the parent -> part direction.
                previous.controller.target = -previous.controller.target;
                previous.controller.amplitude = -previous.controller.amplitude;
                previous.angle = -previous.angle;
            }
        }
        JointType type = static_cast<JointType>(_jointType.selectedSegment);
        Vec3 anchor = getVector(_anchor);
        Vec3 axis = getVector(_axis);
        double speed = number(_speed);
        double torque = number(_torque);
        if (torque < 0) {
            throw std::runtime_error(type == JointType::Prismatic ? "Max force must be zero or positive"
                                                                  : "Max torque must be zero or positive");
        }
        bool limits = isActuatedJointType(type) && _limitsEnabled.state == NSControlStateValueOn;
        double lower = limits ? number(_lowerLimit) : 0;
        double upper = limits ? number(_upperLimit) : 0;
        if (limits && lower > upper) {
            clearInvalidNumberField();
            throw std::runtime_error("Min must not be greater than Max");
        }
        if (limits && type == JointType::Hinge) {
            lower *= robotics::pi / 180;
            upper *= robotics::pi / 180;
        }
        // The UI shows limits and targets relative to the existing joint's zero pose.
        bool sameType = existed && previous.type == type;
        bool sameAxis = false;
        bool sameGeometry = false;
        if (existed) {
            const Body& oldParent = _world.bodies[previous.a];
            Vec3 oldAnchor = oldParent.point(previous.anchorA);
            Vec3 oldAxis = oldParent.orientation.rotate(previous.axisA);
            if (previous.a != parent) {
                oldAxis = -oldAxis;
            }
            double anchorTolerance = 1e-4 * std::max(1.0, oldAnchor.length());
            sameAxis = axis.length() > 0 && (axis.normalized() - oldAxis).length() < 1e-4;
            sameGeometry = sameType && previous.a == parent && sameAxis
                           && (anchor - oldAnchor).length() < anchorTolerance;
        }
        Joint* joint = nullptr;
        double zeroShift = 0; // Old joint coordinate of the current pose, subtracted after re-zeroing.
        bool zeroReset = false;
        if (sameGeometry) {
            // Same connection: update the actuator and limits in place, keeping the encoder zero,
            // controller and history, so limits stay where they physically were.
            joint = &candidate.joints[existing];
        } else {
            std::erase_if(candidate.joints, [&](const Joint& j) {
                return (j.a == parent && j.b == _selected) || (j.b == parent && j.a == _selected);
            });
            int index = candidate.connect(parent, _selected, type, anchor, axis);
            joint = &candidate.joints[index];
            if (sameType && isActuatedJointType(type)) {
                joint->controller = previous.controller; // Same units, so the tuning still applies.
                if (sameAxis) {
                    zeroShift = previous.angle; // connect() re-zeroes at the current pose.
                } else {
                    zeroReset = std::abs(previous.angle) > 1e-9;
                }
            }
        }
        joint->motorSpeed = speed;
        joint->maxTorque = torque;
        if (limits) {
            joint->setLimits(lower - zeroShift, upper - zeroShift);
        } else {
            joint->limitsEnabled = false;
            joint->lowerLimit = 0;
            joint->upperLimit = 0;
        }
        if (!sameGeometry && joint->controller.type != ControllerType::SpeedMotor
            && joint->controller.variable == ControlVariable::Position) {
            joint->controller.target -= zeroShift;
        }
        bool outside = limits && (joint->angle < joint->lowerLimit || joint->angle > joint->upperLimit);
        candidate.validate();
        [self recordUndo:existed ? @"Update Connection" : @"Connect Parts"];
        _world = std::move(candidate);
        [self invalidateSnapshot];
        [self refresh];
        NSString* message = [NSString
            stringWithFormat:@"%@ joint %@: %s → %s", jointName(type), existed ? @"updated" : @"created",
                             _world.bodies[parent].name.c_str(), _world.bodies[_selected].name.c_str()];
        if (zeroReset) {
            message =
                [message stringByAppendingString:@". The axis changed, so the joint's zero was reset to the "
                                                 @"current pose"];
        }
        if (outside) {
            message =
                [message stringByAppendingString:@". The joint is currently outside the range and will be "
                                                 @"pushed back in"];
        }
        [self toast:message];
    } catch (const std::exception& e) {
        markInvalidNumberField();
        [self showError:[NSString stringWithUTF8String:e.what()]];
    }
}

- (void)removeJoint:(id)sender {
    (void)sender;
    if (![self canEdit] || [self existingJoint] < 0) {
        return;
    }
    int parent = [self parentIndex];
    [self recordUndo:@"Disconnect"];
    std::erase_if(_world.joints, [&](const Joint& j) {
        return (j.a == parent && j.b == _selected) || (j.b == parent && j.a == _selected);
    });
    [self invalidateSnapshot];
    [self refresh];
    [self toast:@"Disconnected. The part is now free."];
}

@end
