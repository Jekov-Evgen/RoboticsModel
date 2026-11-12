// EditorController (PartsList): the parts outline shows the assembly as a tree (parent -> child joints).

#import "mac/editor_controller.h"

using namespace robotics;
using namespace editor_ui;

@implementation EditorController (PartsList)

- (void)rebuildTree {
    int n = static_cast<int>(_world.bodies.size());
    std::vector<int> parent(n, -1);
    for (const auto& j : _world.joints) {
        if (parent[j.b] < 0 && j.a != j.b) {
            parent[j.b] = j.a;
        }
    }
    // Break cycles so every part is reachable from a root.
    for (int i = 0; i < n; ++i) {
        int p = parent[i];
        int steps = 0;
        while (p >= 0 && p != i && steps <= n) {
            p = parent[p];
            ++steps;
        }
        if (p == i) {
            parent[i] = -1;
        }
    }
    _roots.clear();
    _children.assign(n, {});
    for (int i = 0; i < n; ++i) {
        (parent[i] < 0 ? _roots : _children[parent[i]]).push_back(i);
    }
}

- (NSInteger)outlineView:(NSOutlineView*)outlineView numberOfChildrenOfItem:(id)item {
    (void)outlineView;
    return static_cast<NSInteger>((item ? _children[[item intValue]] : _roots).size());
}

- (id)outlineView:(NSOutlineView*)outlineView child:(NSInteger)index ofItem:(id)item {
    (void)outlineView;
    return @((item ? _children[[item intValue]] : _roots)[index]);
}

- (BOOL)outlineView:(NSOutlineView*)outlineView isItemExpandable:(id)item {
    (void)outlineView;
    return !_children[[item intValue]].empty();
}

- (NSView*)outlineView:(NSOutlineView*)outlineView viewForTableColumn:(NSTableColumn*)column item:(id)item {
    (void)column;
    NSTableCellView* cell = [outlineView makeViewWithIdentifier:@"cell" owner:self];
    if (!cell) {
        cell = [[NSTableCellView alloc] initWithFrame:NSMakeRect(0, 0, 200, 26)];
        cell.identifier = @"cell";
        NSImageView* icon = [[NSImageView alloc] initWithFrame:NSMakeRect(2, 5, 16, 16)];
        icon.autoresizingMask = NSViewMaxXMargin;
        [cell addSubview:icon];
        cell.imageView = icon;
        NSTextField* text = [NSTextField labelWithString:@""];
        text.frame = NSMakeRect(24, 4, 170, 18);
        text.autoresizingMask = NSViewWidthSizable;
        text.lineBreakMode = NSLineBreakByTruncatingTail;
        [cell addSubview:text];
        cell.textField = text;
    }
    int index = [item intValue];
    const Body& b = _world.bodies[index];
    cell.imageView.image = [NSImage imageWithSystemSymbolName:b.mass == 0 ? @"pin.fill" : shapeSymbol(b.shape)
                                     accessibilityDescription:nil];
    cell.imageView.contentTintColor = bodyTint(b, index);
    NSString* via = @"";
    for (const auto& j : _world.joints) {
        if (j.b == index) {
            via = [NSString stringWithFormat:@"  %@", jointName(j.type).lowercaseString];
            break;
        }
    }
    NSMutableAttributedString* text =
        [[NSMutableAttributedString alloc] initWithString:@(b.name.c_str())
                                               attributes:@{
                                                   NSFontAttributeName : [NSFont systemFontOfSize:12.5],
                                                   NSForegroundColorAttributeName : NSColor.labelColor
                                               }];
    [text appendAttributedString:[[NSAttributedString alloc]
                                     initWithString:via
                                         attributes:@{
                                             NSFontAttributeName : [NSFont systemFontOfSize:10.5],
                                             NSForegroundColorAttributeName : NSColor.secondaryLabelColor
                                         }]];
    NSMutableParagraphStyle* style = [[NSMutableParagraphStyle alloc] init];
    style.lineBreakMode = NSLineBreakByTruncatingTail;
    [text addAttribute:NSParagraphStyleAttributeName value:style range:NSMakeRange(0, text.length)];
    cell.textField.attributedStringValue = text;
    cell.toolTip = [NSString
        stringWithFormat:@"%@ · %.3g kg%@", shapeName(b.shape), b.mass, b.mass == 0 ? @" (fixed)" : @""];
    return cell;
}

- (void)outlineViewSelectionDidChange:(NSNotification*)notification {
    (void)notification;
    if (_syncingSelection) {
        return;
    }
    NSInteger row = _outline.selectedRow;
    int index = row >= 0 ? [[_outline itemAtRow:row] intValue] : -1;
    if (index != _selected) {
        [self selectBody:index];
    }
}

@end
