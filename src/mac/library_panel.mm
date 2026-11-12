// Library window: see library_panel.h.

#import "mac/library_panel.h"
#import "mac/editor_controller.h"

using namespace robotics;
using namespace editor_ui;

@implementation LibraryPanel {
    NSMutableDictionary<NSString*, NSImage*>* _thumbnails;
}

- (instancetype)initWithEditor:(EditorController*)editor {
    self = [super init];
    if (!self) {
        return nil;
    }
    _editor = editor;
    _thumbnails = [NSMutableDictionary dictionary];
    _window = [[NSWindow alloc]
        initWithContentRect:NSMakeRect(0, 0, 560, 660)
                  styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable
                    backing:NSBackingStoreBuffered
                      defer:NO];
    _window.title = @"Part Library";
    _window.minSize = NSMakeSize(460, 420);
    _window.releasedWhenClosed = NO;
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    NSView* root = _window.contentView;
    NSRect b = root.bounds;

    _placement =
        [NSSegmentedControl segmentedControlWithLabels:@[ @"Attach to Selected Part", @"Place on the Floor" ]
                                          trackingMode:NSSegmentSwitchTrackingSelectOne
                                                target:self
                                                action:@selector(placementChanged:)];
    _placement.selectedSegment = 0;
    _placement.frame = NSMakeRect(12, b.size.height - 40, b.size.width - 24, 26);
    _placement.segmentDistribution = NSSegmentDistributionFillEqually;
    _placement.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
    _placement.toolTip =
        @"Attach: connect the module to the selected part. Floor: place it unattached near the "
        @"center of the 3D view.";
    [root addSubview:_placement];
    _hint = label(root, @"", NSMakeRect(14, b.size.height - 62, b.size.width - 28, 16), 11,
                  NSFontWeightRegular, NSColor.secondaryLabelColor);
    _hint.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;

    NSScrollView* scroll =
        [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 56, b.size.width, b.size.height - 126)];
    scroll.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    scroll.hasVerticalScroller = YES;
    _table = [[NSTableView alloc] initWithFrame:scroll.bounds];
    _table.headerView = nil;
    _table.style = NSTableViewStylePlain;
    _table.rowHeight = 84;
    NSTableColumn* column = [[NSTableColumn alloc] initWithIdentifier:@"item"];
    column.width = b.size.width - 20;
    column.resizingMask = NSTableColumnAutoresizingMask;
    [_table addTableColumn:column];
    _table.columnAutoresizingStyle = NSTableViewUniformColumnAutoresizingStyle;
    _table.dataSource = self;
    _table.delegate = self;
    _table.target = self;
    _table.doubleAction = @selector(insert:);
    scroll.documentView = _table;
    [root addSubview:scroll];

    _insert = pushButton(root, @"Insert", NSMakeRect(b.size.width - 112, 14, 100, 30), self,
                         @selector(insert:), @"plus");
    _insert.autoresizingMask = NSViewMinXMargin;
    _insert.keyEquivalent = @"\r";
    _insert.toolTip = @"Insert the chosen module (double-click also works). Undo with ⌘Z.";
    _save = pushButton(root, @"Save Selected Assembly…", NSMakeRect(12, 14, 196, 30), self,
                       @selector(saveAssembly:), @"square.and.arrow.down");
    _save.toolTip = @"Save the selected part and everything attached below it as a reusable item";
    _remove = pushButton(root, @"Remove Item", NSMakeRect(214, 14, 116, 30), self, @selector(removeItem:),
                         @"trash");
    _remove.toolTip = @"Delete the chosen saved item from the library (built-in modules stay)";
    [self reload];
    return self;
}

- (void)show {
    if (!_window.visible) {
        [_window center];
    }
    [self reload];
    [_window makeKeyAndOrderFront:nil];
}

- (void)reload {
    EditorController* editor = _editor;
    NSInteger row = _table.selectedRow;
    _items = [editor libraryItems];
    [_table reloadData];
    [_table selectRowIndexes:[NSIndexSet
                                 indexSetWithIndex:static_cast<NSUInteger>(std::max<NSInteger>(row, 0))]
        byExtendingSelection:NO];
    [self refresh];
}

- (void)refresh {
    EditorController* editor = _editor;
    int selected = editor->_selected;
    BOOL hasPart = selected >= 0 && selected < static_cast<int>(editor->_world.bodies.size());
    BOOL editable = !editor->_running && !editor->_reviewing;
    [_placement setEnabled:hasPart forSegment:0];
    if (!hasPart) {
        _placement.selectedSegment = 1;
    }
    if (!editable) {
        _hint.stringValue = @"Pause the simulation and return to Live to insert parts.";
    } else if (_placement.selectedSegment == 0) {
        _hint.stringValue = [NSString stringWithFormat:@"Attaches to “%@” at a free spot.",
                                                       @(editor->_world.bodies[selected].name.c_str())];
    } else {
        _hint.stringValue = @"Inserts unattached, near the center of the 3D view.";
    }
    _insert.enabled = editable && _table.selectedRow >= 0;
    _save.enabled = editable && hasPart;
    NSInteger row = _table.selectedRow;
    _remove.enabled = row >= 0 && row < static_cast<NSInteger>(_items.size()) && _items[row].custom;
}

- (void)placementChanged:(id)sender {
    (void)sender;
    [self refresh];
}

- (void)insert:(id)sender {
    (void)sender;
    NSInteger row = _table.selectedRow;
    if (row < 0 || row >= static_cast<NSInteger>(_items.size())) {
        return;
    }
    EditorController* editor = _editor;
    [editor insertItem:_items[static_cast<std::size_t>(row)] attach:_placement.selectedSegment == 0];
    [self refresh];
}

- (void)saveAssembly:(id)sender {
    [_editor saveAssemblyToLibrary:sender];
}

- (void)removeItem:(id)sender {
    (void)sender;
    NSInteger row = _table.selectedRow;
    if (row < 0 || !_items[static_cast<std::size_t>(row)].custom) {
        return;
    }
    const LibraryItem& item = _items[static_cast<std::size_t>(row)];
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = [NSString stringWithFormat:@"Remove “%@” from the library?", @(item.title.c_str())];
    alert.informativeText = @"The saved file moves to the Trash. Parts already in your robot stay.";
    [alert addButtonWithTitle:@"Remove"];
    [alert addButtonWithTitle:@"Cancel"];
    if ([alert runModal] == NSAlertFirstButtonReturn) {
        [NSFileManager.defaultManager trashItemAtURL:[NSURL fileURLWithPath:@(item.id.c_str())]
                                    resultingItemURL:nil
                                               error:nil];
        [_thumbnails removeObjectForKey:@(item.id.c_str())];
        [self reload];
    }
}

- (NSInteger)numberOfRowsInTableView:(NSTableView*)tableView {
    (void)tableView;
    return static_cast<NSInteger>(_items.size());
}

- (NSImage*)thumbnail:(const LibraryItem&)item {
    NSString* key = @(item.id.c_str());
    NSImage* image = _thumbnails[key];
    if (!image) {
        EditorController* editor = _editor;
        image = [editor->_scene thumbnailOfWorld:item.assembly width:208 height:152];
        if (image) {
            _thumbnails[key] = image;
        }
    }
    return image;
}

- (NSView*)tableView:(NSTableView*)tableView viewForTableColumn:(NSTableColumn*)column row:(NSInteger)row {
    (void)column;
    const LibraryItem& item = _items[static_cast<std::size_t>(row)];
    NSView* cell = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, tableView.bounds.size.width, 84)];
    NSImageView* picture = [[NSImageView alloc] initWithFrame:NSMakeRect(8, 6, 100, 72)];
    picture.image = [self thumbnail:item];
    picture.imageScaling = NSImageScaleProportionallyUpOrDown;
    picture.wantsLayer = YES;
    picture.layer.cornerRadius = 6;
    picture.layer.masksToBounds = YES;
    [cell addSubview:picture];
    CGFloat width = tableView.bounds.size.width - 130;
    NSTextField* title = [NSTextField labelWithString:@(item.title.c_str())];
    title.font = [NSFont systemFontOfSize:13 weight:NSFontWeightSemibold];
    title.frame = NSMakeRect(118, 56, width, 18);
    title.autoresizingMask = NSViewWidthSizable;
    [cell addSubview:title];
    NSTextField* category = [NSTextField labelWithString:@(item.category.c_str())];
    category.font = [NSFont systemFontOfSize:10.5 weight:NSFontWeightMedium];
    category.textColor = NSColor.tertiaryLabelColor;
    category.frame = NSMakeRect(118, 42, width, 14);
    [cell addSubview:category];
    NSTextField* summary = [NSTextField wrappingLabelWithString:@(item.summary.c_str())];
    summary.font = [NSFont systemFontOfSize:11];
    summary.textColor = NSColor.secondaryLabelColor;
    summary.frame = NSMakeRect(118, 6, width, 34);
    summary.autoresizingMask = NSViewWidthSizable;
    [cell addSubview:summary];
    cell.toolTip = @(item.summary.c_str());
    return cell;
}

- (void)tableViewSelectionDidChange:(NSNotification*)notification {
    (void)notification;
    [self refresh];
}

@end
