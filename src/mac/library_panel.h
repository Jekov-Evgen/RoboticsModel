#pragma once
// Library window: ready-made modules and the user's saved assemblies.

#import <Cocoa/Cocoa.h>

#include "robotics/editing/library.hpp"

#include <vector>

@class EditorController;

// Instance variables are @public for the editor smoke test (src/mac/editor_smoke_test.mm).
@interface LibraryPanel : NSObject <NSTableViewDataSource, NSTableViewDelegate> {
  @public
    __weak EditorController* _editor;
    NSWindow* _window;
    NSTableView* _table;
    NSSegmentedControl* _placement; // 0 attach to the selected part, 1 place on the floor.
    NSTextField* _hint;
    NSButton* _insert;
    NSButton* _save;
    NSButton* _remove;
    std::vector<robotics::LibraryItem> _items;
}
- (instancetype)initWithEditor:(EditorController*)editor;
- (void)show;
- (void)reload;  // Rereads saved items.
- (void)refresh; // Placement and button states.
- (void)insert:(id)sender;
@end
