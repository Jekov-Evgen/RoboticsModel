#import "mac/app_icon.h"
#include <iostream>

int main(int argc, char** argv) {
    @autoreleasepool {
        if (argc != 2) {
            std::cerr << "Usage: icon_builder OUTPUT.iconset\n";
            return 1;
        }
        NSString* directory = [NSString stringWithUTF8String:argv[1]];
        NSError* error = nil;
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directory
                                       withIntermediateDirectories:YES
                                                        attributes:nil
                                                             error:&error]) {
            std::cerr << error.localizedDescription.UTF8String << '\n';
            return 1;
        }
        for (int size : {16, 32, 128, 256, 512}) {
            for (int factor : {1, 2}) {
                NSBitmapImageRep* bitmap = renderRoboticsIcon(size * factor);
                NSData* data = [bitmap representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
                NSString* name =
                    [NSString stringWithFormat:@"icon_%dx%d%@.png", size, size, factor == 2 ? @"@2x" : @""];
                if (![data writeToFile:[directory stringByAppendingPathComponent:name] atomically:YES]) {
                    std::cerr << "Failed to write icon PNG\n";
                    return 1;
                }
            }
        }
        std::cout << "Generated application icon at 10 resolutions\n";
    }
    return 0;
}
