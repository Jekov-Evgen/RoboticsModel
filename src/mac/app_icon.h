#pragma once
#import <Cocoa/Cocoa.h>
#include <initializer_list>

// Resolution-independent native artwork; shared by the Dock icon and .icns builder.
inline NSBitmapImageRep* renderRoboticsIcon(int pixels) {
    NSBitmapImageRep* bitmap = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:nullptr
                                                                       pixelsWide:pixels
                                                                       pixelsHigh:pixels
                                                                    bitsPerSample:8
                                                                  samplesPerPixel:4
                                                                         hasAlpha:YES
                                                                         isPlanar:NO
                                                                   colorSpaceName:NSDeviceRGBColorSpace
                                                                      bytesPerRow:0
                                                                     bitsPerPixel:0];
    [NSGraphicsContext saveGraphicsState];
    NSGraphicsContext* context = [NSGraphicsContext graphicsContextWithBitmapImageRep:bitmap];
    [NSGraphicsContext setCurrentContext:context];
    context.shouldAntialias = YES;
    NSAffineTransform* scale = [NSAffineTransform transform];
    [scale scaleBy:pixels / 1024.0];
    [scale concat];
    auto color = [](CGFloat r, CGFloat g, CGFloat b) {
        return [NSColor colorWithCalibratedRed:r green:g blue:b alpha:1];
    };
    auto round = [&](NSRect rect, CGFloat radius, NSColor* fill) {
        [fill setFill];
        [[NSBezierPath bezierPathWithRoundedRect:rect xRadius:radius yRadius:radius] fill];
    };
    auto circle = [&](NSRect rect, NSColor* fill) {
        [fill setFill];
        [[NSBezierPath bezierPathWithOvalInRect:rect] fill];
    };
    auto line = [&](NSPoint a, NSPoint b, CGFloat width, NSColor* stroke) {
        [stroke setStroke];
        NSBezierPath* path = [NSBezierPath bezierPath];
        path.lineWidth = width;
        path.lineCapStyle = NSLineCapStyleRound;
        [path moveToPoint:a];
        [path lineToPoint:b];
        [path stroke];
    };
    round(NSMakeRect(46, 38, 932, 932), 214, color(0.025, 0.045, 0.08));
    NSBezierPath* tile = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(48, 56, 928, 920)
                                                         xRadius:210
                                                         yRadius:210];
    NSGradient* background = [[NSGradient alloc] initWithStartingColor:color(0.06, 0.11, 0.18)
                                                           endingColor:color(0.15, 0.25, 0.34)];
    [background drawInBezierPath:tile angle:60];
    [NSGraphicsContext saveGraphicsState];
    [tile addClip];
    NSColor* grid = [NSColor colorWithCalibratedWhite:0.7 alpha:0.055];
    for (int k = 160; k < 1000; k += 112) {
        line(NSMakePoint(k, 60), NSMakePoint(k, 970), 2, grid);
        line(NSMakePoint(48, k), NSMakePoint(976, k), 2, grid);
    }
    [NSGraphicsContext restoreGraphicsState];
    circle(NSMakeRect(190, 167, 644, 140), [NSColor colorWithCalibratedWhite:0 alpha:0.22]);
    // Wide tires with machined tread highlights.
    for (CGFloat x : {168.0, 688.0}) {
        round(NSMakeRect(x, 232, 168, 316), 55, color(0.035, 0.055, 0.085));
        round(NSMakeRect(x + 18, 250, 132, 280), 38, color(0.12, 0.17, 0.22));
        for (int y = 280; y < 510; y += 47) {
            line(NSMakePoint(x + 28, y), NSMakePoint(x + 140, y), 12, color(0.19, 0.26, 0.32));
        }
    }
    round(NSMakeRect(242, 289, 540, 335), 75, color(0.02, 0.34, 0.4));
    NSBezierPath* body = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(240, 315, 544, 322)
                                                         xRadius:72
                                                         yRadius:72];
    NSGradient* turquoise = [[NSGradient alloc] initWithStartingColor:color(0.05, 0.59, 0.66)
                                                          endingColor:color(0.28, 0.93, 0.91)];
    [turquoise drawInBezierPath:body angle:90];
    round(NSMakeRect(311, 349, 402, 83), 27, color(0.035, 0.2, 0.27));
    for (int k = 0; k < 5; ++k) {
        round(NSMakeRect(352 + k * 68, 371, 42, 39), 10, color(0.12, 0.41, 0.46));
    }
    line(NSMakePoint(512, 684), NSMakePoint(512, 765), 26, color(0.46, 0.64, 0.69));
    line(NSMakePoint(512, 765), NSMakePoint(586, 813), 22, color(0.46, 0.64, 0.69));
    circle(NSMakeRect(552, 787, 68, 68), color(1, 0.65, 0.26));
    round(NSMakeRect(263, 458, 498, 274), 75, color(0.025, 0.09, 0.14));
    round(NSMakeRect(283, 482, 458, 230), 58, color(0.16, 0.25, 0.31));
    round(NSMakeRect(308, 512, 408, 165), 44, color(0.025, 0.075, 0.12));
    for (CGFloat x : {365.0, 570.0}) {
        circle(NSMakeRect(x - 10, 542, 104, 104), [NSColor colorWithCalibratedRed:0.15
                                                                            green:0.9
                                                                             blue:0.93
                                                                            alpha:0.1]);
        round(NSMakeRect(x, 558, 84, 72), 25, color(0.28, 0.97, 0.95));
        round(NSMakeRect(x + 12, 602, 54, 10), 5, color(0.75, 1, 1));
    }
    line(NSMakePoint(466, 531), NSMakePoint(558, 531), 9, color(0.32, 0.53, 0.6));
    round(NSMakeRect(718, 716, 72, 20), 10, color(1, 0.65, 0.26));
    [NSGraphicsContext restoreGraphicsState];
    return bitmap;
}

inline NSImage* roboticsAppIcon() {
    NSImage* image = [[NSImage alloc] initWithSize:NSMakeSize(512, 512)];
    [image addRepresentation:renderRoboticsIcon(1024)];
    return image;
}
