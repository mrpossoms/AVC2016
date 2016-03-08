//
//  PointPlotControl.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/8/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "PointPlotControl.h"
#include "system.h"

@interface PointPlotControl()

@property uint8_t index;

@end

@implementation PointPlotControl

- (void)commonInit
{
    static const CGFloat black[] = { 0, 0, 0, 1 };
    static const CGFloat white[] = { 1, 1, 1, 1 };
    
    pointColor = black;
    clearColor = white;

    self->points = (CGPoint*)calloc(256, sizeof(CGPoint));
}

- (instancetype)init
{
    self = [super init];
    if(self){
        [self commonInit];
    }
    
    return self;
}

- (void)dealloc
{
    free(self->points);
}

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if(self){
        [self commonInit];        
    }
    
    return self;
}

- (void)addPoint:(CGPoint)magReading
{
    points[_index] = magReading;

    _index++;

    if(_index > pointCount){
        pointCount = _index;
    }
}

- (BOOL)findMinMax
{
    if(!points) return NO;
    
    for(NSUInteger i = pointCount; i--;){
        if(points[i].x > max.x){
            max.x = points[i].x;
        }
        if(points[i].y > max.y){
            max.y = points[i].y;
        }

        if(points[i].x < min.x){
            min.x = points[i].x;
        }
        if(points[i].y < min.y){
            min.y = points[i].y;
        }
    }
    
    return YES;
}

- (void)drawRect:(CGRect)rect
{
    CGContextRef ctx = UIGraphicsGetCurrentContext();

    CGContextClearRect(ctx, rect);
    CGContextSetFillColor(ctx, clearColor);
    CGContextFillRect(ctx, rect);

    CGPoint center  = { rect.size.width / 2, rect.size.height / 2 };
    CGVector radius = { center.x, center.y };
    const CGFloat strokeColor[] = { 0.75, 0.75, 0.75, 1.0 };
    CGContextSetStrokeColor(ctx, strokeColor);
    
    CGPoint verticalAxis[]   = { { center.x, 0 }, { center.x, rect.size.height } };
    CGPoint horizontalAxis[] = { { 0, center.y }, { rect.size.width, center.y } };
    CGContextStrokeLineSegments(ctx, verticalAxis, 2);
    CGContextStrokeLineSegments(ctx, horizontalAxis, 2);
    
    
    if(!self.minMaxSet && ![self findMinMax]) return;
    
//    const float s = 1.25f;
//    min.x *= s; min.y *= s;
//    max.x *= s; max.y *= s;

    CGFloat color[4];
    memcpy(color, pointColor, sizeof(color));
    color[3] *= 0.5;

    CGContextSetFillColor(ctx, color);
    for(NSUInteger i = pointCount; i--;){
        CGPoint point = points[i];

        CGPoint norm = CGPointMake(
           (point.x - min.x) * 2 / (max.x - min.x) - 1,
           (point.y - min.y) * 2 / (max.y - min.y) - 1
        );
        
        
        CGPoint transformed = { norm.x * radius.dx + radius.dx, norm.y * radius.dy + radius.dy};
        CGContextFillRect(ctx, CGRectMake(transformed.x - 1, transformed.y - 1, 2, 2));
    }
    
    NSMutableParagraphStyle* style = [[[NSParagraphStyle alloc] init] mutableCopy];
    NSDictionary* attrs = nil;
    style.alignment = NSTextAlignmentLeft;
    CGFloat textBoxWidth = radius.dx / 2;
    [[NSString stringWithFormat:@"%f", min.x] drawInRect:CGRectMake(horizontalAxis[0].x, horizontalAxis[0].y, textBoxWidth, 20) withAttributes:attrs];
    
    style.alignment = NSTextAlignmentRight;
    [[NSString stringWithFormat:@"%f", max.x] drawInRect:CGRectMake(horizontalAxis[1].x - textBoxWidth, horizontalAxis[1].y, textBoxWidth, 20) withAttributes:attrs];

    style.alignment = NSTextAlignmentLeft;
    [[NSString stringWithFormat:@"%f", min.y] drawInRect:CGRectMake(verticalAxis[0].x, verticalAxis[0].y, textBoxWidth, 20) withAttributes:attrs];
    
    style.alignment = NSTextAlignmentRight;
    [[NSString stringWithFormat:@"%f", max.y] drawInRect:CGRectMake(verticalAxis[1].x, verticalAxis[1].y - 20, textBoxWidth, 20) withAttributes:attrs];
}

+ (instancetype)plotWithFrame:(CGRect)frame
{
    return [[PointPlotControl alloc] initWithFrame:frame];
}

@end
