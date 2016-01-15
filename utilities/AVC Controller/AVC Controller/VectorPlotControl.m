//
//  VectorPlotControl.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/14/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "VectorPlotControl.h"

@implementation VectorPlotControl

- (void)commonInit
{
    static const CGFloat white[] = { 1, 1, 1, 1 };
    
}

- (instancetype)init
{
    self = [super init];
    if(self){
        [self commonInit];
    }
    
    return self;
}

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if(self){
        [self commonInit];
    }
    
    return self;
}

- (void)drawRect:(CGRect)rect
{
    const CGFloat clearColor[] = { 1, 1, 1, 1 };
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
    
    //    const float s = 1.25f;
    //    min.x *= s; min.y *= s;
    //    max.x *= s; max.y *= s;
    
    for(NSUInteger i = pointCount; i--;){
        CGPoint point = points[i];
        
        CGPoint norm = CGPointMake(point.x, point.y);
        
        CGPoint transformed = { norm.x * radius.dx + radius.dx, norm.y * radius.dy + radius.dy};
        CGPoint line[] = { center, transformed };
        
        CGContextSetStrokeColor(ctx, pointColor[i]);
        CGContextStrokeLineSegments(ctx, line, 2);
    }
    
    NSMutableParagraphStyle* style = [[[NSParagraphStyle alloc] init] mutableCopy];
    NSDictionary* attrs = nil;
    style.alignment = NSTextAlignmentLeft;
    CGFloat textBoxWidth = radius.dx / 2;
    [[NSString stringWithFormat:@"%f", -1.0f] drawInRect:CGRectMake(horizontalAxis[0].x, horizontalAxis[0].y, textBoxWidth, 20) withAttributes:attrs];
    
    style.alignment = NSTextAlignmentRight;
    [[NSString stringWithFormat:@"%f", 1.0f] drawInRect:CGRectMake(horizontalAxis[1].x - textBoxWidth, horizontalAxis[1].y, textBoxWidth, 20) withAttributes:attrs];
    
    style.alignment = NSTextAlignmentLeft;
    [[NSString stringWithFormat:@"%f", -1.0f] drawInRect:CGRectMake(verticalAxis[0].x, verticalAxis[0].y, textBoxWidth, 20) withAttributes:attrs];
    
    style.alignment = NSTextAlignmentRight;
    [[NSString stringWithFormat:@"%f", 1.0f] drawInRect:CGRectMake(verticalAxis[1].x, verticalAxis[1].y - 20, textBoxWidth, 20) withAttributes:attrs];
}

+ (instancetype)plotWithFrame:(CGRect)frame
{
    return [[VectorPlotControl alloc] initWithFrame:frame];
}

@end
