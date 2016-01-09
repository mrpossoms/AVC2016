//
//  ThumbStickView.m
//  AVC Controller
//
//  Created by Kirk Roerig on 12/29/15.
//  Copyright © 2015 PossomGames. All rights reserved.
//

@import CoreGraphics;

#import "ThumbStickControl.h"

@interface ThumbStickControl(){
    float axisMinMax[2][2];
}

@property (nonatomic) CGPoint stickPos;

@property UITouch* currentTouch;

@end

@implementation ThumbStickControl

- (void)setStickPos:(CGPoint)stickPos
{
    _stickPos = stickPos;
    [self setNeedsDisplay];
}

- (CGPoint)viewCenter
{
    CGRect rect = self.bounds;
    return CGPointMake(rect.origin.x + rect.size.width / 2, rect.origin.x + rect.size.height / 2);
}

- (void)reset
{
    self.stickPos = [self viewCenter];
    
    axisMinMax[0][0] = -1;
    axisMinMax[0][1] =  1;
    axisMinMax[1][0] = -1;
    axisMinMax[1][1] =  1;
}

- (void)touchesBegan:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    self.currentTouch = [touches anyObject];
}

- (void)touchesMoved:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    CGPoint point = [self.currentTouch locationInView:self];
    if(point.x < self.bounds.origin.x) point.x = self.bounds.origin.x;
    if(point.x > self.bounds.origin.x + self.bounds.size.width) point.x = self.bounds.origin.x + self.bounds.size.width;
    if(point.y < self.bounds.origin.y) point.y = self.bounds.origin.y;
    if(point.y > self.bounds.origin.y + self.bounds.size.height) point.x = self.bounds.origin.y + self.bounds.size.height;
    
    self.stickPos = point;
}

- (void)touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    self.currentTouch = nil;
    self.stickPos = [self viewCenter];
}

- (void)setRangeForAxis:(int)axis withMin:(float)min andMax:(float)max
{
    axisMinMax[axis][0] = min;
    axisMinMax[axis][1] = max;
}

- (void)mapValuesInformDelegate:(CGPoint)stickPos
{
    CGPoint center = [self viewCenter];
    
    stickPos.x -= center.x;
    stickPos.y -= center.y;
    
    stickPos.x /= center.x;
    stickPos.y /= center.y;
    
    float px = ((stickPos.x + 1) / 2);
    float py = ((stickPos.y + 1) / 2);
    
    float x = axisMinMax[0][0] * (1 - px) + px * axisMinMax[0][1];
    float y = axisMinMax[1][0] * (1 - py) + py * axisMinMax[1][1];
    
    NSLog(@"(%f, %f)", x, y);
    
    [self.delegate didThumbMoveWithValues:CGPointMake(x, y) asSender:self];
}

- (void)drawRect:(CGRect)rect
{
    CGContextRef ctx = UIGraphicsGetCurrentContext();
    
    CGColorSpaceRef space = CGColorSpaceCreateDeviceRGB();
    CGFloat colorComponents[] = {
        0.0,  0.0,  0.0, 1.0,
        0.75, 0.75, 0.75, 1.0
    };
    CGFloat locations[] = { 0, 0.75 };
    CGGradientRef grad = CGGradientCreateWithColorComponents(space, colorComponents, locations, 2);
    CGPoint center = self.stickPos;
    
    if(self.xAxisDisabled) center.x = [self viewCenter].x;
    if(self.yAxisDisabled) center.y = [self viewCenter].y;
    
    [self mapValuesInformDelegate:center];
    
    CGContextDrawRadialGradient(ctx, grad, center, 0, center, sqrt(pow(rect.size.width, 2) + pow(rect.size.height, 2)), kCGGradientDrawsAfterEndLocation);
}

@end
