//
//  PointPlotControl.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/8/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "PointPlotControl.h"

@implementation PointPlotControl

- (void)commonInit
{
    static const CGFloat black[] = { 0, 0, 0, 1 };
    
    pointColor = black;
    _min = CGVectorMake(-1, -1);
    _max = CGVectorMake( 1,  1);
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
    CGContextRef ctx = UIGraphicsGetCurrentContext();
    
    if(self.shouldClearOnRedraw){
        CGContextClearRect(ctx, rect);
    }
        
    CGContextSetFillColor(ctx, pointColor);
    
    for(NSUInteger i = pointCount; i--;){
        CGPoint point = points[i];
        
        CGContextFillRect(ctx, CGRectMake(point.x - 1, point.y - 1, 2, 2));
    }
}

@end
