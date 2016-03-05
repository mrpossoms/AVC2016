//
//  PointPlotControl.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/8/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>

@interface PointPlotControl : UIView{
    @public
    CGPoint*       points;
    NSUInteger     pointCount;
    const CGFloat *pointColor, *clearColor;

    CGPoint min, max;
}

@property BOOL minMaxSet;

- (void)addPoint:(CGPoint)magReading;

+ (instancetype)plotWithFrame:(CGRect)frame;

@end
