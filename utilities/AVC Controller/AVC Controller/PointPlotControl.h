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
    CGPoint*       points;
    NSUInteger     pointCount;
    const CGFloat* pointColor;
}

@property CGVector min, max;
@property BOOL   shouldClearOnRedraw;

@end
