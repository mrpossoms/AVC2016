//
//  VectorPlotControl.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/14/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface VectorPlotControl : UIView{
@public
    CGPoint*        points;
    const CGFloat **pointColor;
    NSUInteger      pointCount;
}

+ (instancetype)plotWithFrame:(CGRect)frame;

@end
