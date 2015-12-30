//
//  ThumbStickView.h
//  AVC Controller
//
//  Created by Kirk Roerig on 12/29/15.
//  Copyright © 2015 PossomGames. All rights reserved.
//

#import <UIKit/UIKit.h>

@class ThumbStickView;
@protocol ThumbStickDelegate

- (void)didThumbMoveWithValues:(CGPoint)values asSender:(ThumbStickView*)sender;

@end

@interface ThumbStickView : UIView

@property BOOL xAxisDisabled, yAxisDisabled;
@property (weak) id<ThumbStickDelegate> delegate;

- (void)reset;
- (void)setRangeForAxis:(int)axis withMin:(float)min andMax:(float)max;

@end
