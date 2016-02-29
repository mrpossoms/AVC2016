//
//  SnapshotAnnotationView.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SnapshotAnnotationView.h"

#define POINTS(e) (sizeof(e) / sizeof(CGPoint))

@implementation SnapshotAnnotationView

- (instancetype)initWithAnnotation:(id<MKAnnotation>)annotation reuseIdentifier:(NSString *)reuseIdentifier
{
    self = [super initWithAnnotation:annotation reuseIdentifier:reuseIdentifier];
    if(!self) return nil;

    self.opaque = NO;

    return self;
}

- (void)rotatePoints:(CGPoint*)points withCount:(NSInteger)n toAngle:(CGFloat)theta
{
    float c = cos(theta), s = sin(theta);
    for(;n--;){
        CGPoint* v = points + n;
        points[n] = CGPointMake(c * v->x - s * v->y, s * v->x + c * v->y);
    }
}

- (void)transformPoints:(CGPoint*)points withCount:(NSInteger)n to:(CGRect)rect
{
    CGSize size = { rect.size.width / 2, rect.size.height / 2 };

    for(;n--;){
        points[n].x = points[n].x * size.width + size.width;
        points[n].y = points[n].y * size.height + size.height;
    }
}

- (void)drawRect:(CGRect)rect
{
    CGPoint body[] = {
        // chassis
        { 0, -0.8 }, { 0.3, 0.2 },
        { 0.3, 0.2 }, { 0, 0.6 },
        { 0, -0.8 }, { -0.3, 0.2 },
        { -0.3, 0.2 }, { 0, 0.6 },

        // swing arms
        { -0.4, -0.8 }, { 0.4, -0.8 },
        { -0.4, 0.6 },  { 0.4, 0.6 },

    };

    CGPoint wheels[] = {
        { -0.4, -1.0 }, { -0.4, -0.6 },
        { -0.4,  0.8 }, { -0.4,  0.4 },

        { 0.4, -1.0 }, { 0.4, -0.6 },
        { 0.4,  0.8 }, { 0.4,  0.4 },
    };

    CGPoint rawMag[] = {
        { 0, 0 }, { _snapshot.imu.raw.mag.x, _snapshot.imu.raw.mag.y }
    };

    CGPoint filteredMag[] = {
        { 0, 0 }, { _snapshot.imu.adj.mag.x, _snapshot.imu.adj.mag.y }
    };

    CGPoint goal[] = {
        { 0, 0 }, { _snapshot.estimated.goalHeading.x, _snapshot.estimated.goalHeading.y }
    };

    static CGFloat black[] = { 0, 0, 0, 1 };
    static CGFloat white[] = { 1, 1, 1, 1 };
    static CGFloat red[]  = { 1, 0, 0, 1 };
    static CGFloat blue[] = { 0, 0, 1, 1 };
    static CGFloat green[] = { 0, 1, 0, 1 };


    CGContextRef ctx = UIGraphicsGetCurrentContext();



    [self rotatePoints:body withCount:POINTS(body) toAngle:_snapshot.estimated.headingAngle];
    [self rotatePoints:wheels withCount:POINTS(wheels) toAngle:_snapshot.estimated.headingAngle];

    [self transformPoints:body withCount:POINTS(body) to:rect];
    [self transformPoints:wheels withCount:POINTS(wheels) to:rect];

    CGContextSetStrokeColor(ctx, white);
    CGContextStrokeLineSegments(ctx, body, POINTS(body));

    CGContextSetStrokeColor(ctx, black);
    CGContextSetLineWidth(ctx, 4);
    CGContextStrokeLineSegments(ctx, wheels, POINTS(wheels));

    CGContextSetStrokeColor(ctx, red);
    CGContextStrokeLineSegments(ctx, rawMag, 2);

    CGContextSetStrokeColor(ctx, blue);
    CGContextStrokeLineSegments(ctx, filteredMag, 2);

    CGContextSetStrokeColor(ctx, green);
    CGContextStrokeLineSegments(ctx, goal, 2);

}

@end
