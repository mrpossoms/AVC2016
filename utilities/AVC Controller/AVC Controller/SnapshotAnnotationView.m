//
//  SnapshotAnnotationView.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//
#import "SnapshotAnnotationView.h"
#import "PointPlotControl.h"

#define POINTS(e) (sizeof(e) / sizeof(CGPoint))

@import MapKit;

@interface SnapshotAnnotationView()
@property (nonatomic) NSObject<MKAnnotation>* myAnno;
@property PointPlotControl* magPlot;
@end

@implementation SnapshotAnnotationView

- (void)setSnapshot:(sysSnap_t)snapshot
{
    _snapshot = snapshot;
    _snapshot.estimated.headingAngle = atan2f(snapshot.estimated.heading.y, snapshot.estimated.heading.x) + M_PI_2;

    [self.magPlot addPoint:CGPointMake(snapshot.imu.cal.mag.x, snapshot.imu.cal.mag.y)];
    self.magPlot.frame = self.bounds;
    [self.magPlot setNeedsDisplay];
}

- (void)setFrame:(CGRect)frame
{
    super.frame = frame;
    _magPlot.frame = self.bounds;
}

- (instancetype)initWithAnnotation:(id<MKAnnotation>)annotation reuseIdentifier:(NSString *)reuseIdentifier
{
    self = [super initWithAnnotation:annotation reuseIdentifier:reuseIdentifier];
    if(!self) return nil;

    self.opaque = NO;
    _myAnno = annotation;
    _magPlot = [[PointPlotControl alloc] initWithFrame:self.frame];
    _magPlot.opaque = NO;
//    _magPlot->min = CGPointMake(-3000, -3000);
//    _magPlot->max = CGPointMake( 3000,  3000);
//    _magPlot.minMaxSet = NO;

    static CGFloat pointColor[4] = { 1, 0, 0, 1 };
    static CGFloat clearColor[4] = { 0, 0, 0, 0.1 };

    [self addSubview: _magPlot];
    _magPlot->pointColor = pointColor;
    _magPlot->clearColor = clearColor;

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

- (void)scalePoints:(CGPoint*)points withCount:(NSInteger)n andFactor:(CGFloat)s
{
    for(;n--;){
        points[n].x *= s;
        points[n].y *= s;
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
    [super drawRect:rect];

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
        { 0, 0 }, { -_snapshot.imu.cal.mag.x, -_snapshot.imu.cal.mag.y }
    };

    CGPoint heading[] = {
        { 0, 0 }, { _snapshot.estimated.heading.x, _snapshot.estimated.heading.y }
    };

    CGPoint goal[] = {
        { 0, 0 }, { -_snapshot.estimated.goalHeading.x, _snapshot.estimated.goalHeading.y }
    };

    static CGFloat black[] = { 0, 0, 0, 1 };
    static CGFloat white[] = { 1, 1, 1, 1 };
    static CGFloat red[]  = { 1, 0, 0, 1 };
    static CGFloat blue[] = { 0, 0, 1, 1 };
    static CGFloat green[] = { 0, 1, 0, 1 };
    static CGFloat yellow[] = { 1, 1, 0, 1 };

    static CGFloat* axisColors[] = {
        blue,
        green,
        red
    };

    CGContextRef ctx = UIGraphicsGetCurrentContext();

    [self scalePoints:rawMag withCount:2 andFactor: 0.9 / sqrt(pow(rawMag[1].x, 2) + pow(rawMag[1].y, 2))];
    [self scalePoints:filteredMag withCount:2 andFactor: 0.9];
    [self scalePoints:heading withCount:2 andFactor:0.9];
    [self scalePoints:goal withCount:2 andFactor:0.9];

    [self rotatePoints:body withCount:POINTS(body) toAngle:_snapshot.estimated.headingAngle];
    [self rotatePoints:wheels withCount:POINTS(wheels) toAngle:_snapshot.estimated.headingAngle];

    [self transformPoints:body withCount:POINTS(body) to:rect];
    [self transformPoints:wheels withCount:POINTS(wheels) to:rect];

    [self transformPoints:rawMag withCount:2 to:rect];
    [self transformPoints:filteredMag withCount:2 to:rect];
    [self transformPoints:goal withCount:2 to:rect];
    [self transformPoints:heading withCount:2 to:rect];

    CGPoint accOrigin[] = { { 0, 15 }, { 20, 15 } };
    CGContextSetLineWidth(ctx, 0.5);
    CGContextSetStrokeColor(ctx, white);
    CGContextStrokeLineSegments(ctx, accOrigin, 2);
    CGContextSetLineWidth(ctx, 3);
    for(int i = 3; i--;){
        CGPoint line[] = {
            { 5 + i * 5, 15 },
            { 5 + i * 5, 15 + _snapshot.imu.cal.acc.v[i] * 2 }
        };

        CGContextSetStrokeColor(ctx, axisColors[i]);
        CGContextStrokeLineSegments(ctx, line, 2);
    }

    CGContextScaleCTM(ctx, 0.5, 0.5);
    CGContextTranslateCTM(ctx, rect.size.width / 2, rect.size.height / 2);

    CGContextSetStrokeColor(ctx, white);
    CGContextStrokeLineSegments(ctx, body, POINTS(body));

    CGContextSetStrokeColor(ctx, black);
    CGContextSetLineWidth(ctx, 4);
    CGContextStrokeLineSegments(ctx, wheels, POINTS(wheels));

    CGContextSetLineWidth(ctx, 3);

    CGContextSetStrokeColor(ctx, red);
    CGContextStrokeLineSegments(ctx, rawMag, 2);

    CGContextSetStrokeColor(ctx, yellow);
    CGContextStrokeLineSegments(ctx, filteredMag, 2);

    CGContextSetStrokeColor(ctx, green);
    CGContextStrokeLineSegments(ctx, goal, 2);

    CGContextSetStrokeColor(ctx, blue);
    CGContextStrokeLineSegments(ctx, heading, 2);
}

@end
