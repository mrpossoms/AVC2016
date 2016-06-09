//
//  WaypointsOverlay.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/29/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "WaypointsOverlay.h"

@implementation WaypointsOverlay


//- (MKMapRect)boundingMapRect
//{
//    CLLocationCoordinate2D min = self.currentGoal;
//
//    min.longitude = min.longitude < self.nextGoal.longitude ? min.longitude : self.nextGoal.longitude;
//    min.latitude = min.latitude < self.nextGoal.latitude ? min.latitude : self.nextGoal.latitude;
//
//    CGFloat width = fabs(self.currentGoal.longitude - self.nextGoal.longitude);
//    CGFloat height = fabs(self.currentGoal.latitude - self.nextGoal.latitude);
//
//    return MKMapRectMake(min.longitude, min.latitude, width, height);
//}

- (NSString*)title
{
    return @"Waypoint";
}

- (CLLocationCoordinate2D)coordinate
{
    const float dia = 6371000 * 2; // diameter of the earth (meters)

    return CLLocationCoordinate2DMake(
                                      (_location.y / dia) / (M_PI / 180.0f),
                                      (_location.x / dia) / (M_PI / 180.0f)
                                      );
}

+ (instancetype)overlayAt:(vec3d_t)location
{
    WaypointsOverlay* o = [[WaypointsOverlay alloc] init];

    o.location = location;

    return o;
}

@end
