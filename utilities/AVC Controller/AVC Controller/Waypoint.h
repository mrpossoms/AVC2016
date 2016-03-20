//
//  waypoint.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/3/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <Foundation/Foundation.h>

#include "base/types.h"

@import CoreLocation;
@import MapKit;

@class Waypoint;
@interface Waypoint : MKAnnotationView <MKAnnotation>

@property (nonatomic) CLLocationCoordinate2D coordinate;
@property (nonatomic, copy) NSString *title;
@property float tolerance;
@property Waypoint *next, *previous;

- (instancetype)initWithPosition:(CLLocationCoordinate2D)location;
- (instancetype)addNext:(CLLocationCoordinate2D)location;
- (void)remove;
- (int)length;
- (float)distanceTo:(Waypoint*)waypoint;
@end
