//
//  waypoint.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/3/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "Waypoint.h"

@implementation Waypoint

- (instancetype)initWithPosition:(CLLocationCoordinate2D)location
{
    self = [super initWithAnnotation:self reuseIdentifier:@"WAYPOINT"];
    if(!self) return self;
    
    self.coordinate = location;
    self.title = @"Waypoint";
    
    return self;
}

- (instancetype)addNext:(CLLocationCoordinate2D)location
{
    Waypoint* new = [[Waypoint alloc] initWithPosition:location];

    new.previous = self;
    self.next    = new;
    
    return new;
}

- (float)distanceTo:(Waypoint*)waypoint
{
    CLLocation* l0 = [[CLLocation alloc] initWithLatitude:self.coordinate.latitude longitude:self.coordinate.longitude];
    CLLocation* l1 = [[CLLocation alloc] initWithLatitude:waypoint.coordinate.latitude longitude:waypoint.coordinate.longitude];
    
    return [l0 distanceFromLocation:l1];
}

- (void)remove
{
    self.previous.next = self.next;
    self.next.previous = self.previous;
}

- (int)length
{
    if(self.next){
        return 1 + [self.next length];
    }
    
    return 1;
}

@end
