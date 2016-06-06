//
//  SnapshotDisplay.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SnapshotMapView.h"
#import "SnapshotAnnotationView.h"
#import "WaypointsOverlay.h"

@interface SnapshotMapView(){
    vec3f_t lastWaypoint;
}

@property CLLocationManager* locationManager;
@property SnapshotAnnotationView* carAnnotationView;
@property vec3f_t lastPosition;

@end

@implementation SnapshotMapView

- (void)setSnapshot:(sysSnap_t)snapshot
{
    vec3f_t h;

    if(vec3Dist(_lastPosition, _snapshot.pose.pos) >= 9){
        vec3Lerp(_lastPosition, _lastPosition, _snapshot.pose.pos, 0.25);
    }

    _snapshot = snapshot;

    const float dia = 6371000 * 2; // diameter of the earth (meters)
    [self setCoordinate:CLLocationCoordinate2DMake(
                                                   (_snapshot.pose.pos.y / dia) / (M_PI / 180.0f),
                                                   (_snapshot.pose.pos.x / dia) / (M_PI / 180.0f)
                                                   )];

    if(memcpy(&lastWaypoint, &_snapshot.currentWaypoint.location, sizeof(vec3f_t))){
        NSMutableArray<id<MKAnnotation>>* toRemove = [NSMutableArray array];
        for(id<MKAnnotation> annotation in self.annotations){
            if([annotation isKindOfClass:[WaypointsOverlay class]]){
                [toRemove addObject:annotation];
            }
        }

        [self removeAnnotations:toRemove];
        [self addAnnotation:[WaypointsOverlay overlayAt:_snapshot.currentWaypoint.location]];
        [self addAnnotation:[WaypointsOverlay overlayAt:_snapshot.nextWaypoint.location]];
    }


    vec3Sub(h, snapshot.pose.pos, _lastPosition);
//    dispatch_async(dispatch_get_main_queue(), ^{
        [self focusOnLocation:[[CLLocation alloc] initWithLatitude:self.coordinate.latitude longitude:self.coordinate.longitude]];
        self.carAnnotationView.snapshot = snapshot;
//        self.carAnnotationView.center = [self convertCoordinate:self.coordinate toPointToView:self];
        [self.carAnnotationView setNeedsDisplay];
//    });
}

- (instancetype)initWithCoder:(NSCoder *)aDecoder
{
    self = [super initWithCoder:aDecoder];
    if(!self) return nil;

    self.showsBuildings = NO;
    self.showsPointsOfInterest = NO;
    self.delegate = self;
    self.showsScale = YES;
    self.mapType = MKMapTypeSatellite;

    if([CLLocationManager locationServicesEnabled]){
        self.locationManager = [[CLLocationManager alloc] init];
        self.locationManager.activityType = CLActivityTypeOther;
        self.locationManager.delegate = self;

        [self.locationManager requestWhenInUseAuthorization];
        [self.locationManager startUpdatingLocation];
    }

    [self addAnnotation:self];

    return self;
}


- (void)focusOnLocation:(CLLocation*)location
{
    if(location.coordinate.longitude > -180 && location.coordinate.longitude < 180 &&
       location.coordinate.latitude > -90 && location.coordinate.latitude < 90){
        BOOL isAnimated = fabs(self.region.center.latitude - location.coordinate.latitude) < 1;
        [self setRegion:MKCoordinateRegionMakeWithDistance(location.coordinate, 10, 10) animated:isAnimated];
    }
}

#pragma mark - MKAnnotation protocol

- (NSString*)title
{
    return @"Das Robit";
}

#pragma mark - Location manager delegate


- (void)locationManager:(CLLocationManager *)manager didChangeAuthorizationStatus:(CLAuthorizationStatus)status
{
    if(status != kCLAuthorizationStatusAuthorizedWhenInUse){
        UIAlertView* locationNeeded = [[UIAlertView alloc] initWithTitle:@"Location Services Required"
                                                                 message:@"Do it dummy."
                                                                delegate:self
                                                       cancelButtonTitle:@"OK"
                                                       otherButtonTitles:nil];
        locationNeeded.tag = 0;
    }
}

#pragma mark - Map view delegate

- (void)mapView:(MKMapView *)mapView didUpdateUserLocation:(MKUserLocation *)userLocation
{
    [self focusOnLocation:userLocation.location];
    [self.locationManager stopUpdatingLocation];
}

- (nullable MKAnnotationView *)mapView:(MKMapView *)mapView viewForAnnotation:(id <MKAnnotation>)annotation
{
    if([annotation isKindOfClass:[self class]]){
        self.carAnnotationView = (SnapshotAnnotationView*)[mapView dequeueReusableAnnotationViewWithIdentifier:@"RobitAnnotation"];

        if(!self.carAnnotationView){
            self.carAnnotationView = [[SnapshotAnnotationView alloc] initWithAnnotation:annotation reuseIdentifier:@"RobitAnnotation"];
        }
        self.carAnnotationView.bounds = CGRectMake(0, 0, 160, 160);
        self.carAnnotationView.snapshot = _snapshot;

        [self.carAnnotationView setNeedsDisplay];

        return self.carAnnotationView;
    }

    if([annotation isKindOfClass:[WaypointsOverlay class]]){
        MKPinAnnotationView* view = (MKPinAnnotationView*)[mapView dequeueReusableAnnotationViewWithIdentifier:@"RobitWaypoint"];

        if(!view){
            view = [[MKPinAnnotationView alloc] initWithAnnotation:annotation reuseIdentifier:@"RobitWaypoint"];
        }

        return view;
    }

    return nil;
}

- (void)mapView:(MKMapView *)mapView didFailToLocateUserWithError:(NSError *)error
{
    NSLog(@"Map error: %@", error);
}


@end
