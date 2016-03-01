//
//  SnapshotDisplay.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SnapshotDisplay.h"
#import "SnapshotAnnotationView.h"
#import "WaypointsOverlay.h"

@interface SnapshotDisplay()

@property CLLocationManager* locationManager;
@property SnapshotAnnotationView* annotaion;
@property vec3f_t lastPosition;

@end

@implementation SnapshotDisplay

- (void)setSnapshot:(sysSnap_t)snapshot
{
    vec3f_t h;

    if(vec3Dist(_lastPosition, _snapshot.estimated.position) >= 9){
        vec3Lerp(_lastPosition, _lastPosition, _snapshot.estimated.position, 0.25);
    }

    _snapshot = snapshot;

    vec3Sub(h, snapshot.estimated.position, _lastPosition);

    _snapshot.estimated.headingAngle = atan2f(h.y, h.x);
    
    [self focusOnLocation:[[CLLocation alloc] initWithLatitude:self.coordinate.latitude longitude:self.coordinate.longitude]];

    while(self.annotations.count){
        [self removeAnnotation:self.annotations.firstObject];
    }

    [self addAnnotation:self];
    [self addAnnotation:[WaypointsOverlay overlayAt:_snapshot.currentWaypoint.location]];
    [self addAnnotation:[WaypointsOverlay overlayAt:_snapshot.nextWaypoint.location]];
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

    [self addAnnotation:self];

    if([CLLocationManager locationServicesEnabled]){
        self.locationManager = [[CLLocationManager alloc] init];
        self.locationManager.activityType = CLActivityTypeOther;
        self.locationManager.delegate = self;

        [self.locationManager requestWhenInUseAuthorization];
        [self.locationManager startUpdatingLocation];
    }

    return self;
}


- (void)focusOnLocation:(CLLocation*)location
{
    if(location.coordinate.longitude > -180 && location.coordinate.longitude < 180 &&
       location.coordinate.latitude > -90 && location.coordinate.latitude < 90){
        [self setRegion:MKCoordinateRegionMakeWithDistance(location.coordinate, 10, 10) animated:YES];
//        [self setCenterCoordinate:location.coordinate animated:NO];
    }
}

#pragma mark - MKAnnotation protocol

- (NSString*)title
{
    return @"Das Robit";
}

- (CLLocationCoordinate2D) coordinate
{
    const float dia = 6371000 * 2; // diameter of the earth (meters)

    return CLLocationCoordinate2DMake(
                                      (_snapshot.estimated.position.y / dia) / (M_PI / 180.0f),
                                      (_snapshot.estimated.position.x / dia) / (M_PI / 180.0f)
                                      );
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
        SnapshotAnnotationView* annoView = (SnapshotAnnotationView*)[mapView dequeueReusableAnnotationViewWithIdentifier:@"RobitAnnotation"];

        if(!annoView){
            annoView = [[SnapshotAnnotationView alloc] initWithAnnotation:annotation reuseIdentifier:@"RobitAnnotation"];
        }
        annoView.bounds = CGRectMake(0, 0, 160, 160);
        annoView.snapshot = _snapshot;

        [annoView setNeedsDisplay];

        return annoView;
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
