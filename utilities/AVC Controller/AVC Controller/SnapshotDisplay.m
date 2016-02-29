//
//  SnapshotDisplay.m
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SnapshotDisplay.h"
#import "SnapshotAnnotationView.h"

@interface SnapshotDisplay()

@property CLLocationManager* locationManager;
@property SnapshotAnnotationView* annotaion;

@end

@implementation SnapshotDisplay

- (void)setSnapshot:(sysSnap_t)snapshot
{
    _snapshot = snapshot;

    [self focusOnLocation:[[CLLocation alloc] initWithLatitude:self.coordinate.latitude longitude:self.coordinate.longitude]];

    [self removeAnnotation:self];
    [self addAnnotation:self];
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
    [self setRegion:MKCoordinateRegionMakeWithDistance(location.coordinate, 10, 10) animated:YES];
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

- (void)mapViewWillStartLocatingUser:(MKMapView *)mapView
{
    [self focusOnLocation:mapView.userLocation.location];
}

- (void)mapView:(MKMapView *)mapView didUpdateUserLocation:(MKUserLocation *)userLocation
{
    [self focusOnLocation:userLocation.location];
    [self.locationManager stopUpdatingLocation];
}

- (nullable MKAnnotationView *)mapView:(MKMapView *)mapView viewForAnnotation:(id <MKAnnotation>)annotation
{
    SnapshotAnnotationView* annoView = [[SnapshotAnnotationView alloc] initWithAnnotation:annotation reuseIdentifier:@"RobitAnnotation"];
    annoView.bounds = CGRectMake(0, 0, 80, 80);

    return annoView;
}

- (MKOverlayRenderer*)mapView:(MKMapView *)mapView rendererForOverlay:(id<MKOverlay>)overlay
{
//    if(overlay == self.routeLine)
//    {
//        if(nil == self.routeLineRenderer)
//        {
//            self.routeLineRenderer = [[MKPolylineRenderer alloc] initWithPolyline:self.routeLine];
//            self.routeLineRenderer.fillColor = [UIColor redColor];
//            self.routeLineRenderer.strokeColor = [UIColor redColor];
//            self.routeLineRenderer.lineWidth = 2;
//
//        }
//
//        return self.routeLineRenderer;
//    }

    return nil;
}

- (void)mapView:(MKMapView *)mapView didFailToLocateUserWithError:(NSError *)error
{
    NSLog(@"Map error: %@", error);
}


@end
