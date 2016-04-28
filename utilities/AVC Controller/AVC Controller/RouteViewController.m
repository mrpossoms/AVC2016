//
//  RouteViewController.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/3/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

@import MapKit;
@import CoreLocation;

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

#import <AudioToolbox/AudioServices.h>
#import "clientAddress.h"
#import "RouteViewController.h"
#import "Waypoint.h"
#import "ViewController.h"

static Waypoint* WAYPOINT_START;

typedef enum{
    MODE_NONE = 0,
    MODE_RECORDING,
    MODE_MANUAL,
}recordMode_e;

@interface RouteViewController()

@property (weak, nonatomic) IBOutlet MKMapView *map;
@property (weak, nonatomic) IBOutlet UIButton *modeButton;

@property CLLocationManager* locationManager;
@property CLLocation*        lastLocation;
@property UILongPressGestureRecognizer* mapTap;
@property Waypoint* last;
@property (nonatomic) recordMode_e recordingMode;

@property (weak, nonatomic) IBOutlet UIProgressView *uploadProgress;
@property MKPolyline* routeLine;
@property MKPolylineRenderer *routeLineRenderer; //overlay view
@end

@implementation RouteViewController

- (void)setRecordingMode:(recordMode_e)recordingMode
{
    switch(recordingMode){
        case MODE_MANUAL:
            [self.map setScrollEnabled:YES];
            [self.modeButton setTitle:@"Clear" forState:UIControlStateNormal];
            break;
        case MODE_RECORDING:
            [self.map setScrollEnabled:NO];
            [self.modeButton setTitle:@"Clear" forState:UIControlStateNormal];
            break;
        default:
            [self.modeButton setTitle:@"Manual" forState:UIControlStateNormal];
            [self.map setScrollEnabled:NO];
    }
    
    _recordingMode = recordingMode;
}

- (void)viewDidLoad
{
    self.map.showsBuildings = NO;
    self.map.showsPointsOfInterest = NO;
    self.map.delegate = self;
    self.map.showsScale = YES;
    self.map.mapType = MKMapTypeSatellite;

    // don't allow the user to lose focus of their current position
    [self.map setScrollEnabled:NO];
    
    self.recordingMode = MODE_NONE;
    
    if([CLLocationManager locationServicesEnabled]){
        self.locationManager = [[CLLocationManager alloc] init];
        self.locationManager.activityType = CLActivityTypeOther;
        self.locationManager.delegate = self;
        
        [self.locationManager requestWhenInUseAuthorization];
    }
    
    if(!self.mapTap){
        self.mapTap = [[UILongPressGestureRecognizer alloc] initWithTarget:self action:@selector(didTapMap:)];
        [self.map addGestureRecognizer:self.mapTap];
    }
}

- (void)viewDidAppear:(BOOL)animated
{
    // we still have a route!
    if(WAYPOINT_START){
        
        // reset the last waypoint
        for(Waypoint* w = WAYPOINT_START; w; w = w.next){
            self.last = w;
        }
        
        [self updateRecordedRoute];
    }
}
- (IBAction)didModeToggle:(id)sender {
    switch (self.recordingMode) {
        case MODE_NONE:
            self.recordingMode = MODE_MANUAL;
            break;
        case MODE_RECORDING:
        case MODE_MANUAL:
            self.recordingMode = MODE_NONE;
            [self didClearRoute:nil];
            break;
    }
}

- (IBAction)didClearRoute:(id)sender {
    for(Waypoint* last = self.last; last;){
        Waypoint* lastLast = last.previous;
        [last remove];
        last = lastLast;
    }
    
    WAYPOINT_START = nil;
    
    self.recordingMode = MODE_NONE;
    [self updateRecordedRoute];
}

- (void)reduceRoute:(Waypoint*)waypoint
{
    NSLog(@"Old length %d", [waypoint length]);
    
    for(Waypoint* curr = waypoint; curr;){
        if([curr distanceTo:curr.next] < 1){
            [curr.next remove];
        }
        else{
            curr = curr.next;
        }
    }

    NSLog(@"New length %d", [waypoint length]);
}

- (IBAction)didRequestUpload:(id)sender {
    
    if(!HOST_ADDRESS){
        UIAlertController* alert = [UIAlertController alertControllerWithTitle:@"Error"
                                                                       message:@"No host specified, return to the control view to enter one"
                                                                preferredStyle:UIAlertControllerStyleAlert];
        [alert addAction:[UIAlertAction actionWithTitle:@"Go back"
                                                  style:UIAlertActionStyleCancel
                                                handler:^(UIAlertAction * _Nonnull action) {
                                                    [self dismissViewControllerAnimated:YES completion:^{ }];
                                                }]];
        [self presentViewController:alert animated:YES completion:^{ }];
        
        
        return;
    }

    [self reduceRoute:WAYPOINT_START];
    [self updateRecordedRoute];
    
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.uploadProgress setHidden:NO];
            [self.uploadProgress setProgress:0 animated:NO];
        });

        int sockfd;
        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)))
        {
            struct sockaddr_in addr = *HOST_ADDRESS;
            addr.sin_port = htons(1339);

            if(connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                UIAlertController* alert = [UIAlertController alertControllerWithTitle:@"Error"
                                                                               message:@"Failed to connect to host. Service or device might be down."
                                                                        preferredStyle:UIAlertControllerStyleAlert];
                [alert addAction:[UIAlertAction actionWithTitle:@"OK"
                                                          style:UIAlertActionStyleCancel
                                                        handler:^(UIAlertAction * _Nonnull action) { }]];
                [self presentViewController:alert animated:YES completion:^{ }];
                close(sockfd);
                
                dispatch_async(dispatch_get_main_queue(), ^{ self.uploadProgress.hidden = YES; });
                return;
            }
        
            // allocate structures to send over the network
            gpsRouteHeader_t header = {
                .waypoints = [WAYPOINT_START length]
            };
            
            gpsWaypoint_t* waypoints = malloc(header.waypoints * sizeof(gpsWaypoint_t));
            int i = 0;
            for(Waypoint* waypoint = WAYPOINT_START; waypoint; waypoint = waypoint.next, ++i){
                waypoints[i].location.x = waypoint.coordinate.longitude;
                waypoints[i].location.y = waypoint.coordinate.latitude;
                waypoints[i].tolerance = 4;
                waypoints[i].nextWaypoint = i + 1;
            }

            // send all the things
            size_t written = 0, allTheBytes = sizeof(header) + sizeof(gpsWaypoint_t) * header.waypoints;
            uint32_t action = 0; // zero indicates mission upload

            write(sockfd, &action, sizeof(action));

            written += write(sockfd, &header, sizeof(header));
            for(int i = 0; i < header.waypoints; ++i){
                written += write(sockfd, waypoints + i, sizeof(gpsWaypoint_t));
                dispatch_async(dispatch_get_main_queue(), ^{
                   [self.uploadProgress setProgress:written / (float)allTheBytes animated:YES];
                });
            }
            
            // clean up
            close(sockfd);
            free(waypoints);
            sleep(1);
            dispatch_async(dispatch_get_main_queue(), ^{ [self.uploadProgress setHidden:YES]; });
        }
        else{
            
        }
        
    });
}

- (void)didTapMap:(UILongPressGestureRecognizer*)recognizer
{
    if(recognizer.state == UIGestureRecognizerStateEnded){
        switch (self.recordingMode) {
            case MODE_NONE:
                self.recordingMode = MODE_RECORDING;
                self.map.tintColor = [UIColor greenColor];
                [self.locationManager startUpdatingLocation];
                break;
            case MODE_RECORDING:
                self.recordingMode = MODE_NONE;
                self.map.tintColor = [UIColor redColor];
                [self.locationManager stopUpdatingLocation];
                break;
            case MODE_MANUAL:
                [self addWaypointWithCoordinate:self.map.centerCoordinate];
                AudioServicesPlayAlertSound(kSystemSoundID_Vibrate);
                break;
        }
    }
}

- (IBAction)didDismiss:(id)sender {
    [self dismissViewControllerAnimated:YES completion:^{

    }];
}

#pragma mark - Location manager delegate

- (void)updateRecordedRoute
{
    static CLLocationCoordinate2D* coords;
    
    if(self.routeLine){
        self.routeLineRenderer = nil;
        [self.map removeOverlay:self.routeLine];
        free(coords);
    }
    
    int routeLen = [WAYPOINT_START length];
    coords = malloc(sizeof(CLLocationCoordinate2D) * routeLen);
    
    int ind = 0;
    for(Waypoint* curr = WAYPOINT_START; curr; curr = curr.next, ++ind){
        coords[ind] = curr.coordinate;
    }
    
    self.routeLine = [MKPolyline polylineWithCoordinates:coords count:routeLen];
    [self.map addOverlay:self.routeLine];
}

- (void)locationManager:(CLLocationManager *)manager didChangeAuthorizationStatus:(CLAuthorizationStatus)status
{
    if(status == kCLAuthorizationStatusAuthorizedWhenInUse){
        [self.map setUserTrackingMode:MKUserTrackingModeFollow animated:YES];
    }
    else{
        UIAlertView* locationNeeded = [[UIAlertView alloc] initWithTitle:@"Location Services Required"
                                                                 message:@"Do it dummy."
                                                                delegate:self
                                                       cancelButtonTitle:@"OK"
                                                       otherButtonTitles:nil];
        locationNeeded.tag = 0;
        [locationNeeded show];
    }
}

- (void)addWaypointWithCoordinate:(CLLocationCoordinate2D)coord
{
    if(!WAYPOINT_START){
        self.last = WAYPOINT_START = [[Waypoint alloc] initWithPosition:coord];
    }
    else{
        self.last = [self.last addNext:coord];
    }
    [self updateRecordedRoute];
}

- (void)focusOnLocation:(CLLocation*)location
{
    if(SHOULD_FOLLOW){
        int sockfd;
        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)))
        {
            struct sockaddr_in addr = *HOST_ADDRESS;
            addr.sin_port = htons(1339);

            if(connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                close(sockfd);

                dispatch_async(dispatch_get_main_queue(), ^{
                    UIAlertController* alert = [UIAlertController alertControllerWithTitle:@"Error"
                                                                                   message:@"Failed to connect to host. Service or device might be down."
                                                                            preferredStyle:UIAlertControllerStyleAlert];
                    [alert addAction:[UIAlertAction actionWithTitle:@"OK"
                                                              style:UIAlertActionStyleCancel
                                                            handler:^(UIAlertAction * _Nonnull action) { }]];
                    [self presentViewController:alert animated:YES completion:^{ }];
                });
                return;
            }

            uint32_t action = MISS_SRV_FOLLOW; // tell the daemon to start up the main AVC program
            gpsWaypoint_t waypoint = {};
            Waypoint* way = [[Waypoint alloc] initWithPosition:self.lastLocation.coordinate];

            waypoint.location.x = way.coordinate.longitude;
            waypoint.location.y = way.coordinate.latitude;
            waypoint.tolerance = 4;
            waypoint.nextWaypoint = 0;

            write(sockfd, &action, sizeof(action));
            write(sockfd, &waypoint, sizeof(waypoint));
            close(sockfd);
        }
    }

    if(self.recordingMode == MODE_MANUAL) return;
    
    [self.map setRegion:MKCoordinateRegionMakeWithDistance(location.coordinate, 10, 10) animated:YES];
}

- (void)locationManager:(CLLocationManager *)manager didUpdateLocations:(NSArray<CLLocation *> *)locations
{
    for(CLLocation* loc in locations){
        if(self.recordingMode == MODE_RECORDING){
            [self addWaypointWithCoordinate:loc.coordinate];
        }
        

        [self focusOnLocation:loc];
        self.lastLocation = loc;
    }
}

- (void)locationManager:(CLLocationManager *)manager didFailWithError:(NSError *)error
{
    NSLog(@"Error %@", error);
}

#pragma mark - Map view delegate

- (void)mapViewWillStartLocatingUser:(MKMapView *)mapView
{
    [self focusOnLocation:mapView.userLocation.location];
}

- (void)mapView:(MKMapView *)mapView didUpdateUserLocation:(MKUserLocation *)userLocation
{
    self.lastLocation = userLocation.location;

    [self focusOnLocation:userLocation.location];
}

- (MKOverlayRenderer*)mapView:(MKMapView *)mapView rendererForOverlay:(id<MKOverlay>)overlay
{
    if(overlay == self.routeLine)
    {
        if(nil == self.routeLineRenderer)
        {
            self.routeLineRenderer = [[MKPolylineRenderer alloc] initWithPolyline:self.routeLine];
            self.routeLineRenderer.fillColor = [UIColor redColor];
            self.routeLineRenderer.strokeColor = [UIColor redColor];
            self.routeLineRenderer.lineWidth = 2;
            
        }
        
        return self.routeLineRenderer;
    }
    
    return nil;
}

- (MKAnnotationView*)mapView:(MKMapView *)mapView viewForAnnotation:(id<MKAnnotation>)annotation
{
    
//    if([annotation isKindOfClass:[Settlement class]]){
//        return [(Settlement*)annotation annotationView];
//    }
//    
//    if([annotation isKindOfClass:[MKUserLocation class]]){
//        return [[MKAnnotationView alloc] initWithAnnotation:[[Merchant alloc] init] reuseIdentifier:@"PlayerAvatar"];
//    }
//    
//    
    return nil;
}

- (void)mapView:(MKMapView *)mapView didFailToLocateUserWithError:(NSError *)error
{
    NSLog(@"Map error: %@", error);
}

@end
