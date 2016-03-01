//
//  WaypointsOverlay.h
//  AVC Controller
//
//  Created by Kirk Roerig on 2/29/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <MapKit/MapKit.h>
#include "system.h"

@interface WaypointsOverlay : NSObject <MKAnnotation>

@property vec3f_t location;

+ (instancetype)overlayAt:(vec3f_t)location;

@end
