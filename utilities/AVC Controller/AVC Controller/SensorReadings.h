//
//  SensorReadings.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "base/system.h"
#include "base/types.h"

@interface SensorReadings : NSObject

@property sysSnap_t data;

@property (readonly) NSString *estPosition, *estVelocity, *estHeading;
@property (readonly) NSString *mesPosition, *mesVelocity, *mesHeading;
@property (readonly) NSString *hasGpsFix;

@end
