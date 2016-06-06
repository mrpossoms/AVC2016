//
//  SensorReadings.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SensorReadings.h"

@implementation SensorReadings


- (NSString*)hasGpsFix
{
    return self.data.hasGpsFix ? @"YES" : @"NO";
}

@end
