//
//  SensorReadings.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import "SensorReadings.h"

@implementation SensorReadings

- (NSString*)stringForVecF:(vec3f_t)v
{
    return [NSString stringWithFormat:@"{ %0.3f, %0.3f, %0.3f }", v.x, v.y, v.z];
}

- (NSString*)stringForVecI:(vec3i16_t)v
{
    return [NSString stringWithFormat:@"{ %d, %d, %d }", v.x, v.y, v.z];
}

- (NSString*)mesPosition
{
    return [self stringForVecF:self.data.measured.position];
}

- (NSString*)mesVelocity
{
    return [self stringForVecF:self.data.measured.velocity];
}

- (NSString*)mesHeading
{
    return [self stringForVecF:self.data.measured.heading];
}

- (NSString*)estPosition
{
    return [self stringForVecF:self.data.estimated.position];
}

- (NSString*)estVelocity
{
    return [self stringForVecF:self.data.estimated.velocity];
}

- (NSString*)estHeading
{
    return [self stringForVecF:self.data.estimated.heading];
}

@end
