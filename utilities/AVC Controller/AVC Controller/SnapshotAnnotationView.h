//
//  SnapshotAnnotationView.h
//  AVC Controller
//
//  Created by Kirk Roerig on 2/28/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <MapKit/MapKit.h>
#include "system.h"

@interface SnapshotAnnotationView : MKAnnotationView

@property (nonatomic) sysSnap_t snapshot;

@end
