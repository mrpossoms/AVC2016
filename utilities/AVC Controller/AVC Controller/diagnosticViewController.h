//
//  diagnosticViewController.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "SensorReadings.h"

@interface diagnosticViewController : UIViewController <UITableViewDataSource, UITableViewDelegate, UIScrollViewDelegate>

@property SensorReadings* data;

@end
