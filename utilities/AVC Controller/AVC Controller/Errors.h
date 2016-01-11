//
//  Errors.h
//  AVC Controller
//
//  Created by Kirk Roerig on 1/10/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface Errors : NSObject

+ (void)presentWithTitle:(NSString *)title onViewController:(UIViewController*)vc;

@end
