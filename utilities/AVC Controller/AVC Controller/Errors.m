//
//  Errors.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/10/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Errors.h"

@implementation Errors

+ (void)presentWithTitle:(NSString *)title onViewController:(UIViewController*)vc
{
    UIAlertController* alert = [UIAlertController alertControllerWithTitle:@"Error"
                                                                   message:title
                                                            preferredStyle:UIAlertControllerStyleAlert];
    [alert addAction:[UIAlertAction actionWithTitle:@"OK"
                                              style:UIAlertActionStyleCancel
                                            handler:^(UIAlertAction * _Nonnull action) { }]];
    [vc presentViewController:alert animated:YES completion:^{ }];
}

@end
