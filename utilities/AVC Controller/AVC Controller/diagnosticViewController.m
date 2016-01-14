//
//  diagnosticViewController.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#include <sys/socket.h>

#import "diagnosticViewController.h"
#import "PointPlotControl.h"
#import "Errors.h"

#include "system.h"
#include "clientAddress.h"

@interface diagnosticViewController (){
    CGPoint magSamplesXY[256];
    CGPoint headingSamplesXY[256];
    uint8_t magIndex;
}

@property PointPlotControl* magPlotXY, *headingPlotXY;

@property (weak, nonatomic) IBOutlet UIScrollView *scrollView;
@property (weak, nonatomic) IBOutlet UITableView *tableView;
@property dispatch_queue_t pollQueue;
@property BOOL shouldPoll;

@end

@implementation diagnosticViewController

system_t SYS;

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    SensorReadings* sen = [[SensorReadings alloc] init];
    sen.data = SYS.body;
    self.data = sen;
    
    self.scrollView.delegate = self;
    
    self.tableView.delegate   = self;
    self.tableView.dataSource = self;
    
    self.pollQueue = dispatch_queue_create("MESSAGE_POLL_QUEUE", NULL);
}

- (void)viewDidLayoutSubviews
{
    CGSize parentSize = self.scrollView.frame.size;
    
    self.scrollView.contentSize = CGSizeMake(parentSize.width, parentSize.height * 2);
    self.magPlotXY = [PointPlotControl plotWithFrame:CGRectMake(0, 0, parentSize.width, parentSize.height)];
    self.headingPlotXY = [PointPlotControl plotWithFrame:CGRectMake(0, parentSize.height, parentSize.width, parentSize.height)];
    [self.scrollView addSubview:self.magPlotXY];
    
    [self.scrollView addSubview:self.headingPlotXY];

}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (IBAction)didDismiss:(id)sender {
    [self dismissViewControllerAnimated:YES completion:^{
        // no-op
    }];
}

- (void)viewWillAppear:(BOOL)animated
{
    self.shouldPoll = YES;
}

- (void)viewDidAppear:(BOOL)animated
{
    struct sockaddr_in host = {};
    
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
    
    host = *HOST_ADDRESS;
    host.sin_port = htons(1340);

    dispatch_async(self.pollQueue, ^{
        CFAbsoluteTime last = CFAbsoluteTimeGetCurrent();
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        int err;

        if((err = connect(sock, (const struct sockaddr*)&host, sizeof(host)))){
            NSLog(@"Connection failed");
            dispatch_async(dispatch_get_main_queue(), ^{
                [Errors presentWithTitle:@"Could not connect to diagnostic server" onViewController:self];
            });
            return;
        }
        
        while(self.shouldPoll){
            CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
            
            if(now - last < 0.1){
                usleep(100000);
                continue;
            }
            NSLog(@"Polling");
            
            int written = write(sock, "X", 1);
            
            fd_set readfd;
            struct timeval timeout = { 1, 0 };
            FD_ZERO(&readfd);
            FD_SET(sock, &readfd);
            
            if(select(sock + 1, &readfd, NULL, NULL, &timeout) > 0){
                int bytes = read(sock, &SYS.body, sizeof(fusedObjState_t));
            }
            
            magSamplesXY[magIndex] = CGPointMake(SYS.body.imu.rawReadings.mag.x, SYS.body.imu.rawReadings.mag.y);
            headingSamplesXY[magIndex++] = CGPointMake(SYS.body.measured.heading.x, SYS.body.measured.heading.y);
            
            self.data.data = SYS.body;
            self.magPlotXY->points = magSamplesXY;
            if(self.magPlotXY->pointCount < magIndex){
                self.magPlotXY->pointCount = magIndex;
            }
            
            self.headingPlotXY->points = headingSamplesXY;
            if(self.headingPlotXY->pointCount < magIndex){
                self.headingPlotXY->pointCount = magIndex;
            }
            
            dispatch_async(dispatch_get_main_queue(), ^{
                [self.magPlotXY setNeedsDisplay];
                [self.tableView reloadData];
            });
            
            last = now;
        }
        
        close(sock);
    });
}

- (void)viewWillDisappear:(BOOL)animated
{
    self.shouldPoll = NO;
}

- (UITableViewCell*)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    UITableViewCell* cell = [tableView dequeueReusableCellWithIdentifier:@"DIAG-CELL"];
    
    NSString* titles[] = {
        @"Measured position", @"Measured velocity", @"Measured heading",
        @"Estimated position", @"Estimated velocity", @"Estimated heading",
    };
    
    NSString* descriptions[] = {
        self.data.mesPosition, self.data.mesVelocity, self.data.mesHeading,
        self.data.estPosition, self.data.estVelocity, self.data.estHeading,
    };
    
    cell.textLabel.text       = titles[indexPath.row];
    cell.detailTextLabel.text = descriptions[indexPath.row];
    
    int r = 0xC1 - (indexPath.row % 2 ? 0xA : 0);
    int g = 0x31 - (indexPath.row % 2 ? 0xA : 0);
    int b = 0x1D - (indexPath.row % 2 ? 0xA : 0);
    cell.backgroundColor = cell.contentView.backgroundColor = [UIColor colorWithRed:r / 255.0f green:g / 255.0f blue:b / 255.0f alpha:1];
    
    return cell;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return 6;
}

/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/

@end
