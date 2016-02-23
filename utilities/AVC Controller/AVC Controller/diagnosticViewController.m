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
#import "VectorPlotControl.h"
#import "Errors.h"

#include "types.h"
#include "system.h"
#include "clientAddress.h"

@interface diagnosticViewController (){
    CGPoint magSamplesXY[256];
    CGPoint headingSamplesXY[256];
    uint8_t magIndex;
}

@property PointPlotControl *headingPlotXY;
@property VectorPlotControl *vectorPlotXY;

@property (weak, nonatomic) IBOutlet UIActivityIndicatorView *connectingIndicator;
@property (weak, nonatomic) IBOutlet UIScrollView *scrollView;
@property (weak, nonatomic) IBOutlet UITableView *tableView;
@property dispatch_queue_t pollQueue;
@property BOOL shouldPoll;

@end

@implementation diagnosticViewController

system_t SYS;

NSString* DIAG_DATA_TITLES[] = {
    @"Measured position", @"Measured velocity", @"Measured heading",
    @"Estimated position", @"Estimated velocity", @"Estimated heading",
    @"GPS Fix"
};

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
    self.vectorPlotXY = [VectorPlotControl plotWithFrame:CGRectMake(0, 0, parentSize.width, parentSize.height)];
    self.headingPlotXY = [PointPlotControl plotWithFrame:CGRectMake(0, parentSize.height, parentSize.width, parentSize.height)];
    
    self.vectorPlotXY->points     = malloc(sizeof(CGPoint) * 2);
    self.vectorPlotXY->pointColor = malloc(sizeof(CGFloat*) * 2);
    self.vectorPlotXY->pointCount = 2;
    
    static CGFloat red[]  = { 1, 0, 0, 1 };
    static CGFloat blue[] = { 0, 0, 1, 1 };
    
    self.vectorPlotXY->pointColor[0] = red;
    self.vectorPlotXY->pointColor[1] = blue;
    
    [self.scrollView addSubview:self.vectorPlotXY];
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

        dispatch_async(dispatch_get_main_queue(), ^{
            [self.connectingIndicator startAnimating];
        });

        errno = 0;
        if((err = connect(sock, (const struct sockaddr*)&host, sizeof(host))) || errno){
            NSLog(@"Connection failed");
            dispatch_async(dispatch_get_main_queue(), ^{
                [Errors presentWithTitle:@"Could not connect to diagnostic server" onViewController:self];
                dispatch_async(dispatch_get_main_queue(), ^{
                    [self.connectingIndicator stopAnimating];
                });
            });
            return;
        }
        
        // connected!
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.connectingIndicator stopAnimating];
        });
        
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
                int bytes = read(sock, &SYS.body, sizeof(sensorStatef_t) + sizeof(sensorStatei_t));
            }
            
            headingSamplesXY[magIndex++] = CGPointMake(SYS.body.imu.adjReadings.mag.x, SYS.body.imu.adjReadings.mag.y);
            
            self.data.data = SYS.body;
            
//            float wdx = SYS.route.start->self.location.x - SYS.body.measured.position.x;
//            float wdy = SYS.route.start->self.location.y - SYS.body.measured.position.y;
//            float wmag = sqrtf(wdx * wdx + wdy * wdy);
//            
            self.vectorPlotXY->points[0] = CGPointMake(SYS.body.imu.adjReadings.mag.x, SYS.body.imu.adjReadings.mag.y);
//            self.vectorPlotXY->points[1] = CGPointMake(wdx / wmag, wdy / wmag);
            
            self.headingPlotXY->points = headingSamplesXY;
            if(self.headingPlotXY->pointCount < magIndex){
                self.headingPlotXY->pointCount = magIndex;
            }
            
            dispatch_async(dispatch_get_main_queue(), ^{
                [self.vectorPlotXY setNeedsDisplay];
                [self.headingPlotXY setNeedsDisplay];
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
    
    NSString* descriptions[] = {
        self.data.mesPosition, self.data.mesVelocity, self.data.mesHeading,
        self.data.estPosition, self.data.estVelocity, self.data.estHeading,
        self.data.hasGpsFix
    };
    
    cell.textLabel.text       = DIAG_DATA_TITLES[indexPath.row];
    cell.detailTextLabel.text = descriptions[indexPath.row];
    
    int r = 0xC1 - (indexPath.row % 2 ? 0xA : 0);
    int g = 0x31 - (indexPath.row % 2 ? 0xA : 0);
    int b = 0x1D - (indexPath.row % 2 ? 0xA : 0);
    cell.backgroundColor = cell.contentView.backgroundColor = [UIColor colorWithRed:r / 255.0f green:g / 255.0f blue:b / 255.0f alpha:1];
    
    return cell;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return sizeof(DIAG_DATA_TITLES) / sizeof(NSString*);
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
