//
//  diagnosticViewController.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#include <sys/ioctl.h>
#include <sys/socket.h>

#import "diagnosticViewController.h"
#import "PointPlotControl.h"
#import "VectorPlotControl.h"
#import "Errors.h"
#import "SnapshotMapView.h"

#include "types.h"
#include "system.h"
#include "clientAddress.h"

@interface diagnosticViewController (){
    CGPoint magSamplesXY[256];
    CGPoint headingSamplesXY[256];
    uint8_t magIndex;

    blkboxLog_t* logs;
    uint32_t logCount;

    sysSnap_t* blackbox;
    uint32_t blackboxSamples;
}

@property (weak, nonatomic) IBOutlet UILabel *gpCoordLabel;
@property (weak, nonatomic) IBOutlet UILabel *stateNumberLabel;
@property (weak, nonatomic) IBOutlet UISlider *scrubber;
@property (weak, nonatomic) IBOutlet SnapshotMapView *snapshotDisplay;
@property (weak, nonatomic) IBOutlet UITableView *blackboxTable;
@property (weak, nonatomic) IBOutlet UIActivityIndicatorView *connectingIndicator;
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
//    sen.data = SYS.body;
    self.data = sen;

    self.blackboxTable.dataSource = self;
    self.blackboxTable.delegate   = self;

    self.pollQueue = dispatch_queue_create("MESSAGE_POLL_QUEUE", NULL);
}

- (void)viewDidLayoutSubviews
{

    static const char* labels[] = {
        "filtered_mag",
        "est_heading",
        "goal"
    };
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (IBAction)didDismiss:(id)sender {
    self.shouldPoll = NO;

    dispatch_after(DISPATCH_TIME_NOW + NSEC_PER_SEC * 1, dispatch_get_main_queue(), ^{
        [self dismissViewControllerAnimated:YES completion:^{
            // no-op
        }];
    });
}

- (void)viewWillAppear:(BOOL)animated
{
    self.shouldPoll = YES;
}

- (void)viewDidAppear:(BOOL)animated
{
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

    [self.connectingIndicator startAnimating];

    dispatch_async(self.pollQueue, ^{
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        int err;

        struct sockaddr_in host = {};
        host = *HOST_ADDRESS;
        host.sin_port = htons(1339);

        errno = 0;
        if((err = connect(sock, (const struct sockaddr*)&host, sizeof(host))) || errno){
            NSLog(@"Connection failed");
            dispatch_async(dispatch_get_main_queue(), ^{
                [Errors presentWithTitle:@"Could not connect to mission server" onViewController:self];
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

        uint32_t count = 0, action = MISS_SRV_BLKBOX_LIST;
        logCount = 0;

        free(logs);

        write(sock, &action, sizeof(action));
        read(sock, &count, sizeof(count));

        logs = (blkboxLog_t*)calloc(1, sizeof(blkboxLog_t) * count);

        for(int i = 0; i < count; ++i){
            read(sock, logs + i, sizeof(blkboxLog_t));
            ++logCount;
            dispatch_async(dispatch_get_main_queue(), ^{
                [self.blackboxTable reloadData];
            });
        }
        close(sock);
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.blackboxTable reloadData];
        });
    });

    [self startPolling];
}

- (void)viewWillDisappear:(BOOL)animated
{
    self.shouldPoll = NO;
}

- (void)startPolling
{

    dispatch_async(self.pollQueue, ^{
        CFAbsoluteTime last = CFAbsoluteTimeGetCurrent();
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        int err;

        struct sockaddr_in host = {};
        host = *HOST_ADDRESS;
        host.sin_port = htons(1340);

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

            NSLog(@"Polling");


            if(now - last < 0.1){
                usleep(100000);
                continue;
            }

            ssize_t bytes = 0;
            sysSnap_t snapShot = {};

            write(sock, "X", 1);

            while(bytes < sizeof(sysSnap_t)){
                ioctl(sock, FIONREAD, &bytes);
                usleep(10000);
            }

            bytes = read(sock, &snapShot, sizeof(sysSnap_t));

            self.snapshotDisplay.snapshot = snapShot;

            dispatch_async(dispatch_get_main_queue(), ^{
                self.gpCoordLabel.text = [NSString stringWithFormat:@"%f lat, %f lon", self.snapshotDisplay.coordinate.latitude, self.snapshotDisplay.coordinate.longitude];
            });


            last = now;
        }
        
        close(sock);
    });
    

}

- (UITableViewCell*)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    if(tableView == self.blackboxTable){
        UITableViewCell* cell = [tableView dequeueReusableCellWithIdentifier:@"BBOX-CELL"];
        if(!cell){
            cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle
                                          reuseIdentifier:@"BBOX-CELL"];
        }

        cell.textLabel.text = [NSString stringWithUTF8String:(char*)logs[indexPath.row].name];
        cell.detailTextLabel.text = [NSString stringWithFormat:@"%0.1f KB", logs[indexPath.row].bytes / 1024.0f];

        return cell;
    }

    return nil;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return logCount;
}

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
    if(self.connectingIndicator.isAnimating) return;

    dispatch_async(dispatch_get_main_queue(), ^{
        [self.connectingIndicator startAnimating];
        [self.scrubber setValue:0 animated:NO];
    });

    dispatch_async(self.pollQueue, ^{
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        int err;

        struct sockaddr_in host = {};
        host = *HOST_ADDRESS;
        host.sin_port = htons(1339);

        errno = 0;
        if((err = connect(sock, (const struct sockaddr*)&host, sizeof(host))) || errno){
            NSLog(@"Connection failed");
            dispatch_async(dispatch_get_main_queue(), ^{
                [Errors presentWithTitle:@"Could not connect to mission server" onViewController:self];
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

        uint32_t action = MISS_SRV_BLKBOX_DOWNLOAD;

        write(sock, &action, sizeof(action));
        write(sock, logs[indexPath.row].name, sizeof(logs[0].name));

        read(sock, &blackboxSamples, sizeof(blackboxSamples));

        free(blackbox);
        blackbox = (sysSnap_t*)calloc(blackboxSamples + 1, sizeof(sysSnap_t));

        for(int i = 0; i < blackboxSamples; ++i){
            size_t bytes = 0;
            while(bytes < sizeof(sysSnap_t)){
                ioctl(sock, FIONREAD, &bytes);
                usleep(10000);
            }

            bytes = read(sock, blackbox + i, sizeof(sysSnap_t));
            assert(bytes == sizeof(sysSnap_t));
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            [self.connectingIndicator stopAnimating];
            self.snapshotDisplay.snapshot = blackbox[1];
            self.gpCoordLabel.text = [NSString stringWithFormat:@"%f lat, %f lon", self.snapshotDisplay.coordinate.latitude, self.snapshotDisplay.coordinate.longitude];
        });

        close(sock);
    });

//    [UIView animateWithDuration:10 animations:^{
//        [self.scrubber setValue:0.999];
//    }];

}

- (IBAction)didScrub:(id)sender {
    if(blackboxSamples <= 0) return;

    UISlider* slider = sender;
    NSUInteger index = (NSUInteger)(blackboxSamples * slider.value);
    self.snapshotDisplay.snapshot = blackbox[index];
    self.gpCoordLabel.text = [NSString stringWithFormat:@"%f lat, %f lon", self.snapshotDisplay.coordinate.latitude, self.snapshotDisplay.coordinate.longitude];

    self.stateNumberLabel.text = [NSString stringWithFormat:@"%lu / %d", (unsigned long)index, blackboxSamples];
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
