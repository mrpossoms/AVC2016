//
//  diagnosticViewController.m
//  AVC Controller
//
//  Created by Kirk Roerig on 1/9/16.
//  Copyright © 2016 PossomGames. All rights reserved.
//

#include <sys/socket.h>

#import "diagnosticViewController.h"
#import "Errors.h"

#include "system.h"
#include "clientAddress.h"

@interface diagnosticViewController ()

@property (weak, nonatomic) IBOutlet UITableView *tableView;
@property dispatch_queue_t pollQueue;

@end

@implementation diagnosticViewController

system_t SYS;
int SOCK;

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    SensorReadings* sen = [[SensorReadings alloc] init];
    sen.data = SYS.body;
    self.data = sen;
    
    self.tableView.delegate   = self;
    self.tableView.dataSource = self;
    
    self.pollQueue = dispatch_queue_create("MESSAGE_POLL_QUEUE", NULL);
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

- (void)viewDidAppear:(BOOL)animated
{
    int err = 0;
    struct sockaddr_in host = {};
    
    if(SOCK) close(SOCK);
    
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
    
    SOCK = socket(AF_INET, SOCK_STREAM, 0);
    if((err = connect(SOCK, (const struct sockaddr*)&host, sizeof(host)))){
        [Errors presentWithTitle:@"Could not connect to diagnostic server" onViewController:self];
        return;
    }
    
    dispatch_async(self.pollQueue, ^{
        while(1){
            write(SOCK, ".", 1);
            read(SOCK, &SYS.body, sizeof(fusedObjState_t));
            
            self.data.data = SYS.body;
            [self.tableView reloadData];
            sleep(1);
        }
    });
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
