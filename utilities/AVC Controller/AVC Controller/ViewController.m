//
//  ViewController.m
//  AVC Controller
//
//  Created by Kirk Roerig on 12/29/15.
//  Copyright © 2015 PossomGames. All rights reserved.
//

#import "ViewController.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

@interface ViewController ()

@property (weak, nonatomic) IBOutlet ThumbStickView *throttleStick;
@property (weak, nonatomic) IBOutlet ThumbStickView *steerStick;
@property (weak, nonatomic) IBOutlet UITextField *ipAddressText;

@end

typedef struct{
    uint32_t id;
    uint8_t  throttle;
    uint8_t  steering;
} rcMessage_t;

struct sockaddr_in HOST;
rcMessage_t STATE = { 0, 50, 50 };

@implementation ViewController

void transmit(){
    static int sock;

    
    if(!sock){
        sock = socket(AF_INET, SOCK_DGRAM, 0);
    }
    
    sendto(
        sock,
        &STATE,
        sizeof(rcMessage_t),
        0,
        (const struct sockaddr*)&HOST,
        sizeof(HOST)
    );
}

- (void)viewDidLayoutSubviews
{
    [self.throttleStick reset];
    [self.steerStick reset];
    
    self.throttleStick.xAxisDisabled = YES;
    self.steerStick.yAxisDisabled    = YES;
    
    [self.throttleStick setRangeForAxis:1 withMin:75 andMax:25];
    [self.steerStick setRangeForAxis:0 withMin:25 andMax:75];
    
    self.throttleStick.delegate = self;
    self.steerStick.delegate    = self;
    
    [self.view addGestureRecognizer:[[UITapGestureRecognizer alloc]
                                     initWithTarget:self
                                     action:@selector(dismiss)]];
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    
    
}

- (void)didThumbMoveWithValues:(CGPoint)values asSender:(ThumbStickView *)sender
{
    if(sender == self.throttleStick){
        STATE.throttle = values.y;
    }
    
    if(sender == self.steerStick){
        STATE.steering = values.x;
    }
    
    transmit();
}

- (IBAction)didChangeIpAddress:(id)sender {
    struct hostent* he = NULL;
    
    if(!(he = gethostbyname([self.ipAddressText.text UTF8String]))){
        return;
    }
    
    HOST.sin_family = AF_INET;
    HOST.sin_port   = htons(1338);
    memcpy((void*)&(HOST.sin_addr), he->h_addr_list[0], he->h_length);
}

- (void)dismiss {
    [self.ipAddressText resignFirstResponder];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
