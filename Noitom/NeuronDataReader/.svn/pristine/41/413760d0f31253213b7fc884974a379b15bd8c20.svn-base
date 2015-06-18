//
//  ViewController.m
//  demo_xcode
//
//  Created by 何元会 on 14/12/23.
//  Copyright (c) 2014年 Noitom. All rights reserved.
//

#import "ViewController.h"


@implementation ViewController

@synthesize txtLog = _txtLog;

void CALLBACK _OnFrameDataReceived(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{
   printf("aaa\n");
}

void CALLBACK _OnSocketConnectionStatus(void* customedObj, SOCKET_REF sender, SocketStatus status, char* message)
{
    //[_txtLog setStringValue: message];
}


- (void)viewDidLoad {
    [super viewDidLoad];

    // Do any additional setup after loading the view.
    
    //
    BRRegisterSocketStatusCallback(NULL, _OnSocketConnectionStatus);
    
    //
    BRRegisterFrameDataCallback(NULL, _OnFrameDataReceived);
}

- (void)setRepresentedObject:(id)representedObject {
    [super setRepresentedObject:representedObject];

    // Update the view, if already loaded.
}

SOCKET_REF sockPtr = 0;

- (IBAction)btnConnect:(id)sender {
    const char* strIp = [[_txtIP stringValue] cStringUsingEncoding:NSUTF8StringEncoding];
    long nport = [[_txtPort stringValue] integerValue];
    if (_cbUdpMode.isAccessibilityEnabled == 0)
    {
        if (sockPtr) {
            BRCloseSocket(sockPtr);
        }
        sockPtr = BRConnectTo((char* )strIp, (int)nport);
        printf("socket client address is %d\n", (uint)sockPtr);
    }
    else
    {
        if (sockPtr) {
            BRCloseSocket(sockPtr);
        }
        // Create a UDP service to receive data at local 'nPort'
        SOCKET_REF udpSockPtr = BRStartUDPServiceAt((int)nport);
        printf("udp connected.\n");

    }
}
@end
