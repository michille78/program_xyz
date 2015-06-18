//
//  ViewController.h
//  demo_xcode
//
//  Created by 何元会 on 14/12/23.
//  Copyright (c) 2014年 Noitom. All rights reserved.
//

#import <Cocoa/Cocoa.h>

#include "NeuronDataReader.h"
#include "SocketCommand.h"

@interface ViewController : NSViewController

@property (weak) IBOutlet NSTextField *txtIP;
@property (weak) IBOutlet NSTextField *txtPort;
@property (weak) IBOutlet NSButton *cbWithDisp;
@property (weak) IBOutlet NSButton *cbWithPrefix;
@property (weak) IBOutlet NSTextField *txtLog;
@property (weak) IBOutlet NSButton *cbUdpMode;

- (IBAction)btnConnect:(id)sender;


void OnFrameDataReceived(void* customedObj, struct BvhOutputBinaryHeader* header, float* data);

void OnSocketConnectionStatus(void* customedObj, enum SocketConnectionStatusTypes status, char* message);

@end

