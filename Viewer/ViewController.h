//
//  ViewController.h
//  InfiniTAM
//
//  Created by yanshi on 24/01/2019.
//  Copyright (c) 2019 yanshi. All rights reserved.
//

#import <UIKit/UIKit.h>
#define HAS_LIBCXX
#import <AVFoundation/AVFoundation.h>
#import <CoreMotion/CoreMotion.h>

@interface ViewController : UIViewController <AVCaptureDepthDataOutputDelegate>

@property (weak, nonatomic) IBOutlet UIView *renderView;
@property (weak, nonatomic) IBOutlet UITextField *tbOut;
@property (weak, nonatomic) IBOutlet UIButton *bProcessOne;
@property (weak, nonatomic) IBOutlet UIButton *bProcessCont;

- (IBAction)bProcessOne_clicked:(id)sender;
- (IBAction)bProcessCont_clicked:(id)sender;

@end
