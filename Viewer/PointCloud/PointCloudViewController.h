//
//  PointCloudViewController.h
//  BodyFusion
//
//  Created by yanshi on 2019/4/3.
//  Copyright © 2019 yanshi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <SceneKit/SceneKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface PointCloudViewController : UIViewController

@property (weak, nonatomic) IBOutlet SCNView *scnView;

@end

NS_ASSUME_NONNULL_END
