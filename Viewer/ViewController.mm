//
//  ViewController.m
//  InfiniTAM
//
//  Created by yanshi on 23/01/2019.
//  Copyright (c) 2019 yanshi. All rights reserved.
//

#import "ViewController.h"

#include "Engine/ImageSourceEngine.h"
#include "Engine/IMUSourceEngine.h"

#include "ITMLib/ITMLib.h"
#include "ORUtils/MetalContext.h"

using namespace InfiniTAM::Engine;

static void*  SessionRunningContext = &SessionRunningContext;

typedef NS_ENUM(NSInteger, SetupResult) {
    SetupResultSuccess,
    SetupResultCameraNotAuthorized,
    SetupResultDepthCameraNotFound,
    SetupResultSessionConfigurationFailed
};

@interface ViewController()

@property (nonatomic, strong) dispatch_queue_t captureQueue;
@property (nonatomic, strong) dispatch_queue_t renderingQueue;
@property (nonatomic, strong) MetalContext *context;
@property (nonatomic, strong) CMMotionManager *motionManager;
@property (nonatomic) SetupResult setupResult;

@end

@implementation ViewController
{
    // MARK: - Properties
    
    CGColorSpaceRef rgbSpace;
    Vector2i imageSize;
    Vector2f scaleFactor;
    ITMUChar4Image *result;
    
    ImageSourceEngine *imageSource;
    IMUSourceEngine *imuSource;
    ITMLibSettings *internalSettings;
    ITMMainEngine *mainEngine;
    
    ITMIMUMeasurement *imuMeasurement;

    // For iPhone front depth, ITMShortImage short type is half/float16 actually.
    ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    
    AVCaptureSession* session;
    AVCaptureDepthDataOutput *depthOutput;

    bool calibrated;
    bool setupDone;
    bool fullProcess;
    bool isRecording;
    bool usingSensor;
    
    int currentFrameNo;
    int numFusionButtonClicked;
    
    NSTimeInterval totalProcessingTime;
    int totalProcessedFrames;
    
    char documentsPath[1000], *docsPath;
}

// MARK: - View Controller Life Cycle

- (void) viewDidLoad
{
    [super viewDidLoad];
    
    // Disable UI. The UI is enabled if and only if the session starts running.
    self.tbOut.enabled = NO;
    
    // Create the motionManager.
    _motionManager = [[CMMotionManager alloc]init];
    _motionManager.deviceMotionUpdateInterval = 1.0f / 60.0f;
    
    // Create the AVCaptureSession.
    session = [[AVCaptureSession alloc] init];
    
    // Create queue, communicate with the session and on captureQueue.
    self.captureQueue = dispatch_queue_create("capture", DISPATCH_QUEUE_SERIAL);
    self.renderingQueue = dispatch_queue_create("rendering", DISPATCH_QUEUE_SERIAL);
    
    self.setupResult = SetupResultSuccess;
    imageSize = Vector2i(320, 180);
    
    /*
     Check video authorization status. Video access is required and audio
     access is optional. If audio access is denied, audio is not recorded
     during movie recording.
     */
    switch ([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo])
    {
        case AVAuthorizationStatusAuthorized:
        {
            // The user has previously granted access to the camera.
            break;
        }
        case AVAuthorizationStatusNotDetermined:
        {
            /*
             The user has not yet been presented with the option to grant
             video access. We suspend the session queue to delay session
             setup until the access request has completed.
             
             Note that audio access will be implicitly requested when we
             create an AVCaptureDeviceInput for audio during session setup.
             */
            dispatch_suspend(self.captureQueue);
            [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
                if (!granted) {
                    self.setupResult = SetupResultCameraNotAuthorized;
                }
                dispatch_resume(self.captureQueue);
            }];
            break;
        }
        default:
        {
            // The user has previously denied access.
            self.setupResult = SetupResultCameraNotAuthorized;
            break;
        }
    }
    
    /*
     Setup the capture session.
     In general, it is not safe to mutate an AVCaptureSession or any of its
     inputs, outputs, or connections from multiple threads at the same time.
     
     Don't perform these tasks on the main queue because
     AVCaptureSession.startRunning() is a blocking call, which can
     take a long time. We dispatch session setup to the sessionQueue, so
     that the main queue isn't blocked, which keeps the UI responsive.
     */
    dispatch_async(self.captureQueue, ^{
        [self configureSession];
    });
    
    // Record time and frames.
    totalProcessingTime = 0;
    totalProcessedFrames = 0;
    
    // Setup main engine.
    [self setupEngine];
}

- (void) viewWillAppear:(BOOL)animated
{
    [self.navigationController setNavigationBarHidden:YES];
    [super viewWillAppear:animated];
    
    NSProcessInfoThermalState initialThermalState = [[NSProcessInfo processInfo] thermalState];
    if (initialThermalState == NSProcessInfoThermalStateSerious || initialThermalState == NSProcessInfoThermalStateCritical) {
        [self showThermalState:initialThermalState];
    }

    dispatch_async(self.captureQueue, ^{
        switch (self.setupResult)
        {
            case SetupResultSuccess:
            {
                // Only start the session running if setup succeeded.
                [session startRunning];
                break;
            }
            case SetupResultCameraNotAuthorized:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"App doesn't have permission to use the camera, please change privacy settings", @"Alert message when the user has denied access to the camera");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"Front Depth Camera" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    // Provide quick access to Settings.
                    UIAlertAction* settingsAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"Settings", @"Alert button to open Settings") style:UIAlertActionStyleDefault handler:^(UIAlertAction* action) {
                        [[UIApplication sharedApplication] openURL:[NSURL URLWithString:UIApplicationOpenSettingsURLString] options:@{} completionHandler:nil];
                    }];
                    [alertController addAction:settingsAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
            case SetupResultDepthCameraNotFound:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"Cannot find front depth camera, requires iPhoneX, XS, XS Max or XR.", @"Alert message when TrueDepth camera not found");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"TrueDepth Camera" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
            case SetupResultSessionConfigurationFailed:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"Unable to capture media", @"Alert message when something goes wrong during capture session configuration");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"Session Configuration" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
        }
    });
}

- (void) viewWillDisappear:(BOOL)animated
{
    dispatch_async(self.captureQueue, ^{
        if (self.setupResult == SetupResultSuccess) {
            [session stopRunning];
        }
    });
    
    [self.navigationController setNavigationBarHidden:NO];
    [super viewDidDisappear:animated];
}

- (void) didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

// You can use this opportunity to take corrective action to help cool the system down.
-(void) thermalStateChanged:(NSNotification*) notification
{
    NSProcessInfo* processInfo = notification.object;
    [self showThermalState: [processInfo thermalState]];
}

-(void) showThermalState:(NSProcessInfoThermalState) state
{
    dispatch_async(dispatch_get_main_queue(), ^{
        NSString* thermalStateString=@"UNKNOWN";
        if (state == NSProcessInfoThermalStateNominal) {
            thermalStateString = @"NOMINAL";
        } else if (state == NSProcessInfoThermalStateFair) {
            thermalStateString = @"FAIR";
        } else if (state == NSProcessInfoThermalStateSerious) {
            thermalStateString = @"SERIOUS";
        } else if (state == NSProcessInfoThermalStateCritical) {
            thermalStateString = @"CRITICAL";
        }
        NSString* theMessage = [NSString stringWithFormat:@"Thermal state: %@", thermalStateString];
        NSString* message = NSLocalizedString(theMessage, @"Alert message when thermal state has changed");
        UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"BodyFusion" message:message preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
        [alertController addAction:cancelAction];
        [self presentViewController:alertController animated:YES completion:nil];
    });
}

// MARK: -  KVO and Notifications

- (void) addObservers
{
    [session addObserver:self forKeyPath:@"running" options:NSKeyValueObservingOptionNew context:SessionRunningContext];

    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(thermalStateChanged:) name:NSProcessInfoThermalStateDidChangeNotification object:nil];
}

- (void) removeObservers
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    
    [session removeObserver:self forKeyPath:@"running" context:SessionRunningContext];
}

- (void) observeValueForKeyPath:(NSString*)keyPath
                       ofObject:(id)object
                         change:(NSDictionary*)change
                        context:(void*)context
{
    if (context != SessionRunningContext) {
        [super observeValueForKeyPath:keyPath ofObject:object change:change context:context];
    }
}

// MARK: - Session Management

// Call this on the captureQueue.
- (void) configureSession
{
    if (self.setupResult != SetupResultSuccess) {
        return;
    }
    
    NSError* error = nil;
    
    [session beginConfiguration];
    //    [session setSessionPreset:AVCaptureSessionPreset640x480];  // uncomment to set 4:3 depth, otherwise 16:9
    
    // Create device or device dicovery session.
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInTrueDepthCamera
                                                                 mediaType:AVMediaTypeDepthData
                                                                  position:AVCaptureDevicePositionFront];
    if (!device){
        NSLog(@"Could not find any depth device");
        self.setupResult = SetupResultDepthCameraNotFound;
        return;
    }
    
    // Set depth input
    AVCaptureDeviceInput *deviceInput = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!deviceInput) {
        NSLog(@"Could not create depth device input: %@", error);
        self.setupResult = SetupResultSessionConfigurationFailed;
        [session commitConfiguration];
        return;
    }
    if ([session canAddInput:deviceInput]) {
        [session addInput:deviceInput];
    } else {
        NSLog(@"Could not add depth device input to the session");
        self.setupResult = SetupResultSessionConfigurationFailed;
        [session commitConfiguration];
        return;
    }
    
    // Add depth output
    depthOutput = [[AVCaptureDepthDataOutput alloc] init];
    if ([session canAddOutput:depthOutput]) {
        [session addOutput:depthOutput];
        // Output to capture queue for depth
        [depthOutput setDelegate:self callbackQueue:self.captureQueue];
        //    depthOutput.alwaysDiscardsLateDepthData = NO;  // deliver unprocessed old data as soon as possible.
        depthOutput.filteringEnabled = NO;
    } else {
        NSLog(@"Could not add depth output to the session");
        self.setupResult = SetupResultSessionConfigurationFailed;
        [session commitConfiguration];
        return;
    }
    
    // Select output depth video format (resolution).
    NSArray* formats = device.activeFormat.supportedDepthDataFormats;
    AVCaptureDeviceFormat* selectedFormat = formats[6]; // kCVPixelFormatType_DepthFloat16: 'dpth'/'hdep'  320x180, { 2- 30 fps}, HRSI: 640x 360, fov:67.564
    [device lockForConfiguration:NULL];
    device.activeDepthDataFormat = selectedFormat;
    [device unlockForConfiguration];
    NSLog(@"from front depth camera");
    
    [session commitConfiguration];
}

- (void) setupEngine
{
    calibrated = false;
    setupDone = false;
    fullProcess = false;
    isRecording = false;
    
    currentFrameNo = 0;
    numFusionButtonClicked = 0;
    
    self.context = [MetalContext instance];
    
    NSArray* dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    docsPath = (char*)[[dirPaths objectAtIndex:0]cStringUsingEncoding:[NSString defaultCStringEncoding]];
    memcpy(documentsPath, docsPath, strlen(docsPath));
    
    NSError* error;
    NSString* dataPath = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"/Output"];
    if (![[NSFileManager defaultManager] fileExistsAtPath:dataPath])
        [[NSFileManager defaultManager] createDirectoryAtPath:dataPath withIntermediateDirectories:NO attributes:nil error:&error];

    if (self.setupResult == SetupResultSuccess)
    {
        fullProcess = false;
        [self.tbOut setText:@"front depth"];
        
        [_motionManager startDeviceMotionUpdates];
        
        imuMeasurement = new ITMIMUMeasurement();
        //        const char *calibFile = [[[NSBundle mainBundle]pathForResource:@"calib" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        //        imageSource = new CalibSource(calibFile, Vector2i(320, 240), 0.5f);
        imageSource = new iPhoneSource(imageSize);
        
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        
        usingSensor = true;
    }
    else
    {
        // Offline Process
        char calibFile[2000];
        sprintf(calibFile, "%s/Teddy/calib.txt", documentsPath);
        
        fullProcess = true;
        
        char imageSource_part1[2000], imageSource_part2[2000], imageSource_part3[2000];
        sprintf(imageSource_part1, "%s/CAsmall/Frames/img_%%08d.ppm", documentsPath);
        sprintf(imageSource_part2, "%s/CAsmall/Frames/img_%%08d.irw", documentsPath);
        sprintf(imageSource_part3, "%s/CAsmall/Frames/imu_%%08d.txt", documentsPath);
        
        // TODO: deallocate somewhere
        imageSource = new RawFileReader(calibFile, imageSource_part1, imageSource_part2, Vector2i(320, 240), 0.5f);
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        imuSource = new IMUSourceEngine(imageSource_part3);
        
        [self.tbOut setText:@"from file"];
        
        usingSensor = false;
        imuMeasurement = new ITMIMUMeasurement();
    }
    
    imageSize = imageSource->getDepthImageSize();
    result = new ITMUChar4Image(imageSize, false);
    rgbSpace = CGColorSpaceCreateDeviceRGB();
    
    internalSettings = new ITMLibSettings();
    mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(),
                                   imageSource->getDepthImageSize());
    
    setupDone = true;
}

// MARK: - Navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the point cloud to the new view controller.
    if ([segue.identifier  isEqual: @"Show Cloud"]) {
        NSLog(@"Show Cloud Segue");
    }
}

// MARK: - Actions

// TODO: hold button to record, release to stop record. click button to start/stop fusion.

- (IBAction)bProcessOne_clicked:(id)sender
{
    if (usingSensor)
    {
        isRecording = !isRecording;
        return;
    }
    
    if (!imageSource->hasMoreImages()) return;
    
    imageSource->getImages(inputRGBImage, inputRawDepthImage);
    
    dispatch_async(self.renderingQueue, ^{
        [self updateImage];
    });
}

- (IBAction)bProcessCont_clicked:(id)sender
{
    ++numFusionButtonClicked;
    // If usingSensor, update UI when sensorDidOutputDepthFrame()
    if (usingSensor)
    {
        // When click every the second time, end fullProcess and navigation to PointCloudViewController
        fullProcess = !fullProcess;
        if (numFusionButtonClicked > 0 && numFusionButtonClicked%2 == 0) {
            [self performSegueWithIdentifier:@"Show Cloud" sender:sender];
        }
        [self.tbOut setText:@"front depth"];
        return;  // return if usingSensor, using CalibSource imageSource Engine.
    }
    
    // If not usingSensor, using RawFileReader imageSource Engine, update UI in while loop in renderingQueue.
    // Read previously saved rgb and depth image from documentsPath/Out (following named currentFrameNo), process and then render to UI.
    // Process and update each frame in another thread other than main_queue
    // dispatch_async allows to operate (e.g. click in main UI) in main thread 1, while processing frames in renderingQueue in another thread.
    dispatch_async(self.renderingQueue, ^{
        while (imageSource->hasMoreImages() && imuSource->hasMoreMeasurements())
        {
            imageSource->getImages(inputRGBImage, inputRawDepthImage);
            imuSource->getMeasurement(imuMeasurement);
            [self updateImage];
        }
    });
}

// MARK: - UI Utility Functions

// Call this on the renderingQueue.
- (void) updateImage
{
    if (fullProcess) mainEngine->turnOnMainProcessing();
    else mainEngine->turnOffMainProcessing();
        
    NSDate *timerStart = [NSDate date];
    
    if (imuMeasurement != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, imuMeasurement);
    else mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    if (fullProcess)
    {
        totalProcessedFrames++;
        totalProcessingTime += executionTime;
    }
    
    // Get and update output rendered raycasted image.
    if (fullProcess) mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
    else mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    
    CGContextRef cgContext = CGBitmapContextCreate(result->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8,
                                                     4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    CGImageRef cgImage = CGBitmapContextCreateImage(cgContext);  // copy original depth from memory to CGImageRef

    // Mirror and rotate rendered image if using front camera in portrait interfaceOrientation.
    CGContextScaleCTM(cgContext, 1.0, -1.0);
    CGContextTranslateCTM(cgContext, 0.0, -imageSize.y);
    CGContextRotateCTM(cgContext, 90.0/180.0*M_PI);  // rotate the user coordinate system
    CGContextTranslateCTM(cgContext, 0.0, -imageSize.x);
    CGContextDrawImage(cgContext, CGRectMake(0, 0, imageSize.y, imageSize.x), cgImage);  // draw cgImage to cgContext memory
    CGImageRef cgImageTransformed = CGBitmapContextCreateImage(cgContext);  // copy mirrored cgImageMirrored from cgContext memory
    
    // After processing one frame in rendering_queue, callback and send notification to the main_queue.
    dispatch_sync(dispatch_get_main_queue(), ^{
        self.renderView.layer.contents = (__bridge id)cgImageTransformed;
        
        if (fullProcess && !isRecording) {
            NSString *theValue = [NSString stringWithFormat:@"%5.4lf spf", totalProcessingTime / totalProcessedFrames];
            [self.tbOut setText:theValue];
        }
    });

    CGImageRelease(cgImageTransformed);
    CGImageRelease(cgImage);
    CGContextRelease(cgContext);
}

// MARK: - Depth Data Output Delegate

// Once receive one depth frame, reload rotationMatrix and inputRawDepthImage, then update rendering image in UI.
- (void)depthDataOutput:(AVCaptureDepthDataOutput *)output didOutputDepthData:(AVDepthData *)depthData timestamp:(CMTime)timestamp connection:(AVCaptureConnection *)connection
{
    if (setupDone)
    {
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        
        // Calibration iPhone depth first and only once, intrinsic reference to imageSize(320, 180).
        // TODO: depth distortion correction.
        if (!calibrated)
        {
            AVCameraCalibrationData* calibData = depthData.cameraCalibrationData;
//            float pixelSize = calibData.pixelSize;  // The size, in millimeters, of one image pixel.
            matrix_float3x3 intrinsicMatrix = calibData.intrinsicMatrix;
//            matrix_float4x3 extrinsicMatrix = calibData.extrinsicMatrix;  // translation vector's units are millimeters.
            CGSize imgDim = calibData.intrinsicMatrixReferenceDimensions;

            float refPixelX = imgDim.width;  // 3840
            float refPixelY = imgDim.height;  // 2160
            scaleFactor[0] = imageSize[0] / refPixelX;
            scaleFactor[1] = imageSize[1] / refPixelY;
            float fx = intrinsicMatrix.columns[0][0];
            float fy = intrinsicMatrix.columns[1][1];
            float cx = intrinsicMatrix.columns[2][0];
            float cy = intrinsicMatrix.columns[2][1];

            NSAssert(scaleFactor[0] == scaleFactor[1], @"scaleX and scaleY must be the same.");
            ((iPhoneSource*)imageSource)->calibrate(fx, fy, cx, cy, scaleFactor[0]);
            calibrated = true;
        }
        
//        // Maybe depthDataType is kCVPixelFormatType_DisparityFloat16.
//        if (depthData.depthDataType != kCVPixelFormatType_DepthFloat16) {
//            depthData = [depthData depthDataByConvertingToDepthDataType:kCVPixelFormatType_DepthFloat16];
//        }
        
        // Copy depth pixelBuffer to inputRawDepthImage
        CVPixelBufferLockBaseAddress(depthData.depthDataMap, kCVPixelBufferLock_ReadOnly);
        memcpy(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), CVPixelBufferGetBaseAddress(depthData.depthDataMap), imageSize.x * imageSize.y * sizeof(short));
        CVPixelBufferUnlockBaseAddress(depthData.depthDataMap, kCVPixelBufferLock_ReadOnly);
        
        dispatch_async(self.renderingQueue, ^{
            if (isRecording)
            {  // If recording, save sensor captured depth images and IMU rotationMatrix to documentsPath/Out naming currentFrameNo
                FILE *f; char fileName[2000];
                
                sprintf(fileName, "%s/Output/img_%08d.irw", documentsPath, currentFrameNo);
                f = fopen(fileName, "wb+");
                fwrite(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), imageSize.x * imageSize.y * sizeof(short), 1, f);
                fclose(f);
                
                sprintf(fileName, "%s/Output/imu_%08d.txt", documentsPath, currentFrameNo);
                f = fopen(fileName, "w+");
                fprintf(f, "%f %f %f %f %f %f %f %f %f",
                        rotationMatrix.m11, rotationMatrix.m12, rotationMatrix.m13,
                        rotationMatrix.m21, rotationMatrix.m22, rotationMatrix.m23,
                        rotationMatrix.m31, rotationMatrix.m32, rotationMatrix.m33);
                
                fclose(f);
                
                if (currentFrameNo % 5 == 0) {
                    dispatch_async(dispatch_get_main_queue(), ^{
                        NSString *theValue = [NSString stringWithFormat:@"record %d", currentFrameNo];
                        [self.tbOut setText:theValue];
                    });
                }

                currentFrameNo++;
            }
            
            [self updateImage];
        });
    }
}

@end
