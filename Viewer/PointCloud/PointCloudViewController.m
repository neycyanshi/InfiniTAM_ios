//
//  PointCloudViewController.m
//  BodyFusion
//
//  Created by yanshi on 2019/4/3.
//  Copyright © 2019 yanshi. All rights reserved.
//

#import "PointCloudViewController.h"

#import <ModelIO/ModelIO.h>
#import <MetalKit/MetalKit.h>

@interface PointCloudViewController ()

@end

@implementation PointCloudViewController
{
    // MARK: - Properties
    
    SCNScene* scene;
//    SCNNode* pointNode;
    float zCamera;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self setupScene];
    [self drawModel];
}

- (void)setupScene {
    scene = [SCNScene scene];
    zCamera = 0.3f;

    SCNNode* cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    cameraNode.camera.zNear = 0.0;
    cameraNode.camera.zFar = 10.0;
    cameraNode.position = SCNVector3Make(0.0, 0.0, zCamera);
    [scene.rootNode addChildNode:cameraNode];
    
    SCNNode* lightNode = [SCNNode node];
    lightNode.light = [SCNLight light];
    lightNode.light.type = SCNLightTypeOmni;
//    lightNode.light.color = UIColor.whiteColor;
    lightNode.position = SCNVector3Make(0.0, 0.0, 3.0);
    [scene.rootNode addChildNode:lightNode];
    
//    SCNSphere* sphere = [SCNSphere sphereWithRadius:0.001];
//    sphere.firstMaterial.diffuse.contents = UIColor.blueColor;
//    pointNode = [SCNNode nodeWithGeometry:sphere];
    
    self.scnView.scene = scene;
    self.scnView.allowsCameraControl = YES;
    self.scnView.showsStatistics = YES;
    self.scnView.backgroundColor = UIColor.blackColor;
}

- (void)drawModel {
    NSLog(@"draw model: %@", self.modelPath);
    MDLAsset* asset = [[MDLAsset alloc] initWithURL:[NSURL URLWithString:self.modelPath]
                                   vertexDescriptor:<#(nullable MDLVertexDescriptor *)#>
                                    bufferAllocator:[MTKMeshBufferAllocator ]];
}

- (void)drawPointCloud {
    
//    guard let colorImage = image, let cgColorImage = colorImage.cgImage else { fatalError() }
//    guard let depthData = depthData else { fatalError() }
//
//    let depthPixelBuffer = depthData.depthDataMap
//    int width  = CVPixelBufferGetWidth(depthPixelBuffer)
//    int height = CVPixelBufferGetHeight(depthPixelBuffer)
//
//    let resizeScale = CGFloat(width) / colorImage.size.width
//    let resizedColorImage = CIImage(cgImage: cgColorImage).transformed(by: CGAffineTransform(scaleX: resizeScale, y: resizeScale))
//    guard let pixelDataColor = resizedColorImage.createCGImage().pixelData() else { fatalError() }
//
//    let pixelDataDepth: [Float32]
//    pixelDataDepth = depthPixelBuffer.grayPixelData()
    
//    // Sometimes the z values of the depth are bigger than the camera's z
//    // So, determine a z scale factor to make it visible
//    let zMax = pixelDataDepth.max()!
//    let zNear = zCamera - 0.2
//    let zScale = zMax > zNear ? zNear / zMax : 1.0
//    print("z scale: \(zScale)")
//    let xyScale: Float = 0.0002
//    
//    let pointCloud: [SCNVector3] = pixelDataDepth.enumerated().map {
//        let index = $0.offset
//        // Adjusting scale and translating to the center
//        let x = Float(index % width - width / 2) * xyScale
//        let y = Float(height / 2 - index / width) * xyScale
//        // z comes as Float32 value
//        let z = Float($0.element) * zScale
//        return SCNVector3(x, y, z)
//    }
//
//    // Draw as a custom geometry
//    let pc = PointCloud()
//    pc.pointCloud = pointCloud
//    pc.colors = pixelDataColor
//    let pcNode = pc.pointCloudNode()
//    pcNode.position = SCNVector3(x: 0, y: 0, z: 0)
//    scene.rootNode.addChildNode(pcNode)
}

- (void)drawTorus{
    SCNGeometry* torus = [SCNTorus torusWithRingRadius:0.05 pipeRadius:0.02];
    SCNNode* torusNode = [SCNNode nodeWithGeometry:torus];
    torus.firstMaterial.diffuse.contents = UIColor.cyanColor;
    torus.firstMaterial.specular.contents = UIColor.whiteColor;
    torus.firstMaterial.shininess = 1.0;
    SKTexture* noiseTexture = [SKTexture textureNoiseWithSmoothness:0.25 size:CGSizeMake(512, 512) grayscale:NO];
    //    SKTexture* noiseNormalMapTexture = [noiseTexture textureByGeneratingNormalMap];
    SKTexture* noiseNormalMapTexture = [noiseTexture textureByGeneratingNormalMapWithSmoothness:1.0 contrast:1.0];
    torus.firstMaterial.normal.contents = noiseNormalMapTexture;
    [torus.firstMaterial setLightingModelName:SCNLightingModelBlinn];
    torusNode.position = SCNVector3Make(0.0, 0.0, 0.0);
    [scene.rootNode addChildNode:torusNode];
}


/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation to another view from PointCloudView
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/

@end
