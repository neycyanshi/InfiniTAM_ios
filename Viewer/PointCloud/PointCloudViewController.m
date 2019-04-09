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
    // MARK: - Private Variables
    
    SCNScene *scene;
    SCNNode *modelNode;
    float zCamera;
}

- (void)viewDidLoad
{
    [super viewDidLoad];

    [self setupScene];
}

- (void)viewWillAppear:(BOOL)animated
{
    [super viewWillAppear:animated];

    [self createModelNode];
    [scene.rootNode addChildNode:modelNode];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [self deleteModel:self.modelPath];
    [modelNode removeFromParentNode];

    [super viewWillDisappear:animated];
}


// MARK: - Draw Scene

- (void)setupScene {
    scene = [SCNScene scene];
    zCamera = 0.3f;

    SCNNode* cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    cameraNode.camera.zNear = 0.0;
    cameraNode.camera.zFar = 10.0;
    cameraNode.position = SCNVector3Make(0.0, 0.0, zCamera);
    [scene.rootNode addChildNode:cameraNode];

    self.scnView.scene = scene;
    self.scnView.allowsCameraControl = YES;
    self.scnView.autoenablesDefaultLighting = YES;
    self.scnView.showsStatistics = YES;
    self.scnView.backgroundColor = UIColor.blackColor;
}

- (void)createModelNode {
    NSLog(@"create modelNode from: %@", self.modelPath);
    modelNode = [self loadSTLWithPath:self.modelPath];

    // Set 3D model material
//    modelNode.geometry.firstMaterial.diffuse.contents = [UIColor colorWithRed:0.6 green:0.6 blue:0.6 alpha:1];
    modelNode.geometry.firstMaterial.diffuse.contents = UIColor.cyanColor;
    modelNode.geometry.firstMaterial.specular.contents = UIColor.whiteColor;
    modelNode.geometry.firstMaterial.shininess = 1.0;
    modelNode.geometry.firstMaterial.lightingModelName = SCNLightingModelBlinn;
    
    // Set position, transform and scale
    modelNode.position = SCNVector3Make(0.0, 0.0, 0.0);
    SCNMatrix4 transform = SCNMatrix4Identity;
    transform = SCNMatrix4Rotate(transform, M_PI/2, 0, 0, -1);
    transform = SCNMatrix4Rotate(transform, M_PI, 0, 1, 0);
    float scaleFactor = 10.0;
    transform = SCNMatrix4Scale(transform, scaleFactor, scaleFactor, scaleFactor);
    modelNode.transform = transform;
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

// MARK: - Utility Functions

// Delete model if needed.
- (BOOL) deleteModel:(NSString*) modelPath
{
    return [[NSFileManager defaultManager] removeItemAtPath:modelPath error:NULL];
}

- (SCNNode *)loadSTLWithPath:(NSString *)path
{
    SCNNode *node = nil;
    NSData *data = [NSData dataWithContentsOfFile:path];
    if (data.length > 80)
    {
        // STL header contains 80 chars
        NSData *headerData = [data subdataWithRange:NSMakeRange(0, 80)];
        NSString *headerStr = [[NSString alloc] initWithData:headerData encoding:NSASCIIStringEncoding];
        if ([headerStr containsString:@"solid"])
        {
            //ASCII encoded STL file
            node = [self loadASCIISTLWithData:data];
        }
        else
        {
            //Binary encoded STL file
            node = [self loadBinarySTLWithData:[data subdataWithRange:NSMakeRange(84, data.length - 84)]];
        }
    }
    return node;
}

- (SCNNode *)loadBinarySTLWithData:(NSData *)data
{
    NSMutableData *vertices = [NSMutableData data];
    if (data.length % 50 != 0)
    {
        NSLog(@"STL(Binary) file error");
        return nil;
    }
    NSInteger allCount = data.length/50;

    // Parse simple STL without normal
    for (int i = 0; i < allCount; i ++)
    {
        for (int j = 1; j <= 3; j ++)
        {
            [vertices appendData:[data subdataWithRange:NSMakeRange(i * 50 + j*12, 12)]];
        }
    }
    SCNGeometrySource *verticesSource = [SCNGeometrySource geometrySourceWithData:vertices semantic:SCNGeometrySourceSemanticVertex vectorCount:allCount*3 floatComponents:YES componentsPerVector:3 bytesPerComponent:sizeof(float) dataOffset:0 dataStride:sizeof(SCNVector3)];
    SCNGeometryElement *geometryElement = [SCNGeometryElement geometryElementWithData:nil primitiveType:SCNGeometryPrimitiveTypeTriangles primitiveCount:allCount bytesPerIndex:sizeof(int)];
    SCNGeometry *geometry = [SCNGeometry geometryWithSources:@[verticesSource] elements:@[geometryElement]];

    // Create SCNNode
    SCNNode *node = [SCNNode nodeWithGeometry:geometry];
    return node;
}

- (SCNNode *)loadASCIISTLWithData:(NSData *)data
{
    NSMutableData *vertices = [NSMutableData data];
    NSMutableData *normals = [NSMutableData data];
    NSMutableData *elements = [NSMutableData data];
    NSString *asciiStr = [[NSString alloc] initWithData:data encoding:NSASCIIStringEncoding];
    NSArray *asciiStrArr = [asciiStr componentsSeparatedByString:@"\n"];
    int elementCount = 0;
    for (int i = 0; i < asciiStrArr.count; i ++)
    {
        NSString *currentStr = asciiStrArr[i];
        if ([currentStr containsString:@"facet"])
        {
            if ([currentStr containsString:@"normal"])
            {
                for (int j = 1; j <= 3; j++)
                {
                    NSArray *subNormal = [currentStr componentsSeparatedByString:@" "];
                    SCNVector3 normal = SCNVector3Make([subNormal[subNormal.count - 3] floatValue], [subNormal[subNormal.count - 2] floatValue], [subNormal[subNormal.count - 1] floatValue]);
                    [normals appendBytes:&normal length:sizeof(normal)];
                    NSArray *subVertice = [asciiStrArr[i+j+1] componentsSeparatedByString:@" "];
                    SCNVector3 vertice = SCNVector3Make([subVertice[subVertice.count - 3] floatValue], [subVertice[subVertice.count - 2] floatValue], [subVertice[subVertice.count - 1] floatValue]);
                    [vertices appendBytes:&vertice length:sizeof(vertice)];
                }
                int element[3] = {elementCount * 3,elementCount * 3 + 1,elementCount * 3 + 2};
                elementCount++;
                [elements appendBytes:&element length:sizeof(element)];
                i = i+6;
            }
        }
    }
    SCNGeometrySource *verticesSource = [SCNGeometrySource geometrySourceWithData:vertices semantic:SCNGeometrySourceSemanticVertex vectorCount:(elementCount-1)*3 floatComponents:YES componentsPerVector:3 bytesPerComponent:sizeof(float) dataOffset:0 dataStride:sizeof(SCNVector3)];
    SCNGeometrySource *normalsSource = [SCNGeometrySource geometrySourceWithData:normals semantic:SCNGeometrySourceSemanticNormal vectorCount:(elementCount-1)*3 floatComponents:YES componentsPerVector:3 bytesPerComponent:sizeof(float) dataOffset:0 dataStride:sizeof(SCNVector3)];
    SCNGeometryElement *geoMetryElement = [SCNGeometryElement geometryElementWithData:elements primitiveType:SCNGeometryPrimitiveTypeTriangles primitiveCount:elementCount - 1 bytesPerIndex:sizeof(int)];
    SCNGeometry *geometry = [SCNGeometry geometryWithSources:@[verticesSource,normalsSource] elements:@[geoMetryElement]];
    // Create SCNNode
    SCNNode *node = [SCNNode nodeWithGeometry:geometry];
    return node;
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
