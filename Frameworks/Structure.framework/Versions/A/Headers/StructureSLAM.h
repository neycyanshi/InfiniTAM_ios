/*
    This file is part of the Structure SDK.
    Copyright © 2016 Occipital, Inc. All rights reserved.
    http://structure.io
*/

#pragma once

#import <Structure/Structure.h>
#import <CoreMotion/CoreMotion.h>
#import <GLKit/GLKit.h>

//------------------------------------------------------------------------------
#pragma mark - STMesh

// Dictionary keys for [STMesh writeToFile:options:error:].
extern NSString* const kSTMeshWriteOptionFileFormatKey;
extern NSString* const kSTMeshWriteOptionUseXRightYUpConventionKey;

/** Constants specifying a file format for storing an STMesh on disk.

See also:

- [STMesh writeToFile:options:error:]
*/
typedef NS_ENUM(NSInteger, STMeshWriteOptionFileFormat)
{
/** Wavefront OBJ format.
    
If the mesh has a texture, an MTL file will also be generated, and the texture saved to a JPEG file.
    
Filenames with spaces are not supported by OBJ and will return an error.
*/
    STMeshWriteOptionFileFormatObjFile     = 0,
    
/** Wavefront OBJ format, compressed into a ZIP file.
    
The archive will also embed the MTL and JPEG file if the mesh has a texture.
*/
    STMeshWriteOptionFileFormatObjFileZip  = 1
};

/** Reference to face-vertex triangle mesh data.

Stores mesh data as a collection of vertices and faces. STMesh objects are references, and access to the underlying data should be protected by locks in case multiple threads may be accessing it.

Since OpenGL ES only supports 16 bits unsigned short for face indices, meshes larger than 65535 faces have to be split into smaller sub-meshes. STMesh is therefore a reference to a collection of partial meshes, each of them having less than 65k faces.
*/
@interface STMesh : NSObject

/// Number of partial meshes.
- (int)numberOfMeshes;

/** Number of faces of a given submesh.

@param meshIndex Index to the partial mesh.
*/
- (int)numberOfMeshFaces:(int)meshIndex;

/** Number of vertices of a given submesh.

@param meshIndex Index to the partial mesh.
*/
- (int)numberOfMeshVertices:(int)meshIndex;

/** Number of lines (edges) of a given submesh.

@param meshIndex Index to the partial mesh.
*/
- (int)numberOfMeshLines:(int)meshIndex;

/// Whether per-vertex normals are available.
- (BOOL)hasPerVertexNormals;

/// Whether per-vertex colors are available.
- (BOOL)hasPerVertexColors;

/// Whether per-vertex UV texture coordinates are available.
- (BOOL)hasPerVertexUVTextureCoords;

/** Pointer to a contiguous chunk of `numberOfMeshVertices:meshIndex` `GLKVector3` values representing (x, y, z) vertex coordinates.

@param meshIndex Index to the partial mesh.
*/
- (GLKVector3 *)meshVertices:(int)meshIndex;

/** Pointer to a contiguous chunk of `numberOfMeshVertices:meshIndex` `GLKVector3` values representing (nx, ny, nz) per-vertex normals.

@note Returns `NULL` is there are no per-vertex normals.

@param meshIndex Index to the partial mesh.
*/
- (GLKVector3 *)meshPerVertexNormals:(int)meshIndex;

/** Pointer to a contiguous chunk of `numberOfMeshVertices:meshIndex` `GLKVector3` values representing (r, g, b) vertices colors.

@note Returns `NULL` is there are no per-vertex colors.

@param meshIndex Index to the partial mesh.
*/
- (GLKVector3 *)meshPerVertexColors:(int)meshIndex;

/** Pointer to a contiguous chunk of `numberOfMeshVertices:meshIndex` `GLKVector2` values representing normalized (u, v) texture coordinates.

@note Returns `NULL` is there are no per-vertex texture coordinates.

@param meshIndex Index to the partial mesh.
*/
- (GLKVector2 *)meshPerVertexUVTextureCoords:(int)meshIndex;

/** Pointer to a contiguous chunk of `(3 * numberOfMeshFaces:meshIndex)` 16 bits `unsigned short` values representing vertex indices. Each face is represented by three vertex indices.

@param meshIndex Index to the partial mesh.
*/
- (unsigned short *)meshFaces:(int)meshIndex;

/** Optional texture associated with the mesh.

The pixel buffer is encoded using `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange`.
*/
- (CVPixelBufferRef)meshYCbCrTexture;

/** Pointer to a contiguous chunk of `(2 * numberOfMeshLines:meshIndex)` 16 bits `unsigned short` values representing vertex indices.

Each line is represented by two vertex indices. These lines can be used for wireframe rendering, using GL_LINES.

@param meshIndex Index to the partial mesh.
*/
- (unsigned short *)meshLines:(int)meshIndex;

/** Save the STMesh to a file.

Sample usage:

    NSError* error;
    [myMesh writeToFile:@"/path/to/mesh.obj"
                options:@{kSTMeshWriteOptionFileFormatKey: STMeshWriteOptionFileFormatObjFile} 
                  error:&error];

@param filePath Path to output file.
@param options Dictionary of options. The valid keys are:

- `kSTMeshWriteOptionFileFormatKey`: STMeshWriteOptionFileFormat value to specify the output file format. Required.
- `kSTMeshWriteOptionUseXRightYUpConventionKey`: Sets the exported mesh coordinate frame to be X right, Y Up, and Z inwards (right handed).

@param error will contain detailed information if the provided options are incorrect.
*/
- (BOOL)writeToFile:(NSString *)filePath options:(NSDictionary *)options error:(NSError* __autoreleasing *)error;

/** Create a copy of the current mesh.

@param mesh The mesh from which to copy.
*/
- (instancetype)initWithMesh:(STMesh *)mesh;

/** Return an asynchronous task to create a decimated low-poly version of the given mesh with a maximal target number of faces.

Sample usage:

    __block STMesh* outputMesh;
    STBackgroundTask* task;
    task = [STMesh newDecimateTaskWithMesh:myMesh
                                  numFaces:2000
                         completionHandler:^(STMesh *result, NSError *error) { outputMesh = result; };
    [task start];
    [task waitUntilCompletion];

@note If the target number of faces is larger than the current mesh number of faces, no processing is done.

@param inputMesh Input mesh to decimate.
@param numFaces Target number of faces to decimate.
@param completionHandler Block to execute once the task is finished or cancelled.
*/
+ (STBackgroundTask*)newDecimateTaskWithMesh:(STMesh *)inputMesh numFaces:(unsigned int)numFaces completionHandler:(void(^)(STMesh *result, NSError *error))completionHandler;

/** Return an asynchronous task to create a version of the given mesh with holes filled.

@note Additionally, the output will result in a smoother mesh, with non-manifold faces removed.

@param inputMesh Input mesh to fill holes.
@param completionHandler Block to execute once the task is finished or cancelled.
*/
+ (STBackgroundTask*)newFillHolesTaskWithMesh:(STMesh *)inputMesh completionHandler:(void(^)(STMesh *result, NSError *error))completionHandler;

@end

//------------------------------------------------------------------------------
#pragma mark - STMeshIntersector
/** A utility class to compute the intersection of a ray to the mesh
 */
@interface STMeshIntersector : NSObject
- (instancetype)initWithMesh:(STMesh*)inputMesh;
/** Intersect the mesh with a ray specified by the origin and end points.
 If TRUE is returned, `intersection` will contain the first (x, y, z) coordinate on the mesh that the ray intersects when traveling from the origin to the end.
 @param origin The origin of ray.
 @param end The end of ray.
 @param intersection The intersection point to the mesh if intersection happens.
 @param normal The vector describing the surface normal.
 @param ignoreBackFace whether to ignore the back face when computing the intersection.
 @return TRUE if there is an intersection, FALSE otherwise
 */
- (BOOL)intersectWithRayOrigin:(GLKVector3)origin rayEnd:(GLKVector3)end intersection:(GLKVector3 *)intersection normal:(GLKVector3 *)normal ignoreBackFace:(BOOL)ignoreBackFace;
- (BOOL)intersectWithRayOrigin:(GLKVector3)origin rayEnd:(GLKVector3)end intersection:(GLKVector3 *)intersection intersectionFaceIndex:(int *) intersectionFaceIndex ignoreBackFace:(BOOL)ignoreBackFace;
- (BOOL)faceIsOnAPlane:(int)faceIndex normal:(GLKVector3 *)normal;
@end

//------------------------------------------------------------------------------
#pragma mark - STScene

/** Common data shared and updated by the SLAM pipeline.

An STScene object contains information about the sensor and the reconstructed mesh.

Special care should be taken when accessing STScene members if an STTracker or STMapper is still active, as they could be accessing the STScene from background threads.
In particular, STMesh objects should be properly locked.
*/
@interface STScene : NSObject

/** Mandatory initializer for STScene.

@param glContext a valid EAGLContext.
*/
- (instancetype)initWithContext:(EAGLContext *)glContext;

/** Reference to the current scene mesh.

This mesh may be modified by a background thread if an instance of STMapper is running, so proper locking is necessary.
*/
- (STMesh *)lockAndGetSceneMesh;

/// Unlocks the mesh previously locked with `lockAndGetSceneMesh`.
- (void)unlockSceneMesh;

/** Render the scene mesh from the given viewpoint.

A virtual camera with the given projection and pose matrices will be used to render the mesh using OpenGL. This method is generally faster than using sceneMeshRef and manually rendering it, since in most cases STScene can reuse mesh data previously uploaded to the GPU.

@param cameraPose A GLKMatrix4 camera position used for rendering.
@param glProjection Projection matrix used during rendering. See also [STDepthFrame glProjectionMatrix] and [STColorFrame glProjectionMatrix].
@param alpha A float value for transparency between 0 (fully transparent) and 1 (fully opaque)
@param highlightOutOfRangeDepth Whether the areas of the mesh which are below Structure Sensor minimal depth range should be highlighted in red.
@param wireframe Whether to render a wireframe of the mesh or filled polygons. When enabled, STMapper needs to be initialized with the `kSTMapperEnableLiveWireFrameKey` option set to `@YES`.

@return TRUE on success, FALSE if there is no STMapper attached to the same STScene with the corresponding (wireframe or triangles) live mode enabled.
*/
- (BOOL)renderMeshFromViewpoint:(GLKMatrix4)cameraPose
             cameraGLProjection:(GLKMatrix4)glProjection
                          alpha:(float)alpha
       highlightOutOfRangeDepth:(BOOL)highlightOutOfRangeDepth
                      wireframe:(BOOL)wireframe;

/// Clear the scene mesh and state.
- (void)clear;

@end

//------------------------------------------------------------------------------
#pragma mark - STTracker

/** Constants specifying the tracking algorithm used by STTracker.

See also:
 
- [STTracker initWithScene:options:]
*/
typedef NS_ENUM(NSInteger, STTrackerType)
{
/** Specifies a tracker that will only use the depth information from Structure Sensor.

This tracker works best at close/mid-range, in combination with `kSTTrackerTrackAgainstModelKey`.
*/
    STTrackerDepthBased = 0,

/** Specifies a tracker that will use both the depth information from Structure Sensor and the color information from the iOS device camera.

Only `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange` is supported for the color buffer format.
*/
    STTrackerDepthAndColorBased = 1,
};

/** Constants that specify profiles to optimize the tracker for.
 
 See also:
 
 - [STTracker initWithScene:options:]
 - [STTracker setOptions:]
 */
typedef NS_ENUM(NSInteger, STTrackerSceneType)
{
/** Specifies a profile that optimizes for scanning objects.

Assumes that the sensor is moving and can rotate and translate.
*/
    STTrackerSceneTypeObject,
    
/** Specifies a profile that optimizes for scanning objects on a turntable.

Assumes that the sensor is stationary and the object itself rotates.
*/
    STTrackerSceneTypeObjectOnTurntable,
    
/** Specifies a profile that optimizes for scanning rooms

Assumes that the sensor is moving and can rotate and translate. The scene captured should be on the scale of a typical room scan.
*/
    STTrackerSceneTypeRoom,
};

/** Constants specifying a tracking quality hint to STTracker.

See also:

- [STTracker initWithScene:options:]
- [STTracker setOptions:]
*/
typedef NS_ENUM(NSInteger, STTrackerQuality)
{
/// Best during scanning, but it will also take more CPU resources.
    STTrackerQualityAccurate = 0,

/// Designed for very fast tracking, it works best when tracking against a static mesh (e.g. after a scan has been done), or when the available CPU resources are limited.
    STTrackerQualityFast,
};

/** Constants specifying the quality of the STTracker output.

See also:

- [STTracker poseAccuracy]
- [STTracker trackerHints]
*/
typedef NS_ENUM(NSInteger, STTrackerPoseAccuracy)
{
/// The tracker cannot provide any accuracy information. This typically happens when it gets lost or before it sees any frame.
    STTrackerPoseAccuracyNotAvailable,
    
/// The tracker pose accuracy is very low. This can happen when the model has been out of view for a long time or if very few pixels have depth data.
    STTrackerPoseAccuracyVeryLow,

/// The tracker pose accuracy is low. This can happen when tracking against a model that is getting too close or out of view.
    STTrackerPoseAccuracyLow,

/// The tracker pose accuracy is approximate. This can happen during fast movements.
    STTrackerPoseAccuracyApproximate,
    
/// The tracker pose accuracy is nominal.
    STTrackerPoseAccuracyHigh,
};

// Dictionary keys for [STTracker initWithScene:options:].
extern NSString* const kSTTrackerTypeKey;
extern NSString* const kSTTrackerQualityKey;
extern NSString* const kSTTrackerTrackAgainstModelKey;
extern NSString* const kSTTrackerAvoidPitchRollDriftKey;
extern NSString* const kSTTrackerAvoidHeightDriftKey;
extern NSString* const kSTTrackerAcceptVaryingColorExposureKey;
extern NSString* const kSTTrackerBackgroundProcessingEnabledKey;
extern NSString* const kSTTrackerSceneTypeKey;
extern NSString* const kSTTrackerLegacyKey;

/** Tracker Hints

An instance of this struct is provided by the tracker to provide feedback on how to reach a stable pose estimation.

See also:

- [STTracker trackerHints]
- [STTracker poseAccuracy]
*/
typedef struct
{
/// The tracker is completely lost and could not provide a pose estimation.
    bool trackerIsLost;
    
/// The sensor is too close to the scene, creating too many depth values below the sensor range.
    bool sceneIsTooClose;
    
/// When kSTTrackerTrackAgainstModelKey was set, this is true if the model being scanned in not in the view anymore.
    bool modelOutOfView;
}
STTrackerHints;

STTrackerHints STTrackerHints_init (void);

/** A STTracker instance tracks the 3D position of the Structure Sensor.

It uses sensor information and optionally IMU data to estimate how the camera is being moved over time, in real-time.

See also:

- STTrackerType
*/
@interface STTracker : NSObject

/// STScene object storing common SLAM information.
@property (nonatomic, retain) STScene *scene;

/// The initial camera pose. Tracking will use this as the first frame pose.
@property (nonatomic) GLKMatrix4 initialCameraPose;

/** The current tracker hints.

See also:

- STTrackerHints
- [STTracker poseAccuracy]
*/
@property (nonatomic,readonly) STTrackerHints trackerHints;

/** The current tracker pose accuracy.

See also:

- STTrackerPoseAccuracy
- [STTracker trackerHints]
*/
@property (nonatomic,readonly) STTrackerPoseAccuracy poseAccuracy;

/** STTracker initialization.

STTracker cannot be used until an STScene has been provided.

Sample usage:

    NSDictionary* options = @{
        kSTTrackerTypeKey                       = @(STTrackerDepthBased),
        kSTTrackerQualityKey                    = @(STTrackerQualityAccurate),
        kSTTrackerTrackAgainstModelKey          = @YES,
        kSTTrackerAcceptVaryingColorExposureKey = @NO,
        kSTTrackerBackgroundProcessingEnabledKey= @YES,
        kSTTrackerLegacyKey                     = @NO,
        kSTTrackerSceneTypeKey                  = @(STTrackerSceneTypeObject)
    };
    STTracker* tracker = [[STTracker alloc] initWithScene:myScene options:options];

@param scene The STScene context.
@param options Dictionary of options. The valid keys are:

- `kSTTrackerTypeKey`:
  - Specifies the tracking algorithm.
  - NSNumber integral value equal to one the STTrackerType constants.
  - Required.
- `kSTTrackerQualityKey`:
  - Specifies a tracking quality hint.
  - NSNumber integral value equal to one the STTrackerQuality constants.
  - Defaults to `STTrackerQualityAccurate`.
- `kSTTrackerTrackAgainstModelKey`:
  - Specifies whether to enable tracking against the model.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - If enabled, the tracker will attempt to match the current image with the current state of the reconstructed model. This can drastically reduce the pose estimation drift.
  - __Note:__ this option requires an STMapper to be attached to the scene.
- `kSTTrackerAvoidPitchRollDriftKey`:
  - Specifies whether to enable pitch and roll drift cancellation using IMU.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - This can eliminate drift along these rotation axis, but can also result in lower short term accuracy.
  - Recommended for unbounded tracking.
- `kSTTrackerAvoidHeightDriftKey`:
  - Specifies whether to enable height drift cancellation using ground plane detection.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - This can eliminate vertical drift, but can also result in lower short term accuracy.
  - Recommended for unbounded tracking.
- `kSTTrackerBackgroundProcessingEnabledKey`:
  - Specifies whether to enable background processing.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - This can significantly reduce the time spent in the tracker before getting a new pose, but the tracker may keep using CPU/GPU resources between frames.
- `kSTTrackerAcceptVaryingColorExposureKey`:
  - Specifies whether to accept varying exposures for the iOS camera.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - To ensure the optimal robustness, it is recommended to lock the iOS color camera exposure during tracking if the `STTrackerDepthAndColorBased` tracker type is being used.
  - By default, the tracker will return an error if it detects a change in the exposure settings, but it can be forced to accept it by enabling this option.
- `kSTTrackerLegacyKey`:
  - As of SDK 0.8, improved tracking is enabled by default. Set this to @YES if you want to enable the pre-0.8 tracking behavior.
  - Defaults to `@NO`.
  - This setting affects all tracker types, i.e. both STTrackerDepthBased and STTrackerDepthAndColorBased.
- `kSTTrackerSceneTypeKey`
  - Specifies a general "<span>scene</span> type" to account for tracker-specific presets.
  - NSNumber integral value equal to one of the STTrackerSceneType constants.
  - Defaults to STTrackerSceneTypeObject.
  - You will get better tracking if this matches the scene you are tracking against.
  - STTrackerSceneTypeObject assumes the target is to scan objects or people (e.g. Scanner).
  - STTrackerSceneTypeObjectOnTurntable assumes the target is an object or person on a moving platform and that the device (iOS device + Sensor) are not moving.
  - STTrackerSceneTypeRoom assumes the target is to scan a room (e.g. RoomCapture).
*/
- (instancetype)initWithScene:(STScene *)scene options:(NSDictionary*)options;

/// Reset the tracker to its initial state.
- (void)reset;

/** Update the camera pose estimate using the given depth frame.

@param depthFrame The STDepthFrame depth frame.
@param colorFrame The STColorFrame color buffer.
@param error On return, if it fails, points to an NSError describing the failure.

@return TRUE if success, FALSE otherwise, filling error with the explanation.

@note colorFrame can be nil if the tracker type is `STTrackerDepthBased`. For tracker type `STTrackerDepthAndColorBased`, only `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange` is supported for sampleBuffer format.
*/
- (BOOL)updateCameraPoseWithDepthFrame:(STDepthFrame *)depthFrame colorFrame:(STColorFrame*)colorFrame error:(NSError* __autoreleasing *)error;

/** Update the current pose estimates using the provided motion data.

@param motionData Provided motion data.
*/
- (void)updateCameraPoseWithMotion:(CMDeviceMotion *)motionData;

/// Return the most recent camera pose estimate.
- (GLKMatrix4)lastFrameCameraPose;

/** Dynamically adjust the tracker options.

Currently, only `kSTTrackerQualityKey` and `kSTTrackerBackgroundProcessingEnabledKey` can be changed after initialization. Other options will remain unchanged.

@param options Dictionary of options. The valid keys are:

- `kSTTrackerQualityKey`:
  - Specifies a tracking quality hint.
  - NSNumber integral value equal to one the STTrackerQuality constants.
  - Defaults to `STTrackerQualityAccurate`.
- `kSTTrackerBackgroundProcessingEnabledKey`:
  - Specifies whether to enable background processing.
  - NSNumber boolean value.
  - Defaults to `@NO`.
  - This can significantly reduce the time spent in the tracker before getting a new pose, but the tracker may keep using CPU/GPU resources between frames.
*/
- (void)setOptions:(NSDictionary *)options;

@end

//------------------------------------------------------------------------------
#pragma mark - STMapper

// Dictionary keys for [STMapper initWithScene: options:].
extern NSString* const kSTMapperVolumeResolutionKey;
extern NSString* const kSTMapperVolumeBoundsKey;
extern NSString* const kSTMapperVolumeHasSupportPlaneKey;
extern NSString* const kSTMapperEnableLiveWireFrameKey;
extern NSString* const kSTMapperDepthIntegrationFarThresholdKey;
extern NSString* const kSTMapperLegacyKey;

/** A STMapper instance integrates sensor data to reconstruct a 3D model of a scene.

It will update the scene mesh progressively as new depth frames are integrated.
It runs in a background thread, which means that it may update the scene object at any time.

It works over a fixed cuboid defining the volume of interest in the scene.
This volume can be initialized interactively using STCameraPoseInitializer.

The volume is defined by its size in the real world, in meters, and is discretized into voxels.
The volume resolution specifies the size of each voxel in meters. As a consequence, the higher the resolution, the more voxels the mapper will need to compute.
*/
@interface STMapper : NSObject

/// The STScene model which will be updated.
@property (nonatomic, retain) STScene *scene;

/**
Initialize with a given scene and volume resolution.

@param scene The STScene context.
@param options Dictionary of options. The valid keys are:

- `kSTMapperVolumeResolutionKey`:
  - Specifies the volume resolution as the size of each voxel in meters.
  - `NSNumber` floating point value.
  - Required.
- `kSTMapperVolumeBoundsKey`:
  - The extents of the bounding volume in number of voxels along each dimension.
  - `NSArray` of 3 `NSNumber` integral values.
  - Defaults to `@[ @(128), @(128), @(128) ]`.
  - To maintain good performance, we recommend to keep the boundaries under 290x290x290.
- `kSTMapperVolumeHasSupportPlaneKey`:
  - Specifies whether the volume cuboid has been initialized on top of a support plane.
  - `NSNumber` boolean value.
  - Defaults to `@NO`.
  - If the mapper is aware that the volume is on top of a support plane, it will adapt the pipeline to be more robust and scan only the object.
  - This value is typically set from `STCameraPoseInitializer.hasSupportPlane`.
- `kSTMapperEnableLiveWireFrameKey`:
  - Specifies whether the mapper should automatically build a wireframe mesh in the background when new depth frames are provided.
  - `NSNumber` boolean value.
  - Defaults to `@NO`.
- `kSTMapperDepthIntegrationFarThresholdKey`:
  - Specifies the depth integration far threshold.
  - Individual depth values beyond the far threshold will be ignored by the mapper.
  - `NSNumber` floating point value.
  - The value is in meters, and must be in the [0.5, 8.0] range.
  - Defaults to `@(4.0)`.
- `kSTMapperLegacyKey`:
  - Specifies whether the pre-0.5 mapping technique should be used.
  - `NSNumber` boolean value.
  - Defaults to `@NO`.
*/
- (instancetype)initWithScene:(STScene *)scene options:(NSDictionary*)options;

/// Reset the mapper state. This will also stop any background processing.
- (void)reset;

/** Integrate a new depth frame to the model.

The processing will be performed in a background queue, so this method is non-blocking.

@param depthFrame The depth frame.
@param cameraPose The viewpoint to use for mapping.
*/
- (void)integrateDepthFrame:(STDepthFrame *)depthFrame
                 cameraPose:(GLKMatrix4)cameraPose;

/// Wait until ongoing processing in the background queue finishes, and build the final triangle mesh.
- (void)finalizeTriangleMesh;

@end

//------------------------------------------------------------------------------
#pragma mark - STCameraPoseInitializer

/// Dictionary keys for [STCameraPoseInitializer initWithVolumeSizeInMeters: options:].
extern NSString* const kSTCameraPoseInitializerStrategyKey;

/** Constants specifying a camera pose initialization strategy.

See also:

- [STCameraPoseInitializer initWithVolumeSizeInMeters:options:]
*/
typedef NS_ENUM(NSInteger, STCameraPoseInitializerStrategy)
{
/** Try to detect a ground plane and set the camera pose such that the cuboid scanning volume lies on top of it.
If no ground plane is found or if the device is not looking downward, place the scanning volume at the distance given by the central depth pixels.
In both cases, the cuboid orientation will be aligned with the provided gravity vector.
__Note:__ This strategy requires depth information from the Structure Sensor.
*/
    STCameraPoseInitializerStrategyTableTopCube = 0,
    
/** Align the camera orientation using the gravity vector, leaving the translation component to (0,0,0).
__Note:__ This strategy does not require depth information.
*/
    STCameraPoseInitializerStrategyGravityAlignedAtOrigin = 1,
    
/** Align the camera orientation using the gravity vector, and places the camera center at the center of the scanning volume.
__Note:__ This strategy does not require depth information.
*/
    STCameraPoseInitializerStrategyGravityAlignedAtVolumeCenter = 2,
};

typedef struct STCameraPoseInitializerOutput
{
    /// Whether the pose initializer could find a good pose.
    BOOL hasValidPose;
    
    /// Estimated camera pose, taking Structure Sensor as a reference.
    GLKMatrix4 cameraPose;
    
    /// Whether the last cube placement was made with a supporting plane. Useful for STMapper.
    BOOL hasSupportPlane;
    
    /// Equation of the detected support plane (if hasSupportPlane is true)
    GLKVector4 supportPlane;
    
} STCameraPoseInitializerOutput;

/// Determine an initial camera pose to make the best use of the cuboid scanning volume.
@interface STCameraPoseInitializer : NSObject

/// Width, height and depth of the volume cuboid.
@property (nonatomic) GLKVector3 volumeSizeInMeters;

/// Return the last estimated pose. This is the recommended thread-safe way to get the output.
@property (nonatomic, readonly) struct STCameraPoseInitializerOutput lastOutput;

/// Most recent estimated camera pose, taking Structure Sensor as a reference.
@property (nonatomic, readonly) GLKMatrix4 cameraPose __deprecated_msg("use lastOutput instead.");

/// Whether the pose initializer could find a good pose.
@property (nonatomic, readonly) BOOL hasValidPose __deprecated_msg("use lastOutput instead.");

/// Whether the last cube placement was made with a supporting plane. Useful for STMapper.
@property (nonatomic, readonly) BOOL hasSupportPlane __deprecated_msg("use lastOutput instead.");

/** Initialize with all the required fields.

@param volumeSize The current volume size in meters. Not used by the `STCameraPoseInitializerStrategyGravityAlignedAtOrigin` strategy.
@param options The options dictionary. The valid keys are:

- `kSTCameraPoseInitializerStrategyKey`
  - Specifies the camera pose initialization strategy.
  - `NSNumber` integral value equal to one of the STCameraPoseInitializerStrategy constants.
  - Required.
*/
- (instancetype)initWithVolumeSizeInMeters:(GLKVector3)volumeSize
                                   options:(NSDictionary *)options;

/** Update the current pose estimate from a depth frame and a CoreMotion gravity vector, using the strategy specified at init.

@param gravityInDeviceFrame a gravity vector in IMU coordinates, e.g. as provided by CMDeviceMotion.gravity.
@param depthFrame the current processed depth from Structure Sensor. Can be `nil` if using the strategies `STCameraPoseInitializerStrategyGravityAlignedAtOrigin` or `STCameraPoseInitializerStrategyGravityAlignedAtVolumeCenter`.
@param error will contain detailed information if the estimation failed.

@return TRUE on success, FALSE on failure.
*/
- (BOOL)updateCameraPoseWithGravity:(GLKVector3)gravityInDeviceFrame depthFrame:(STDepthFrame *)depthFrame error:(NSError* __autoreleasing *)error;

/** Determine which pixels of a depth frame are inside the current scanning volume.
 
@param depthFrame the current processed depth from Structure Sensor.
@param outputMask should point to an allocated buffer of (`depthFrame.width * depthFrame.height`) pixels. A mask pixel will be 255 if the corresponding 3D point fits inside the volume, 0 if outside or if there is no depth.
*/
- (void)detectInnerPixelsWithDepthFrame:(STDepthFrame*)depthFrame mask:(uint8_t*)outputMask;

@end

//------------------------------------------------------------------------------
#pragma mark - STCubeRenderer

/** STCubeRenderer is an utility class for cuboid rendering.

STCubeRenderer can render a wireframe outline of a cube, and also highlight the part of scene which fits in the given cube.
This can be used to better visualize where the current cuboid scanning volume is located.
*/
@interface STCubeRenderer : NSObject

/** Initialize with required properties.

@param glContext The EAGLContext.
*/
- (instancetype)initWithContext:(EAGLContext *)glContext;

/** A depth frame is required before using renderHighlightedDepth.

@param depthFrame The depth frame.
*/
- (void)setDepthFrame:(STDepthFrame *)depthFrame;

/** Whether the cube has a support plane. Rendering will be adjusted in that case.

@param hasSupportPlane The boolean to enable adjustment of support plane in rendering.
*/
- (void)setCubeHasSupportPlane:(BOOL)hasSupportPlane;

/** Specify the cube size.

@param sizeInMeters The current volume size in meters.
*/
- (void)adjustCubeSize:(const GLKVector3)sizeInMeters;

/** Highlight the depth frame area which fits inside the cube.

@param cameraPose the viewpoint to use for rendering.
@param alpha transparency factor between 0 (fully transparent) and 1 (fully opaque)
*/
- (void)renderHighlightedDepthWithCameraPose:(GLKMatrix4)cameraPose alpha:(float)alpha;

/**
Render the cube wireframe outline at the given pose.
 
@param cameraPose the viewpoint to use for rendering.
@param depthTestEnabled whether the lines should be drawn with `GL_DEPTH_TEST` enabled. This should typically be disabled if used in combination with renderHighlightedDepthWithCameraPose: to avoid having the lines occluded, but enabled if a mesh is also being rendered in the scene.
@param occlusionTestEnabled whether to use the current depth frame to do occlusion testing. You can turn this off for better performance.
*/
- (void)renderCubeOutlineWithCameraPose:(GLKMatrix4)cameraPose
                       depthTestEnabled:(BOOL)depthTestEnabled
                   occlusionTestEnabled:(BOOL)occlusionTestEnabled;

@end

//------------------------------------------------------------------------------
#pragma mark - STNormalFrame

/** STNormalFrame instances store a processed normal frame with normal vector in each pixel.
 
See also:

- STNormalEstimator
*/
@interface STNormalFrame : NSObject

/// Image width.
@property (readonly, nonatomic) int width;

/// Image height.
@property (readonly, nonatomic) int height;

/// Pointer to the beginning of a contiguous chunk of (`width` * `height`) normal pixel values.
@property (readonly, nonatomic) const GLKVector3 *normals;

@end

//------------------------------------------------------------------------------
#pragma mark - STNormalEstimator

/** Helper class to estimate surface normals.
 
STNormalEstimator instances calculate a unit vector representing the surface normals for each depth pixel.

See also:

- STNormalFrame
*/
@interface STNormalEstimator : NSObject

/** Calculate normals with a depth frame. Pixels without depth will have NaN values.

@param floatDepthFrame The depth frame.

@return A STNormalFrame normal frame object.
*/
- (STNormalFrame *)calculateNormalsWithDepthFrame:(STDepthFrame *)floatDepthFrame;

@end

//------------------------------------------------------------------------------
#pragma mark - STKeyFrame

/// STKeyFrame instances store the depth, color and camera pose information of a single frame.
@interface STKeyFrame : NSObject

/** Initialize with required frame data and pose.

@param colorCameraPose The GLKMatrix4 camera pose for this keyFrame.
@param colorFrame The STColorFrame color buffer.
@param depthFrameOrNil The STDepthFrame depth frame image. Can be `nil` if the depth information is not needed.

@note Only `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange` is supported for sampleBuffer format.
*/
- (instancetype)initWithColorCameraPose:(GLKMatrix4)colorCameraPose colorFrame:(STColorFrame *)colorFrame depthFrame:(STDepthFrame *)depthFrameOrNil;
@end

//------------------------------------------------------------------------------
#pragma mark - STKeyFrameManager

/// Dictionary keys for [STKeyFrameManager initWithOptions:].
extern NSString* const kSTKeyFrameManagerMaxSizeKey;
extern NSString* const kSTKeyFrameManagerMaxDeltaRotationKey;
extern NSString* const kSTKeyFrameManagerMaxDeltaTranslationKey;

/// Automatically selects and manages keyframes.
@interface STKeyFrameManager : NSObject

/** Initialize a STKeyframeManager instance with the provided options.

@param options The options dictionary. The valid keys are:

- `kSTKeyFrameManagerMaxSizeKey`:
  - Specifies the maximal number of keyframes.
  - `NSNumber` integral value.
  - Defaults to `@(48)`.
- `kSTKeyFrameManagerMaxDeltaRotationKey`:
  - Sets the maximal rotation in radians tolerated between a frame and the previous keyframes before considering that it is a new keyframe.
  - `NSNumber` floating point value.
  - Defaults to `@(30.0 * M_PI / 180.0)` (30 degrees).
- `kSTKeyFrameManagerMaxDeltaTranslationKey`:
  - Sets the maximal translation in meters tolerated between a frame and the previous keyframes before considering that it is a new keyframe.
  - `NSNumber` floating poing value.
  - Defaults to `@(0.3)`.
*/
- (instancetype)initWithOptions:(NSDictionary *)options;

/** Check if the given pose would be a new keyframe.

@param colorCameraPose The camera pose for this keyFrame.

@return TRUE if the new pose exceeds the `kSTKeyFrameManagerMaxDeltaRotationKey` or `kSTKeyFrameManagerMaxDeltaTranslationKey` criteria.
*/
- (BOOL)wouldBeNewKeyframeWithColorCameraPose:(GLKMatrix4)colorCameraPose;

/** Potentially add the candidate frame as a keyframe if it meets the new keyframe criteria.

@param colorCameraPose The GLKMatrix4 camera pose for this keyFrame.
@param colorFrame The STColorFrame color buffer.
@param depthFrame The STDepthFrame depth frame.

@return TRUE if the frame was considered a new keyframe, FALSE otherwise.

@note Only `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange` is supported for sampleBuffer format.
*/
- (BOOL)processKeyFrameCandidateWithColorCameraPose:(GLKMatrix4)colorCameraPose
                                         colorFrame:(STColorFrame *)colorFrame
                                         depthFrame:(STDepthFrame *)depthFrame;

/** Manually add a keyFrame.

This will bypass the criteria of the manager.

@param keyFrame The STKeyFrame to add into STKeyFrameManager.
*/
- (void)addKeyFrame:(STKeyFrame *)keyFrame;

/// Get the array of STKeyFrame instances accumulated so far.
- (NSArray*)getKeyFrames;

/// Clear the current array of keyframes.
- (void)clear;

@end

//------------------------------------------------------------------------------
#pragma mark - STColorizer

/// Dictionary keys for [STColorizer newColorizeTaskWithMesh: scene: keyframes: completionHandler: options: error:].
extern NSString* const kSTColorizerTypeKey;
extern NSString* const kSTColorizerPrioritizeFirstFrameColorKey;
extern NSString* const kSTColorizerTargetNumberOfFacesKey;
extern NSString* const kSTColorizerQualityKey;

/** Constants specifying a colorization strategy for STColorizer.

See also:

- [STColorizer newColorizeTaskWithMesh:scene:keyframes:completionHandler:options:error:]
*/
typedef NS_ENUM(NSInteger, STColorizerType)
{
/// Generate a color for each vertex of the mesh. Best for small objects.
    STColorizerPerVertex = 0,

/** Generate a global texture map, and UV coordinates for each vertex of the mesh. Optimized for large rooms.
    __Note:__ Only 640x480 color images are supported by this colorizer.
*/
    STColorizerTextureMapForRoom = 1,

/// Generates a global texture map, and UV coordinates for each vertex of the mesh. Optimized for objects and people.
    STColorizerTextureMapForObject = 2,
};

/// Constants specifying the quality of STColorizer.
typedef NS_ENUM(NSInteger, STColorizerQuality)
{
/// Use this when speed is not a concern.
    STColorizerUltraHighQuality = 0,
    
/// Use this to balance between quality and speed.
    STColorizerHighQuality = 1,
    
/// Use this option when speed is a concern.
    STColorizerNormalQuality = 2,
};

/// Uses the color images from a vector of STKeyFrame to colorize an STMesh.
@interface STColorizer : NSObject

/** Returns a background task to colorize the given mesh using the provided keyframes.

@param mesh The STMesh to colorize.
@param scene The STScene context.
@param keyframes Array of keyframes, e.g. as provided by STKeyFrameManager.
@param completionHandler Block to execute once the task is finished or cancelled.
@param options The options dictionary. The valid keys are:

- `kSTColorizerTypeKey`:
  - Specifies the colorizing algorithm.
  - NSNumber integral value equal to one the STColorizerType constants.
  - Required.
- `kSTColorizerPrioritizeFirstFrameColorKey`:
  - Specifies whether the colorizer should give the first frame the highest priority.
  - `NSNumber` boolean value.
  - Defaults to `@NO`.
  - Particularly useful when we want to emphasize the appearance in the first frame, for example, when scanning a human face.
  - Only supported by `STColorizerTextureMapForObject`.
- `kSTColorizerTargetNumberOfFacesKey`:
  - Specifies a target number of faces for the final mesh.
  - NSNumber integral value.
  - Defaults to `@(20000)`.
  - Only supported by `STColorizerTextureMapForObject`.
- `kSTColorizerQualityKey`:
  - Specifies the desired speed/quality trade-off.
  - NSNumber integral value equal to one the STColorizerQuality constants.
  - Defaults to `STColorizerHighQuality`.
  - Only `STColorizerTextureMapForObject` will honor this option.

@param error will contain detailed information if the provided options are incorrect.
@note All the keyframes must have the same image size.
*/
+ (STBackgroundTask*)newColorizeTaskWithMesh:(STMesh *)mesh
                                       scene:(STScene*)scene
                                   keyframes:(NSArray *)keyframes
                           completionHandler:(void(^)(NSError *error))completionHandler
                                     options:(NSDictionary *)options
                                       error:(NSError* __autoreleasing *)error;

@end
