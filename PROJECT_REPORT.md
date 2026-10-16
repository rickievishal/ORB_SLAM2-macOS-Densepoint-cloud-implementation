# ORB-SLAM3 macOS Webcam and Dense Mapping Report

## 1. Project overview

This project is an Apple-Silicon/macOS build of **ORB-SLAM3** with a webcam
example and an experimental dense point-cloud path. The GitHub destination
repository name contains “ORB_SLAM2”, but the code in this workspace is based
on ORB-SLAM3: its core library, Atlas, tracking, local mapping, loop closing,
and monocular example APIs are ORB-SLAM3.

The webcam application has two independent display windows:

1. **ORB-SLAM3: Map Viewer** — Pangolin displays the current camera pose,
   keyframes, sparse ORB map points, and estimated dense colored points.
2. **ORB-SLAM3: Current Frame** — OpenCV displays the latest tracked frame
   with the tracking overlays.

The implementation is split across:

- [`CMakeLists.txt`](./CMakeLists.txt) — core library, examples, dependencies,
  platform-specific OpenGL linkage, and the `mono_webcam` target.
- [`Examples/Monocular/mono_webcam.cc`](./Examples/Monocular/mono_webcam.cc) —
  camera acquisition, ORB-SLAM3 calls, dense-mapper integration, viewer
  lifecycle, and output files.
- [`Examples/Monocular/DenseMapper.h`](./Examples/Monocular/DenseMapper.h)
  and [`Examples/Monocular/DenseMapper.cc`](./Examples/Monocular/DenseMapper.cc)
  — temporal two-view depth estimation and point back-projection.
- [`Examples/Monocular/MacWebcam.yaml`](./Examples/Monocular/MacWebcam.yaml)
  — camera model, image size, ORB parameters, and map-view settings.
- [`src/Tracking.cc`](./src/Tracking.cc) — ORB-SLAM3 tracking and monocular
  two-view initialization.
- [`src/FrameDrawer.cc`](./src/FrameDrawer.cc) — tracked-frame image and
  feature-overlay generation.
- [`src/Viewer.cc`](./src/Viewer.cc) — main-thread Pangolin map window and
  OpenCV camera-frame window.
- [`src/MapDrawer.cc`](./src/MapDrawer.cc) — sparse and dense map rendering,
  thread-safe dense-point storage, and PLY export.
- [`src/System.cc`](./src/System.cc) and
  [`include/System.h`](./include/System.h) — public SLAM API forwarding,
  viewer entry point, current-map access, and dense-map operations.

## 2. Build structure and platform-specific setup

The top-level CMake project builds an `ORB_SLAM3` shared library from the
tracking, Atlas/map, optimization, camera-model, and viewer sources. It also
defines the standard dataset examples and the custom `mono_webcam` executable.
The webcam executable links against the core library and compiles
`DenseMapper.cc` as an additional source file.

Third-party packages are declared in [`vcpkg.json`](./vcpkg.json): Boost
serialization, Eigen, OpenCV, OpenSSL, and Pangolin. DBoW2, g2o, and Sophus
are included under `Thirdparty/`. CMake's project-options/vcpkg setup
bootstraps dependency resolution as part of project configuration.

### macOS OpenGL linkage

On this machine, Pangolin's OpenGL target had a reference to the XQuartz
OpenGL library even though its window used macOS Cocoa. That combination left
Pangolin without the working context needed to create and update the image
texture. The Apple-specific block in [`CMakeLists.txt`](./CMakeLists.txt)
removes the XQuartz `libGL` entry from Pangolin's link interface and links
Apple's OpenGL framework instead. The verified runtime reported an OpenGL
context backed by `Metal` on the Apple M4.

Pangolin's window must be created and serviced on the macOS main thread in
this fork. For that reason, the webcam/tracking loop runs on a worker thread,
while `System::RunViewer()` runs in `main()` and owns the Pangolin event loop.
The OpenCV frame window is also created, updated, and destroyed on that viewer
thread, rather than from the camera worker.

## 3. Runtime data flow

The application processes each webcam frame through this path:

```text
AVFoundation camera
    -> cv::VideoCapture::read
    -> ORB_SLAM3::System::TrackMonocular
    -> Tracking and FrameDrawer::Update
    -> FrameDrawer::DrawFrame
    -> OpenCV current-frame window

Successful tracked frame
    -> DenseMapper::Process
    -> rectify reference/current images using relative SLAM pose
    -> StereoSGBM disparity
    -> reproject disparity to 3D
    -> transform points into the SLAM world coordinates
    -> MapDrawer dense-point buffer
    -> Pangolin map window and PLY output on shutdown
```

### Webcam capture and ORB-SLAM3 tracking

[`mono_webcam.cc`](./Examples/Monocular/mono_webcam.cc) opens camera index 0
through `cv::CAP_AVFOUNDATION`, requests 640×480 frames, and supplies a
monotonic timestamp to `TrackMonocular()`. The YAML declares OpenCV's
three-channel camera input as BGR (`Camera.RGB: 0`); ORB-SLAM3 converts image
data internally as needed for feature extraction.

ORB-SLAM3 extracts ORB features, tracks them between frames, estimates the
camera pose, and updates its sparse map. The two-window viewer obtains the
draw-ready image from `FrameDrawer::DrawFrame()`, which copies the last
tracking image and draws initialization correspondences or tracked-map
feature markers and status text. The image remains in OpenCV BGR format for
`cv::imshow`.

### Monocular map initialization

Monocular initialization is not equivalent to “enough matches were found.”
`Tracking::MonocularInitialization()` first chooses a reference image with
sufficient features, finds correspondences in a later image, and asks the
camera model to reconstruct from the two views. If reconstruction succeeds,
it triangulates points, makes initial keyframes and map points, runs bundle
adjustment, checks the resulting scene depth/tracked points, and normalizes
the initial map scale. If image motion, calibration, feature matches, or
triangulation are inadequate, initialization may fail and be attempted again.

The webcam YAML currently uses approximate pinhole intrinsics (`fx=fy=500`,
`cx=320`, `cy=240`) and zero distortion. This is intentionally safer than
using distortion values from a different camera, but it is not a substitute
for calibration. Incorrect focal length, principal point, or lens distortion
deforms the estimated rays and can reduce initialization and map accuracy.
For best results, calibrate the built-in camera at the exact resolution and
focus configuration used by the application.

## 4. How the dense mapping works

### Important distinction from the RGB-D reference method

The referenced
[ORB-SLAM3-with-dense-pointcloud-reconstruction repository](https://github.com/Rich-King395/ORB-SLAM3-with-dense-pointcloud-reconstruction)
builds dense clouds from RGB-D depth images (and has stereo-specific
operation). Its RGB-D implementation samples the color/depth images, uses
camera calibration to back-project measured depth into 3D, colors each point
from the corresponding RGB pixel, and transforms the camera-local cloud by
the keyframe pose.

This laptop webcam supplies only a single color image per capture. It has no
depth sensor and no second, calibrated, synchronized camera. The local
implementation therefore estimates depth from **two different times from
the same camera**. It uses OpenCV's stereo-rectification and StereoSGBM tools
on those two temporal views. It does **not** use Depth Anything or another
learned monocular-depth model, and it does not receive measured depth.

### Pair selection and relative pose

`DenseMapper` retains a reference image and its world-to-camera pose `Tcw`.
While tracking is `OK`, it waits at least ten processed frames and for the
relative translation between the reference and current poses to exceed
`0.04` in ORB-SLAM's coordinate units. This is a heuristic to avoid matching
nearly identical images; the threshold is not a physical 4-centimeter
baseline because monocular SLAM has arbitrary scale.

The relative transform is calculated as:

```text
Tcurrent_from_reference = Tcw_current * inverse(Tcw_reference)
```

Using this relative rotation and translation, the mapper calls
`cv::stereoRectify()` and creates rectification maps for both images.
`StereoSGBM` then computes a disparity image from the rectified grayscale
pair. The current implementation searches a configured disparity range of
128 values, uses a 5-pixel block, and evaluates every other image pixel
during point generation.

### Disparity to colored world points

Disparity is converted from OpenCV's fixed-point representation to floating
point and passed to `cv::reprojectImageTo3D()` with the rectification matrix
`Q`. Each accepted result must be finite, have a non-negligible disparity,
and have a positive depth within the implementation's 0.1-to-30-unit range.
The point is rotated out of rectified camera coordinates and transformed by
the inverse reference pose (`Twc_reference`) into the current SLAM world
coordinate frame. Its color is sampled from the reference image.

`MapDrawer` stores positions and colors behind a mutex, renders them as
colored OpenGL points, and caps the in-memory viewer/export cloud at 500,000
points. `SaveDenseMapPointsPLY()` writes an ASCII PLY containing XYZ and RGB
properties. A successful live run produced a PLY with 500,000 vertices.

### Map resets and shutdown

If ORB-SLAM3 creates a new active map after tracking loss or failed tracking,
the new map has a different origin and coordinate frame. Combining dense
points from the prior map with points from the new map would create a
misaligned cloud. The webcam worker checks the active map ID after every
tracking call; when it changes, it resets the dense mapper's reference and
clears the stored dense points. It also clears the reference whenever
tracking is not `OK`.

Closing either display window ends the viewer loop. The main function then
stops the camera worker, joins it, calls `SLAM.Shutdown()`, saves the TUM
keyframe trajectory, exports the dense PLY if any points exist, and releases
the camera. The window closure and trajectory/PLY export were exercised in
the live macOS run.

## 5. Accuracy and limitations

This webcam implementation is a useful **approximate depth visualization**,
not a calibrated RGB-D reconstruction system. Its limitations follow from
the sensing method:

- **Monocular scale is unobservable.** The SLAM map has arbitrary scale unless
  a known distance, calibrated stereo baseline, inertial constraints, or
  another metric source is introduced. Point coordinates and the
  `0.04`-unit baseline test must not be interpreted as meters.
- **Temporal stereo assumes a static scene.** Moving objects, people, changing
  lighting, auto-exposure, motion blur, and rolling-shutter distortion break
  correspondence between images captured at different times.
- **Motion direction matters.** StereoSGBM works best when rectification
  produces useful horizontal disparity. Rotation-dominated motion or
  insufficient translation yields weak or incorrect depth.
- **Calibration affects geometry.** Approximate intrinsics and zero
  distortion can visibly bend or scale the reconstructed geometry.
- **Dense points are not globally fused.** This implementation appends
  samples from successful pairs; it does not perform voxel fusion, surface
  reconstruction, robust multi-view consistency filtering, or reintegration
  after later bundle-adjustment/loop-closure pose corrections. Therefore
  overlap, ghosting, and drift can appear.
- **A reset intentionally discards the previous dense cloud.** This prevents
  incompatible map frames from being blended, but it also means the saved
  cloud covers the current map only, not a stitched history of every reset.
- **Point density is not accuracy.** Generating hundreds of thousands of
  samples creates a visually dense cloud but does not mean each estimated
  depth is correct.

For geometry closer to the reference repository's result, use synchronized,
calibrated stereo cameras or an RGB-D sensor and feed its real depth into a
keyframe-based back-projection/fusion pipeline. A learned monocular-depth
model could be an alternative source of relative depth, but its predictions
also need scale calibration and careful alignment with SLAM poses.

## 6. Build and run

From the repository root:

```bash
cmake --build build --target mono_webcam -j2
./Examples/Monocular/mono_webcam \
  ./Vocabulary/ORBvoc.txt \
  ./Examples/Monocular/MacWebcam.yaml
```

The build target has been verified on the Apple-Silicon macOS workspace. A
live webcam run also verified OpenGL context creation, two separate visible
windows, successful monocular initialization, dense point generation, dense
point clearing/rebuilding after a map reset, clean camera-thread shutdown,
and non-empty trajectory and PLY files. Results still depend on sufficient
scene texture, useful camera movement, lighting, and camera calibration.

Generated files are written in the process's current working directory:

- `KeyFrameTrajectory_webcam.txt` — TUM-format keyframe timestamp and pose.
- `DensePointCloud_webcam.ply` — colored point cloud from the active dense
  map buffer.

These generated outputs are local artifacts and are not source files.

## 7. Git checkpoint

The implementation and two-window viewer checkpoint was committed as
`31f888a` (“Checkpoint webcam dense mapping and split viewer windows”).
The map-reset dense-cloud correction was committed as `cb7f53a` (“Clear dense
cloud when SLAM map resets”). Both commits were pushed to the configured
`personal` remote:

<https://github.com/rickievishal/ORB_SLAM2-macOS-Densepoint-cloud-implementation>
