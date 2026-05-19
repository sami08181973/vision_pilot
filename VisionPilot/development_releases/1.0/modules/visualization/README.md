# VISUALIZATION MODULE

## Acknowledgement

I would like to thank [Ethan](https://dev.to/ethand91) and his blog post of [Streaming Camera with C++ WebRTC GStreamer](https://dev.to/ethand91/streaming-camera-with-c-webrtc-gstreamer-pof).

Your implementation was truly helpful and inspiring for me to complete this module.


## I. Overview

The WebRTC Visualization Module provides a real-time video streaming capability for the VisionPilot pipeline via WebRTC protocol. It serves the following core functions:

1. **Real-time frame capture and encoding** which accepts OpenCV `cv::Mat` frames and encodes them to VP8 video codec via GStreamer.
2. **WebRTC peer-to-peer streaming** which establishes a WebRTC peer connection between the server (VisionPilot app) and browser clients, enabling live video delivery over the internet or LAN.
3. **Lightweight browser client** which serves a minimal, self-contained HTML5 page with built-in WebRTC JavaScript client without external dependencies required for the browser.
4. Implements WebSocket-based signaling for SDP (Session Description Protocol) offer/answer negotiation and ICE (Interactive Connectivity Establishment) candidate exchange.
5. **Thread-safe frame streaming** that manages concurrent frame pushes from the main application thread while running a GStreamer pipeline and event loop in separate threads.

This module is essential for downstream remote monitoring, debugging, and visualization of autonomous driving pipelines during development and testing phases.


## II. Architecture && Module structure

### 1. Architecture

```

┌─────────────────────────────────────────────────────────────────┐
│                    VisionPilot Application                      │
│                  (vision_pilot.cpp main thread)                 │
│                                                                 │
│   DATA CAPTURE                                                  │
│   ┌──────────────────────┐                                      │
│   │  V4L2/ROS2 Camera    │                                      │
│   │      Source          │                                      │
│   └──────────┬───────────┘                                      │
│              │                                                  │
│              │ cv::Mat frames (33ms loop)                       │
│              │                                                  │
│              ▼                                                  │
│   ┌──────────────────────────────────────────┐                  │
│   │  (Various upstream modules, like         │                  │
│   │  model inference, processing, calc etc.) │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
│              │ Frames & longitudinal/lateral planning results   │
│              │                                                  │
│              ▼                                                  │
│   VISUALIZATION                                                 │
│   ┌──────────────────────────────────────────┐                  │
│   │  visualization::render_frame()           │                  │
│   │  (draw frame + planning results)         │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
│              ▼                                                  │
│   ┌──────────────────────────────────────────┐                  │
│   │  WebRTCStreamer::push_frame()            │                  │
│   │  (stream to endpoint via WebRTC)         │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
└──────────────┼──────────────────────────────────────────────────┘
               │
               │ BGR frames + metadata
               │
    ┌──────────▼───────────────────────────────────────────┐
    │         WebRTCStreamer::Impl (Internal)              │
    │                                                      │
    │  ┌────────────────────────────────────────────────┐  │
    │  │  GStreamer Pipeline (separate thread)          │  │
    │  │                                                │  │
    │  │  appsrc => queue => videoconvert => vp8enc =>  │  │
    │  │  rtpvp8pay => webrtcbin                        │  │
    │  │                                                │  │
    │  │  ┌──────────────────────────────────────────┐  │  │
    │  │  │ WebRTC peer connection (GStreamer)       │  │  │
    │  │  │  - Manages media stream                  │  │  │
    │  │  │  - Generates SDP offers                  │  │  │
    │  │  │  - Gathers ICE candidates                │  │  │
    │  │  └──────────────────────────────────────────┘  │  │
    │  └────────────────────────────────────────────────┘  │
    │                                                      │
    │  ┌────────────────────────────────────────────────┐  │
    │  │  Signaling Layer (SoupServer + WebSocket)      │  │
    │  │                                                │  │
    │  │  HTTP handler:                                 │  │
    │  │    GET / => serves kBrowserHtml                │  │
    │  │                                                │  │
    │  │  WebSocket handler:                            │  │
    │  │    - Receives: SDP answer, ICE candidates      │  │
    │  │    - Sends: SDP offer, ICE candidates          │  │
    │  │    - Queue + flush mechanism for ordering      │  │
    │  │                                                │  │
    │  └────────────────────────────────────────────────┘  │
    │                                                      │
    └───────────────┬──────────────────────────────────────┘
                    │
        ┌───────────┴──────────────┐
        │                          │
        ▼                          ▼
   ┌─────────────────┐        ┌──────────────────┐
   │  Browser Client │        │  Network         │
   │  (HTML5 + JS)   │ <====> │  (Internet/LAN)  │
   │                 │        └──────────────────┘
   │ ┌─────────────┐ │        
   │ │ RTCPeerConn │ │
   │ │ (signaling) │ │
   │ ├─────────────┤ │
   │ │ WebSocket   │ │
   │ │ (SDP/ICE)   │ │
   │ ├─────────────┤ │
   │ │ <video>     │ │
   │ │ (playback)  │ │
   │ └─────────────┘ │
   └─────────────────┘

```

### 2. Flow summary

1. `VisionPilot` application calls `webrtc_streamer->push_frame(cv::Mat)` in its main loop. This frame is generated from `visualization::render_frame()`.
2. Frame is validated, converted to BGR, and pushed to the GStreamer pipeline's `appsrc` element.
3. GStreamer encodes the frame using VP8 codec and feeds it to the `webrtcbin` element.
4. On the first frame, `webrtcbin` triggers `on-negotiation-needed`, which creates an SDP offer.
5. SDP offer is queued and sent to the browser client via WebSocket.
6. Browser responds with SDP answer and ICE candidates.
7. Server receives answer, sets remote description, and flushes any pending ICE candidates.
8. Media stream begins flowing from server to browser via the established peer connection.

### 3. Module structure

```

visualization/
├── CMakeLists.txt
├── README.md (this file)
├── include/
│   └── visualization/
│       ├── visualization.hpp           (visualization header)
│       └── visualization_to_webrtc.hpp (WebRTC header)
└── src/
    ├── visualization.cpp               (visualization drawing, OpenCV window management)
    └── visualization_to_webrtc.cpp     (WebRTC implementation)

```


## III. Build

### 1. Prerequisites

- `ROS2 Humble` (tested on Ubuntu 22.04)
    - `source /opt/ros/humble/setup.bash`
- `GStreamer` development libraries:
    - `libgstreamer1.0-dev`
    - `libgstreamer-plugins-base1.0-dev`
    - `libgstreamer-plugins-bad1.0-dev`
- `libsoup 2.4` (HTTP/WebSocket server):
    - `libsoup2.4-dev`
- `JSON-GLib` (JSON signaling message handling):
    - `libjson-glib-dev`
- `OpenCV`:
  - `libopencv-dev`
- `Standard build tools`:
  - `build-essential`, `cmake` (≥3.22.1), `pkg-config`

Install all at once:

```bash

sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libsoup2.4-dev libjson-glib-dev

```

### 2. Steps

```bash
# 1. Navigate to workspace root
cd /path/to/VisionPilot/development_releases/1.0

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. Build (from workspace root; CMake will configure all modules)
mkdir -p build && cd build
cmake .. -DONNXRUNTIME_ROOT=$your_ONNXRUNTIME_path
make -j$(nproc)

```

### 3. Expected Output

```bash
[ 83%] Built target visualization
[ 89%] Building CXX object app/CMakeFiles/VisionPilot.dir/vision_pilot.cpp.o
[ 97%] Linking CXX executable ../VisionPilot
[100%] Built target VisionPilot
```

Binary location: `build/VisionPilot`


## IV. Test/demo

### 1. Running with WebRTC enabled

This demo shall guide you through testing this WebRTC streaming with a V4L2 mount, streamed 
via a combination of the `v4l2loopback` kernel module and FFmpeg.

With this demo, you will:
1. Publish a V4L2 video streaming mount from a local video.
2. Use `VisionPilot` application to subscribe to that streaming mount, process and stream frames to a local host.

```bash

# 1. Navigate to build directory
cd /path/to/VisionPilot/development_releases/1.0/build


# 2. Initiate V4L2 streaming mount

# a. Install package
sudo apt update
sudo apt install ffmpeg -y
sudo apt install v4l2loopback-dkms -y

# b. Load the module (assuming you gonna stream it at `/dev/video9`)
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=9 card_label="Virtual Camera" exclusive_caps=1

# c. Publish looping video at that mount
ffmpeg -re -stream_loop -1 -i <absolute path to local video> -f v4l2 -pix_fmt yuv420p /dev/video9


# 3. Kickstart VisionPilot app with V4L2 subscription to that mount, and stream frames to http://127.0.0.1:8080/
./VisionPilot 1 /dev/video0 10 1 8080
```

**Arguments:**

- `1`: V4L2 mode (use `0` for ROS2 mode)
- `/dev/video0`: V4L2 device path (if ROS2 mode, this second arg will be ROS2 topic name)
- `10`: target FPS
- `1`: enable WebRTC (use `0` to disable)
- `8080`: WebRTC server port (not available if WebRTC is disabled)

**Expected terminal output:**

```bash

Starting in V4L2 mode with device: /dev/video9 and FPS: 10
[V4L2Reader INFO] Initializing V4L2 Reader
[V4L2Reader INFO]   Device Path: /dev/video9
[V4L2Reader INFO]   Target FPS: 10
[V4L2Reader INFO] V4L2 device configured successfully
[V4L2Reader INFO]   Received resolution: 2560x1440
[V4L2Reader INFO]   Received FPS: 10.000000
Starting WebRTC streamer on port: 8080
[WebRTCStreamer] soup_server created
[WebRTCStreamer] soup_server listening on port 8080 and handlers installed
[WebRTCStreamer] pipeline created, appsrc=0x57add3f02e20 webrtc=0x57add3f34110
[WebRTCStreamer] pipeline set to PLAYING
Open browser at: http://127.0.0.1:8080/
Local OpenCV preview is disabled while WebRTC is enabled.

```

### 2. Accessing the stream

1. Open a web browser and navigate to: `http://127.0.0.1:8080/`
2. The minimal HTML client page will load.
3. WebRTC negotiation will begin automatically.
4. Once offer/answer exchange completes, the video stream should appear in the `<video>` element.

### 3. Troubleshooting

These are known/enountered bugs and errors. If you encounter a completely new one, try posting it as new issue
at [Autoware VisionPilot repository](https://github.com/autowarefoundation/autoware_vision_pilot).

1. **Black/blank video**: frame may not be arriving from the camera. Test with WebRTC disabled:
  
    ```bash
    
    ./VisionPilot 1 /dev/video0 10 0
    
    ```
    
    If OpenCV preview works, the camera is fine, then the issue might be somewhat WebRTC-specific.
  
2. **Connection refused**: ensure port `8080` is not in use:
  
    ```bash
    
    lsof -i :8080
    
    ```


