

Data from topic might be of type `h264`. In this case, we need to handle decoding process differently to when you simply use, say, jpeg as compression. Video decoding is not frame-independent. Thus, it is a little more complication.

`ros2 topic echo /front_stereo_camera/left/image_compressed`
```bash
header:
  stamp:
    sec: 1707938728
    nanosec: 36456552
  frame_id: front_stereo_camera_left_optical
format: h264
data:
- 0
- 0
```


# What needs to be done

```bash
ROS2 Node
  └── receives H.264 frames (bytes)
      └── pushes them into GStreamer
          └── decodes
              └── displays
```

## 1. Using CPU pipeline

```bash
ROS2 Executor (CPU)
   |
   |  H.264 byte stream
   v
GStreamer appsrc (user-space)
   |
   |  buffers
   v
h264parse → avdec_h264 → videoconvert → autovideosink

```

## Using GPU pipeline (zero copy)
GPU takes over from decoding step. And CPU will never see the image frame. Basically after coded bytes stream crosses the boundary between main memory and GPU's NVMM, GPU takes over processing, relieving of the CPU.


The steps below
```text
avdec_h264
videoconvert
autovideosink
```
is replaced by:
```text
nvh264dec
nvvidconv
nveglglessink
```

Everything else is the same.

Refer to GPU pipeline setup below.


> [!IMPORTANT] Zero-copy does **NOT** mean:
> “Data never leaves user space”
> 
> It means:
> 
> “Decoded frames never touch CPU RAM after decode”


```text
H264 bytes (CPU)
→ NVDEC hardware
→ GPU memory (NVMM)
→ EGL window
```

Only **compressed** data touches CPU memory.

That’s the win.

# Setup Guide for CPU pipeline

## Install packages
You need below packages:
```bash
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav

```

If some of them doesn't exist, check the following:

1) Architecture and apt server mismatch
	1) This happened to me once. I simply added a local mirror (Kaist) in `sudo software-properties-gtk` and it hooked me up to `ubuntu-ports` servers, which is mainly for ARM and other architectures, which is different to AMD64 architecture. This caused not finding some of gstreamer packages above.
	2) If this happened to you, you should change the server to main ubuntu server, or any other servers that are not for ports.

Here is  the problematic listing of apt servers:
```bash
cat /etc/apt/sources.list
# deb cdrom:[Ubuntu 22.04.5 LTS _Jammy Jellyfish_ - Release amd64 (20240911)]/ jammy main restricted

# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy main restricted
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy main restricted

## Major bug fix updates produced after the final release of the
## distribution.
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates main restricted

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
## team. Also, please note that software in universe WILL NOT receive any
## review or updates from the Ubuntu security team.
deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy universe
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy universe
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates universe

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu 
## team, and may not be under a free licence. Please satisfy yourself as to 
## your rights to use the software. Also, please note that software in 
## multiverse WILL NOT receive any review or updates from the Ubuntu
## security team.
deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy multiverse
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy multiverse
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates multiverse

## N.B. software from this repository may not have been tested as
## extensively as that contained in the main release, although it includes
## newer versions of some applications which may provide useful features.
## Also, please note that software in backports WILL NOT receive any review
## or updates from the Ubuntu security team.
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse

deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy-security main restricted
# deb-src http://security.ubuntu.com/ubuntu jammy-security main restricted
deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy-security universe
# deb-src http://security.ubuntu.com/ubuntu jammy-security universe
deb https://ftp.kaist.ac.kr/ubuntu-ports/ jammy-security multiverse
# deb-src http://security.ubuntu.com/ubuntu jammy-security multiverse

# This system was installed using small removable media
# (e.g. netinst, live or single CD). The matching "deb cdrom"
# entries were disabled at the end of the installation process.
# For information about how to configure apt package sources,
# see the sources.list(5) manual.
deb http://security.ubuntu.com/ubuntu/ jammy-security restricted main multiverse universe
```

Here is when it was fixed by using ubuntu main servers:
```bash
cat /etc/apt/sources.list
# deb cdrom:[Ubuntu 22.04.5 LTS _Jammy Jellyfish_ - Release amd64 (20240911)]/ jammy main restricted

# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://archive.ubuntu.com/ubuntu jammy main restricted
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy main restricted

## Major bug fix updates produced after the final release of the
## distribution.
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates main restricted

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
## team. Also, please note that software in universe WILL NOT receive any
## review or updates from the Ubuntu security team.
deb http://archive.ubuntu.com/ubuntu jammy universe
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy universe
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates universe

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu 
## team, and may not be under a free licence. Please satisfy yourself as to 
## your rights to use the software. Also, please note that software in 
## multiverse WILL NOT receive any review or updates from the Ubuntu
## security team.
deb http://archive.ubuntu.com/ubuntu jammy multiverse
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy multiverse
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-updates multiverse

## N.B. software from this repository may not have been tested as
## extensively as that contained in the main release, although it includes
## newer versions of some applications which may provide useful features.
## Also, please note that software in backports WILL NOT receive any review
## or updates from the Ubuntu security team.
# deb-src http://kr.archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu jammy-security main restricted
# deb-src http://security.ubuntu.com/ubuntu jammy-security main restricted
deb http://archive.ubuntu.com/ubuntu jammy-security universe
# deb-src http://security.ubuntu.com/ubuntu jammy-security universe
deb http://archive.ubuntu.com/ubuntu jammy-security multiverse
# deb-src http://security.ubuntu.com/ubuntu jammy-security multiverse

# This system was installed using small removable media
# (e.g. netinst, live or single CD). The matching "deb cdrom"
# entries were disabled at the end of the installation process.
# For information about how to configure apt package sources,
# see the sources.list(5) manual.
deb http://security.ubuntu.com/ubuntu/ jammy-security restricted main multiverse universe
```


After successfully installing the packages, the following should pass:

```bash
gst-inspect-1.0 h264parse
```

```bash
gst-inspect-1.0 avdec_h264
```


### Confirm 1
Before touching H.264, confirm your display pipeline is fine:
```bash
gst-launch-1.0 videotestsrc \
! videoconvert \
! autovideosink
```

This should show a small windows screen with some colors and stuff. If this works → **display + GStreamer core are correct**.


### Confirm 2
Generates a few seconds of test video
```bash
gst-launch-1.0 videotestsrc num-buffers=300 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 \
! video/x-h264,stream-format=byte-stream,alignment=au \
! h264parse config-interval=-1 \
! filesink location=sample.h264

Setting pipeline to PAUSED ...
Pipeline is PREROLLING ...
Redistribute latency...
Pipeline is PREROLLED ...
Setting pipeline to PLAYING ...
Redistribute latency...
New clock: GstSystemClock
Got EOS from element "pipeline0".
Execution ended after 0:00:00.547136728
Setting pipeline to NULL ...
Freeing pipeline ...
```

You should see a file a few MB in size.
```bash
ll -h
total 2.6M
drwxr-xr-x  2 ted ted 4.0K  1월 17 22:49 ./
drwxr-x--- 29 ted ted 4.0K  1월 17 22:43 ../
-rw-rw-r--  1 ted ted 2.6M  1월 17 22:51 sample.h264
```

🎉 **Video plays** 
```bash
gst-launch-1.0 filesrc location=sample.h264 \
! h264parse \
! avdec_h264 \
! videoconvert \
! autovideosink
Setting pipeline to PAUSED ...
Pipeline is PREROLLING ...
Got context from element 'autovideosink0': gst.gl.GLDisplay=context, gst.gl.GLDisplay=(GstGLDisplay)"\(GstGLDisplayX11\)\ gldisplayx11-0";
Redistribute latency...
Redistribute latency...
Pipeline is PREROLLED ...
Setting pipeline to PLAYING ...
Redistribute latency...
New clock: GstSystemClock
ERROR: from element /GstPipeline:pipeline0/GstAutoVideoSink:autovideosink0/GstGLImageSinkBin:autovideosink0-actual-sink-glimage/GstGLImageSink:sink: Quit requested
Additional debug info:
../ext/gl/gstglimagesink.c(1872): gst_glimage_sink_show_frame (): /GstPipeline:pipeline0/GstAutoVideoSink:autovideosink0/GstGLImageSinkBin:autovideosink0-actual-sink-glimage/GstGLImageSink:sink
Execution ended after 0:00:09.303448080
Setting pipeline to NULL ...
ERROR: from element /GstPipeline:pipeline0/GstH264Parse:h264parse0: Internal data stream error.
Additional debug info:
../libs/gst/base/gstbaseparse.c(3681): gst_base_parse_loop (): /GstPipeline:pipeline0/GstH264Parse:h264parse0:
streaming stopped, reason error (-5)
Freeing pipeline ...
```




## Python ROS2 Code (CPU pipeline)


> [!IMPORTANT] We're only using Python for CPU pipeline
> For GPU NVDEC zero-copy to work, you need to use C++
> 
> 
> 

Python has problems with:
- DMABUF
- zero-copy GPU memory
- DeepStream plugins
- high-FPS multi-camera

So you should use GPU for production level or PoC code. 

### Additional Installation (Python GI bindings)
```bash
sudo apt install python3-gi python3-gst-1.0
```

### ROS2 Node
`h264_viewer.py`
```python
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

  

import gi

gi.require_version('Gst', '1.0')

from gi.repository import Gst

  

Gst.init(None)

  

PIPELINE = (

'appsrc name=src is-live=true format=time '

'caps=video/x-h264,stream-format=byte-stream,alignment=au '

'! h264parse '

'! avdec_h264 '

'! videoconvert '

'! autovideosink sync=false'

)

  

class H264Viewer(Node):

def __init__(self):

super().__init__('h264_viewer')

  

self.pipeline = Gst.parse_launch(PIPELINE)

self.appsrc = self.pipeline.get_by_name('src')

self.pipeline.set_state(Gst.State.PLAYING)

  

self.start_time = self.get_clock().now()

  

self.sub = self.create_subscription(

CompressedImage,

'/front_stereo_camera/left/image_compressed',

self.on_frame,

10

)

  

self.get_logger().info('H264 viewer started')

  

def on_frame(self, msg: CompressedImage):

if msg.format != 'h264':

self.get_logger().warn(

f'Expected format=h264, got {msg.format}')

return

  

buf = Gst.Buffer.new_allocate(None, len(msg.data), None)

buf.fill(0, msg.data)

  

now = self.get_clock().now()

delta = now - self.start_time

  

buf.pts = delta.nanoseconds

buf.dts = buf.pts

buf.duration = int(1e9 / 30) # assume 30 FPS

  

self.appsrc.emit('push-buffer', buf)

  

def main():

rclpy.init()

node = H264Viewer()

rclpy.spin(node)

node.pipeline.set_state(Gst.State.NULL)

rclpy.shutdown()

  

if __name__ == '__main__':

main()
```

`package.xml`
```xml
<exec_depend>python3-gi</exec_depend>
<exec_depend>gstreamer1.0-tools</exec_depend>
<exec_depend>gstreamer1.0-plugins-base</exec_depend>
<exec_depend>gstreamer1.0-plugins-good</exec_depend>
<exec_depend>gstreamer1.0-plugins-bad</exec_depend>
<exec_depend>gstreamer1.0-plugins-ugly</exec_depend>
<exec_depend>gstreamer1.0-libav</exec_depend>

```

`setup.py`
```python
entry_points={

'console_scripts': [
'h264_viewer = image_processor.h264_viewer:main'
],

},
```

Done!
![[Pasted image 20260117234420.png]]

# C++ ROS2 Code (GPU pipeline)


> [!Warning] Python Limitation
> Python **cannot**:
> - import NVMM buffers
> - manage DMABUF
> - interop with CUDA
> - 
>   So you need to switch to C++ or DeepStream.

    


To be continued

https://chatgpt.com/c/696b357f-57bc-8322-ac45-34d247b3d8b8





# Latency Guideline

## H.264 pipeline Optimization
### Synchronization

This means: "Show frames according to timestamps, synced to system clock."
```
autovideosink sync=true
```

If timestamps drift or burst → GStreamer **waits**.

This is correct for video players  
❌ terrible for robotics / teleop.

Correct: "Display immediately when decoded."
This alone often cuts latency by **50–100 ms**.
```bash
autovideosink sync=false
```

### Frame Reordering
H.264 can reorder frames
Because of:
- B-frames
- reference frames

Decoders may buffer frames internally.


You can’t fully eliminate this unless:

- encoder disables B-frames
- low-latency encoder profile

But you **can** reduce downstream buffering.

### `appsrc` latency control

`appsrc` defaults are not low-latency

By default:
- appsrc buffers aggressively
- assumes file-like input

Fix it explicitly:
```bash
appsrc
  is-live=true  # disables preroll buffering
  block=false # drop instead of stall
  format=time
  do-timestamp=true # auto timestamps if missing
```


### Queue Elements

When pipelines grow, you’ll add:
```bash
queue max-size-buffers=1 leaky=downstream
```

This means "Never buffer more than one frame — drop old ones."

This is **standard robotics practice**.


## ROS2-side Latency Tuning

### Use SensorData QoS
- drops old frames
- never backlogs

```cpp
rclcpp::SensorDataQoS()
```

```python
qos = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1
)
```



# (TODO) Measuring Latency

## 8. Measuring latency (don’t guess)

### Quick visual test

- Move camera fast
    
- Observe delay
    

### Proper way (later)

- embed timestamp in overlay
    
- compare to system clock
    

We’ll do this once NVDEC is active.