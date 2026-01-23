
### GStreamer properties (most important)

You can control `nveglglessink` using properties:

`nveglglessink sync=false fullscreen=true`

Common properties:

| Property          | Meaning                  |
| ----------------- | ------------------------ |
| `sync=false`      | Don’t wait on timestamps |
| `fullscreen=true` | Fullscreen               |
| `qos=false`       | Disable frame dropping   |
| `max-lateness`    | Control latency          |

## End-to-end visualization pipeline (ROS2 → GPU → screen)

### Conceptual flow

```
ROS2 CompressedImage
↓
msg.data (H.264 bytes)
↓
Gst.Buffer
↓
appsrc
↓
nvh264dec (NVDEC)
↓
nveglglessink
↓
Screen

```

ROS never sees decoded images.


# Dependency Resolutions

## Step 1 — Confirm OS & GPU

Run:

`lsb_release -a nvidia-smi`

Tell me:

- Ubuntu version
    
- NVIDIA driver version
    
- GPU model (RTX 5070 Ti is fine)
  
  ``





**Install NVIDIA DeepStream first.**

Why?

- It **guarantees** you get:
    
    - `nvh264dec`
        
    - `nveglglessink`
        
    - `nvvidconv`
        
    - NVMM support
        
- It works on **RTX desktop GPUs**
    
- It avoids distro-specific plugin hell
    
- You _will_ need it later anyway if you want YOLO inference
    

After that:

- You can still use **plain GStreamer**
    
- You can still write **your own ROS2 + appsrc pipeline**
    
- You are not “locking yourself into DeepStream”
    

Think of DeepStream as:

> “A known-good NVIDIA GStreamer distribution + inference plugins”

Test these. If none of them exists, you need to install

```bash
gst-inspect-1.0 h264parse
gst-inspect-1.0 nvh264dec
gst-inspect-1.0 nveglglessink
```

After installing DeepStream:

`gst-inspect-1.0 nvh264dec gst-inspect-1.0 nveglglessink`

✅ will work

You can still write:

`appsrc ! nvh264dec ! nveglglessink`





If there is an error about `h264parse`, then you should install some apt packages.
```
gst-launch-1.0 filesrc location=sample.h264 \
! h264parse ! nvh264dec ! nvvidconv \
! 'video/x-raw(memory:NVMM),format=NV12' \
! nveglglessink
WARNING: erroneous pipeline: no element "h264parse"
```

or you could just run:
```bash
gst-inspect-1.0 h264parse
No such element or plugin 'h264parse'
```
### What it means

GStreamer cannot find the `h264parse` plugin.

### Why

You are missing **gst-plugins-bad / good / ugly** packages.

```
sudo apt update
sudo apt install \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly

```