
### **DeepStream**

**What it is**

- A **video analytics framework** built on:
    
    - GStreamer
        
    - NVDEC / NVENC
        
    - TensorRT
        
- Designed for **video → inference → metadata** pipelines
    

**Free?**  
✅ Yes (SDK download, NVIDIA GPU required)

**What inference can it do?**

- Object detection
    
- Tracking
    
- Classification
    
- Segmentation
    
- Multi-camera batching
    

**YOLO support?**  
✅ YES — very common  
YOLOv5 / YOLOv8 / custom YOLO models are widely used in DeepStream.

DeepStream gives you:

`H.264 → GPU decode → YOLO (TensorRT) → bounding boxes → metadata`

No CPU frames involved.


## Install NVIDIA DeepStream ✅ (recommended)

### What you get

- NVIDIA GStreamer plugins
    
- NVDEC / NVENC
    
- EGL sinks
    
- TensorRT inference plugins
    
- Example apps
    
- Documentation
    

### Cost

- Free
    
- Requires NVIDIA GPU + driver
    

### Works for:

- Visualization
    
- YOLO inference
    
- ROS2 integration
    
- R2B dataset playback
    

### Industry reality

> **This is what most NVIDIA-based robotics teams actually install first.**