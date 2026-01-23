

**What it is**

- ROS2-native stack built by NVIDIA
    
- Internally uses:
    
    - CUDA
        
    - TensorRT
        
    - GXF (NVIDIA graph runtime)
        

**Free?**  
✅ Yes (open-source, NVIDIA GPU required)

**What inference can it do?**

- Stereo depth
    
- Visual odometry
    
- Object detection
    
- SLAM
    
- DNN inference


# What about Isaac ROS?

Isaac ROS:

- Assumes:
    
    - CUDA
        
    - TensorRT
        
    - Often Jetson or carefully matched x86 setups
        
- Is **excellent**, but:
    
    - Heavier
        
    - More opinionated
        
    - Less transparent
        

For **learning + debugging + visualization**, DeepStream is easier.