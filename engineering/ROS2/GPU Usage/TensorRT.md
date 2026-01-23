

**What it is**

- NVIDIA’s low-level **inference engine**
    
- Runs neural networks on NVIDIA GPUs efficiently
    
- Optimizes models (FP16, INT8, layer fusion, etc.)
    

**Free?**  
✅ Yes (requires NVIDIA GPU)

**What inference can it do?**

- Any model you can convert to TensorRT
    
- YOLO, ResNet, segmentation, depth, pose, etc.
    

**Can it replace YOLO?**  
❌ No — YOLO is a _model_  
✅ TensorRT _runs_ YOLO

Think of TensorRT as:

> “The engine that runs the model fast on GPU”