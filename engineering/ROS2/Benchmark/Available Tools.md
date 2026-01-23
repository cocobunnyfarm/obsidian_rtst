
# Node Graph Performance: `ros2_benchmark`
[ros2_benchmark](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark)



# Raw Transport Speed: `performance_test`

> [!NOTE] Shared Memory vs. UDP
> Use this mainly for this purpose

[Performance Test](https://docs.ros.org/en/humble/p/performance_test/)
[ROS2 package info](https://index.ros.org/p/performance_test/)

# Finding Bottlenecks: `ros2_tracing`
[ROS 2 Tracing Repository](https://github.com/ros2/ros2_tracing)


# Other tools
https://developer.nvidia.com/nsight-systems


| Tool                 | Purpose                | Primary Output                      | Use Case                                                                                                         |
| -------------------- | ---------------------- | ----------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **`ros2_benchmark`** | **End-to-End Metrics** | FPS, Latency (ms), CPU/GPU %        | To see _if_ your system is meeting requirements (e.g., "Am I hitting 30 FPS?").                                  |
| **`ros2_tracing`**   | **Execution Logic**    | Callback durations, Executor timing | To find _where_ in the ROS 2 graph logic a delay is happening (e.g., "Is a specific node's callback blocking?"). |
| **`Nsight Systems`** | **System Bottlenecks** | Timeline of CPU/GPU/Memory/I/O      | To find _why_ the hardware is slow (e.g., "Is the GPU waiting for data from the CPU?").                          |

- **For simple verification:** `ros2_benchmark` is enough.
- **For deep optimization:** Using all three is essential. **NVIDIA Isaac ROS** actually integrates these tools; for example, `ros2_benchmark` has flags like `enable_nsys:=true` to automatically generate a profile during a benchmark run so you can correlate FPS drops with hardware activity. 

**Note on Humble:** Since you are using Humble, `ros2_tracing` is already integrated into the core stack (rclcpp/rcl), making it very low-overhead (~0.0033ms delay) and safe for production-level tests.