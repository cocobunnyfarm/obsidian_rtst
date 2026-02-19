

# 설정 방법 

> [!Important] Limitations
> First, the data types to be exchanged need to have a fixed size. This precludes the use of strings and sequences at any level in the data type, though this does not prevent the use of arrays, as their size is fixed at compile time. If any of these types of member variables are encountered in the IDL code generating the data types, shared memory exchange is disabled.
https://cyclonedds.io/docs/cyclonedds/0.9.1/shared_memory.html#limitations




## QoS (For Shared memory)

> Only a subset of QoS settings supports Shared Memory. Those are:
> 
> 1. Liveliness: Automatic
> 2. Deadline: Infinity (0)
> 3. Reliability: Reliable or Best Effort
> 4. Durability: Volatile or Transient Local
> 5. History: Keep Last
> 
> The Keep Last history depth of a writer cannot be larger than the maximum capacity allowed by an iceoryx publisher (currently 16). Otherwise the network stack is implictly used.
> 
> The ROS 2 default settings Reliable, Volatile and Keep Last(10) are supported and applicable to a wide range of applications.
https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md
https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/advanced/configuration-guide.md


### SensorDataQoS
> Sensor Data [QoS](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1QoS.html#classrclcpp_1_1QoS) class
> - History: Keep last,
> - Depth: 5,
> - Reliability: Best effort,
> - Durability: Volatile,
> - Deadline: Default,
> - Lifespan: Default,
> - Liveliness: System default,
> - Liveliness lease duration: default,
> - avoid ros namespace conventions: false
https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html?utm_source=chatgpt.com


### QoS Policy Config 서로 다른 2개의 설명
### QoS settings (cyclonedds humble 문서)
Only a subset of QoS settings supports Shared Memory. Those are:

1. Liveliness: Automatic
2. Deadline: Infinity (0)
3. Reliability: Reliable or Best Effort
4. Durability: Volatile or Transient Local
5. History: Keep Last

The Keep Last history depth of a writer cannot be larger than the maximum capacity allowed by an iceoryx publisher (currently 16). Otherwise the network stack is implictly used.

The ROS 2 default settings Reliable, Volatile and Keep Last(10) are supported and applicable to a wide range of applications.
https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md

### QoS settings (Eclipse CycloneDDS 0.9.1 Documentation)
Second, the manner in which the iceoryx memory pool keeps track of exchanged data puts a number of limitations on the QoS settings. For writers, the following QoS settings are prerequisites for shared memory exchange:

- Liveliness
    - DDS_LIVELINESS_AUTOMATIC
- Deadline
    - DDS_INFINITY
- Reliability
    - DDS_RELIABILITY_RELIABLE
- Durability
    - DDS_DURABILITY_VOLATILE
- History
    - DDS_HISTORY_KEEP_LAST
    - with depth no larger than the publisher history capacity as set in the configuration file

Whereas for readers, the following QoS settings are prerequisites for shared memory exchange:
- Liveliness
    - DDS_LIVELINESS_AUTOMATIC
- Deadline
    - DDS_INFINITY
- Reliability
    - DDS_RELIABILITY_RELIABLE
- Durability
    - DDS_DURABILITY_VOLATILE
- History
    - DDS_HISTORY_KEEP_LAST
    - with depth no larger than the subscriber history request as set in the configuration file


### 두 문서의 차이점
가장 큰 차이는 Reliability다. CycloneDDS쪽은 Reliable을 강제하고 있음. Humble쪽은 Best Effort 허용.

ROS2 Humble에서 CycloneDDS SHM 통합이 완화되었습니다 (라고 함.. 근데 공식문서 확인 필요)

아무튼 Humble쪽 스탠다드를 따라가기로 함. 

| 항목          | 현재 설정         | SHM 가능?       |
| ----------- | ------------- | ------------- |
| Liveliness  | Automatic     | ✅             |
| Deadline    | Infinity      | ✅             |
| Reliability | Best Effort   | ✅ (Humble 허용) |
| Durability  | Volatile      | ✅             |
| History     | Keep Last(10) | ✅             |

### 결론

즉, SensorDataQoS 디폴트값은 거의 비슷해서 문제가 없을 수 있으나, Default값 및 System default 값에 의해 Shared Memory 요구조건과 달리질 수 있다. 따라서 그냥 explicit하게 설정하는 게 좋다.

| 항목          | SensorDataQoS 기본값 | Shared Memory 요구조건          | OK? |
| ----------- | ----------------- | --------------------------- | --- |
| History     | Keep Last         | Keep Last                   | ✅   |
| Depth       | 5                 | ≤ 16                        | ✅   |
| Reliability | Best Effort       | Reliable or Best Effort     | ✅   |
| Durability  | Volatile          | Volatile or Transient Local | ✅   |
| Deadline    | Default           | Infinity (0)                | ⚠️  |
| Liveliness  | System default    | Automatic                   | ⚠️  |
|             |                   |                             |     |

`shared_memory_qos.hpp`
```c++
#pragma once

#include <rclcpp/rclcpp.hpp>

  

namespace image_processor::qos {

  

inline rclcpp::QoS SharedMemorySensorQoS() {

rclcpp::QoS qos(rclcpp::KeepLast(5));

  

qos.best_effort();

qos.durability_volatile();

qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

qos.deadline(rclcpp::Duration(0, 0)); // infinity

  

// return qos;

return rclcpp::SensorDataQoS();

}

  

} // namespace image_processor::qos
```



# 버전 확인 (사용 중인 것은 2.0.5로 확인)

> [!Important] 주의사항
> `iceoryx` & `iox-roudi` 버전이 같은지 확인


```bash
dpkg -l | grep iceoryx

ii  ros-humble-iceoryx-binding-c                      2.0.5-1jammy.20250701.012426            amd64        Eclipse iceoryx inter-process-communication (IPC) middleware C-Language Binding
ii  ros-humble-iceoryx-hoofs                          2.0.5-1jammy.20250701.011506            amd64        Eclipse iceoryx inter-process-communication (IPC) middleware basic building blocks
ii  ros-humble-iceoryx-posh                           2.0.5-1jammy.20250701.011751            amd64        Eclipse iceoryx inter-process-communication (IPC) middleware Posix Shared Memory Library and middleware daemon (RouDi)
```


```bash
iox-roudi --version
RouDi version: 2.0.5
Build date: 2025-07-01T01:17:51Z
Commit ID: 
```



# Guide (v2.0.5 사용)
https://iceoryx.io/v2.0.5/advanced/configuration-guide/

https://iceoryx.io/v2.0.2/advanced/configuration-guide/


# Limitations
https://cyclonedds.io/docs/cyclonedds/0.9.1/shared_memory.html#limitations

# Relevant Guides
https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/advanced/configuration-guide.md

https://docs.ros.org/en/humble/How-To-Guides/Configure-ZeroCopy-loaned-messages.html

https://github.com/ros2/rmw_cyclonedds/blob/rolling/shared_memory_support.md