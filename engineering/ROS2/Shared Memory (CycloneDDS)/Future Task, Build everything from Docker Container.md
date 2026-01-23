
Okay then. How about I just make a docker container, where I will download sources for cyclonedds, iceonyx and do 



Prerequisites Eclipse iceoryx depends on several packages (cmake, libacl1, libncurses5, pkgconfig and maven). Note The following steps were done on Ubuntu 20.04. Install the prerequisite packages: sudo apt install cmake libacl1-dev libncurses5-dev pkg-config maven Get and build iceoryx. The following assumes that the install is in your home directory: git clone https://github.com/eclipse-iceoryx/iceoryx.git -b release_2.0 cd iceoryx cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install -DROUDI_ENVIRONMENT=on -DBUILD_SHARED_LIBS=ON -Hiceoryx_meta cmake --build build --config Release --target install Get Cyclone DDS and build it with shared memory support: git clone https://github.com/eclipse-cyclonedds/cyclonedds.git cd cyclonedds cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install -DENABLE_ICEORYX=On -DBUILD_EXAMPLES=On -DCMAKE_PREFIX_PATH=~/iceoryx/install/ cmake --build build --config Release --target install When the compiler has finished, the files for both iceoryx and Cyclone DDS can be found in the specified install directories. And then also using those and do: Step 2: Build the RMW Layer against your Custom Build You need to tell colcon where your custom Iceoryx and Cyclone DDS are located using the CMAKE_PREFIX_PATH: bash cd ~/rmw_ws source /opt/ros/$ROS_DISTRO/setup.bash # Build the RMW pointing to your manual install directories colcon build --cmake-args \ -DCMAKE_PREFIX_PATH="~/iceoryx/install;~/cyclonedds/install" And take out final files needed from install folder? So when I run my docker container, I don't have to worry ever again how it builds and create everything needed to interface to ROS2? And then I can use that overlay-underlay pattern you mentioned, in another ubuntu system that is outside of the docker container, provided that the system has all the system dependencies

Yes, this is a highly effective way to handle a "production-grade" custom middleware. Using Docker to build everything—

**Iceoryx**, **Cyclone DDS**, and the **RMW layer**—ensures a consistent build environment and allows you to distribute just the "final" artifacts to your host machine.

The Strategy: "Build in Docker, Run as Overlay"

You can use a **Multi-Stage Dockerfile** to build everything and then "extract" the final `install` folders. Once extracted, your host system doesn't need to know _how_ they were built; it just needs to "source" them.

1. The Docker-Build Workflow

You can create a Docker image that performs all the steps you listed.

- **Compilation:** It builds Iceoryx (release_2.0) and Cyclone DDS (with `-DENABLE_ICEORYX=On`).
- **Extraction:** After the build, you copy the `install` folders from the container to your host:
    
    bash
    
    ```
    # Run the container once to copy files out
    docker run --name builder-temp my-ros-custom-build
    docker cp builder-temp:/home/user/iceoryx/install ./iox_install
    docker cp builder-temp:/home/user/cyclonedds/install ./dds_install
    docker cp builder-temp:/home/user/rmw_ws/install ./rmw_install
    docker rm builder-temp
    ```
    
    코드를 사용할 때는 주의가 필요합니다.
    
     

2. Using it as a "Portable Overlay"

On your target Ubuntu system (outside Docker), you can now use the **Underlay/Overlay** pattern without having any of the build tools (like `maven` or `cmake`) installed. 

**Important:** The target system must have the **runtime dependencies** of Iceoryx and Cyclone DDS: 

- `libacl1` (for shared memory access)
- `libncurses5` (if you use `iox-introspection`)
- `libssl` (if Cyclone DDS was built with encryption)

3. Why this is better for Production

- **No Build Pollution:** Your production system doesn't need compilers, `git`, or `maven`. It stays slim.
- **Binary Portability:** As long as your Docker base image (e.g., `ubuntu:20.04`) matches your host OS, the binaries will work perfectly.
- **Easy Rollbacks:** If a new version of the custom RMW has a bug, you simply swap the folder for the previous one and update your `source` command.

How to set it up permanently

On your target system, place these folders in a dedicated directory (e.g., `/opt/custom_ros_rmw/`) and add a small setup script:

bash

```
# /opt/custom_ros_rmw/setup.bash
export CMAKE_PREFIX_PATH="/opt/custom_ros_rmw/iox_install:/opt/custom_ros_rmw/dds_install:/opt/custom_ros_rmw/rmw_install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/opt/custom_ros_rmw/iox_install/lib:/opt/custom_ros_rmw/dds_install/lib:/opt/custom_ros_rmw/rmw_install/lib:$LD_LIBRARY_PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/robot/cyclonedds.xml
```

코드를 사용할 때는 주의가 필요합니다.

Then, in your target system's `~/.bashrc`:

bash

```
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/custom_ros_rmw/setup.bash
```

코드를 사용할 때는 주의가 필요합니다.

Would you like a **Dockerfile template** that automates the entire build process for Iceoryx, Cyclone DDS, and the RMW layer so you can just run one command to get your files?