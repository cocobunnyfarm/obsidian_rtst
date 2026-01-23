
# Concepts you need to know
The Underlay/Overlay Intuition

Think of it like **transparent sheets** on an overhead projector:

- **The Underlay (Base Layer):** This is `/opt/ros`. It contains the standard "factory" version of everything.
- **The Overlay (Your Layer):** This is `~/rmw_ws`. It contains your custom versions. 

When you "source" your overlay, you are telling the system: _"Look at my custom sheet first. If you don't find a package there, look at the base factory sheet underneath it"_. This allows you to **override** specific parts of the system (like the DDS middleware) without touching or breaking the original files.



# Things to avoid
2. Can you install it to `/opt/ros`?

Technically, you _could_ set the `CMAKE_INSTALL_PREFIX` to `/opt/ros/$ROS_DISTRO`, but this is **highly discouraged**.

- **Conflicts**: It will overwrite system-managed files, causing `apt` to fail or report broken dependencies during updates.
- **Permissions**: You would need to run `colcon build` as `sudo`, which can mess up file ownership in your workspace.

# Solution: Use Underlay-Overlay pattern
While it feels like a "hack" for testing, this isolation is exactly how professional robotics companies handle production deployments:

1. **Immutability:** In production, the `/opt/ros` folder is often treated as **read-only**. You never want a field update to accidentally brick a robot by corrupting the base system.
2. **Versioning:** If you have two different robots—one needing CycloneDDS with Iceoryx and another needing standard FastDDS—you can ship the same base OS image to both and just swap the **Overlay** folder.
3. **Docker/Containers:** In a production Dockerfile, you would build your custom RMW into a specific folder (e.g., `/usr/local/custom_rmw`) and then set your entrypoint script to source it.


# What we want eventually:
#!/bin/bash
# 1. Set up the environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/rmw_ws/install/setup.bash

# 2. Set production-specific variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/robot/cyclonedds.xml

# 3. Start the daemon and the app
/home/robot/iceoryx/install/bin/iox-roudi &
sleep 2 # wait for RouDi
ros2 run my_package my_node

```
#!/bin/bash
# 1. Set up the environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/rmw_ws/install/setup.bash

# 2. Set production-specific variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/robot/cyclonedds.xml

# 3. Start the daemon and the app
/home/robot/iceoryx/install/bin/iox-roudi &
sleep 2 # wait for RouDi
ros2 run my_package my_node

```
# Core Issue
We need to put `libddsc.so` file which is a symbolic link, linking at `libddsc.so.0.11.0` to be installed at the right location. 

## Check dependencies
It seems that this file only depends on system libraries. And in our system, it's all available:
```
ldd /home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/lib/libddsc.so
	linux-vdso.so.1 (0x00007ffedaff6000)
	libssl.so.3 => /lib/x86_64-linux-gnu/libssl.so.3 (0x0000775f9a28f000)
	libcrypto.so.3 => /lib/x86_64-linux-gnu/libcrypto.so.3 (0x0000775f99e00000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x0000775f99a00000)
	/lib64/ld-linux-x86-64.so.2 (0x0000775f9a4e2000)
```

If not available, it will instead show, something like:
```
libddsrt.so => not found
libdds_security_ac.so => not found
libdds_security_auth.so => not found
libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6
...
```

You can check this with `readefl` command as well:
```
readelf -d /home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/lib/libddsc.so | grep NEEDED
 0x0000000000000001 (NEEDED)             Shared library: [libssl.so.3]
 0x0000000000000001 (NEEDED)             Shared library: [libcrypto.so.3]
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x0000000000000001 (NEEDED)             Shared library: [ld-linux-x86-64.so.2]
```