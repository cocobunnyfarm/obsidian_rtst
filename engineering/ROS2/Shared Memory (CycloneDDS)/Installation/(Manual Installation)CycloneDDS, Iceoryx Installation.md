
> [!NOTE] Use auto install
> Turns out that regular installation of rmw_cyclonedds using apt contains shared memory build. 
> 
> So below is not needed now unless you really have to build all of them from source



> [!Warning] Status of this document
> Here is what's done:
> 
> 1. Using iceoryx and cyclonedds sources codes to create `.so` files that are needed by rmw_cyclonedds layer (cyclonedds must be built using right iceoryx `.so` files)
> 2. Setting right configuration files for both iceoryx and cyclonedds
> 
> Here is what's need to be done:
> 1. Build rmw_cyclonedds with all the lib files created from previous stage.
> 2. Overlay rmw_cyclonedds with right environment variables so that ROS2 Humble uses our rmw.




# Installtion

Follow instructions written here:

> [!NOTE] Which branch to choose from?
> Don't choose `iceoryx` branch. Older instructions tells you to use that branch, but master branch just works fine. Just follow the git command in the link.


https://cyclonedds.io/docs/cyclonedds/latest/shared_memory/shared_memory.html


# PSMX library loading problem

You may have the following problem:
```
./ThroughputPublisher 16384 0 1 10 "Throughput example"
payloadSize: 16384 bytes burstInterval: 0 ms burstSize: 1 timeOut: 10 seconds partitionName: Throughput example
1768299376.467061 [0] Throughput: Failed to load PSMX library 'psmx_iox' with error "psmx_iox: cannot open shared object file: No such file or directory".
1768299376.467078 [0] Throughput: error loading PSMX instance "iox"
1768299376.467095 [] Throughput: dds_create_participant: Error
Aborted (core dumped)
```

You need to find this dynamic library file:
```
find /home/ted/projects/cyclone_iceoryx_shm d -name "*psmx*" -type f | grep -i psmx
find: ‘d’: No such file or directory
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/src/dds__psmx.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/src/dds_psmx.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_cdds_impl.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_v0_impl.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_public.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_v0_impl.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmxif.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_cdds_impl.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_impl.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_impl.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_dummy_v0_public.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/config_psmx.xml
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/tests/psmx_cdds_data.idl
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsc/include/dds/ddsc/dds_psmx.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/core/ddsi/include/dds/ddsi/ddsi_psmx.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/psmx_iox/src/psmx_iox_impl.cpp
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/psmx_iox/src/psmx_iox2_impl.c
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/psmx_iox/include/psmx_iox2_impl.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/src/psmx_iox/include/psmx_iox_impl.hpp
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/lib/libpsmx_iox.so.0.11.0
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/include/dds/ddsc/dds_psmx.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/include/dds/ddsi/ddsi_psmx.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/docs/manual/api/psmx.rst
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/src/core/CMakeFiles/ddsc.dir/ddsc/src/dds_psmx.c.o.d
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/src/core/CMakeFiles/ddsc.dir/ddsc/src/dds_psmx.c.o
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/src/psmx_iox/CMakeFiles/psmx_iox.dir/src/psmx_iox_impl.cpp.o.d
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/src/psmx_iox/CMakeFiles/psmx_iox.dir/src/psmx_iox_impl.cpp.o
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/src/psmx_iox/include/psmx_iox_export.h
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/build/lib/libpsmx_iox.so.0.11.0
```

In this case, it was at:
```
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/lib/libpsmx_iox.so.0.11.0
```

Check `LD_LIBRARY_PATH` Variable
```
echo $LD_LIBRARY_PATH
/home/ted/projects/cyclone_iceoryx_shm/source_codes/iceoryx/install/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
```

So we add the path 
```
export LD_LIBRARY_PATH=/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/lib/${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

```

```bash
echo $LD_LIBRARY_PATH
/home/ted/projects/cyclone_iceoryx_shm/source_codes/cyclonedds/install/lib/:/home/ted/projects/cyclone_iceoryx_shm/source_codes/iceoryx/install/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
```


# rmw_cyclonedds

Make sure to clone `humble` branch:
```git
git clone https://github.com/ros2/rmw_cyclonedds.git -b humble
```

