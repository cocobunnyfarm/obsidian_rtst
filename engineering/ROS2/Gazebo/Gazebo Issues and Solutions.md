
##Gazebo black screen##

Check if it's either GPU or Screen software issue:
```
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo
```
If this runs fine, then there is an issue.


1. **Ignition itself works fine** - not broken
    
2. **Your NVIDIA GPU/driver IS the problem** - hardware acceleration fails
    
3. **X server/display is working** - software rendering works

So GPU/Driver issue.



```
__GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep -E "vendor|renderer"
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  156 (NV-GLX)
  Minor opcode of failed request:  6 ()
  Value in failed request:  0x0
  Serial number of failed request:  96
  Current serial number in output stream:  96
```

The `__GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo` command failed with a **BadValue X error**. This means the NVIDIA driver is loaded but can't communicate with the X server properly.


```
xrandr --listproviders
Providers: number : 2
Provider 0: id: 0x45 cap: 0x0 crtcs: 4 outputs: 3 associated providers: 0 name:modesetting
Provider 1: id: 0x1f9 cap: 0x2, Sink Output crtcs: 4 outputs: 4 associated providers: 0 name:NVIDIA-G0
```
The X server is using the `modesetting` driver (Provider 0), not the NVIDIA driver. But NVIDIA is available as Provider 1.


## **The Problem:**

- X server is using `modesetting` driver (software/Intel)
    
- NVIDIA GPU is available as Provider 1 (`NVIDIA-G0`)
    
- `NV-GLX` extension is present, but not active for rendering


```
# See which provider is active
xrandr --listproviders
Providers: number : 2
Provider 0: id: 0x45 cap: 0x0 crtcs: 4 outputs: 3 associated providers: 0 name:modesetting
Provider 1: id: 0x1f9 cap: 0x2, Sink Output crtcs: 4 outputs: 4 associated providers: 0 name:NVIDIA-G0


xrandr --listactivemonitors
Monitors: 1
 0: +*eDP-1 2560/344x1600/215+0+0  eDP-1
```
