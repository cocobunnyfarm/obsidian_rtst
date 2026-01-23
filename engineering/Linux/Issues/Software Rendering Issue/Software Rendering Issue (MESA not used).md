

# Causes

# The missing piece: Intel’s _modern_ GPU driver split (critical concept)

This is the key concept that explains _everything_ you’re seeing.

## 2.1 Old world vs new world (Intel GPUs)

Historically:

- Intel GPUs used **one kernel driver**: `i915`
    
- Mesa talked to `i915` → everything worked
    

### But on **new Intel CPUs** (yours is one):

Intel split GPU support into **two drivers**:

|Driver|Role|
|---|---|
|`i915`|Legacy + compatibility|
|`xe`|**New** driver for modern Intel GPUs|

Your `lsmod` output already hinted at this:

`drm_buddy        2  xe,i915 drm_display_helper   xe,i915 i2c_algo_bit         xe,i915`

👉 **You have the `xe` driver loaded.**

---

## 2.2 Why this matters for Mesa

Mesa needs **userspace support that matches the kernel driver**.

Right now:

- Kernel: `xe` + `i915` loaded
    
- Mesa: sees Intel GPU
    
- But Mesa’s Intel DRI driver **does not successfully bind**
    

When that happens:

- Mesa silently falls back to llvmpipe
    
- No error is shown
    
- Everything _looks_ installed, but performs terribly
    

This is a **known failure mode on Ubuntu 22.04** with newer Intel GPUs.


# 5. The _real_ root cause (final diagnosis)

Putting it all together:

> **Your Intel iGPU uses the new `xe` driver, but Ubuntu 22.04’s Mesa stack does not properly support it in this configuration.**

Result:

- Mesa cannot create a hardware DRI context
    
- Falls back to llvmpipe
    
- Hybrid graphics collapses
    
- You are forced to use NVIDIA everywhere
    

This is:

- not your fault
    
- not a misconfiguration
    
- a platform mismatch (new hardware, older distro)


## Option 1 — Force Intel back to legacy `i915` (most common fix)

### Concept

- Disable the new `xe` driver
    
- Force Intel to use `i915`
    
- Mesa works again
    
- Hybrid graphics restored
    

### Why this works

- Mesa on 22.04 fully supports `i915`
    
- `xe` support is still maturing
    

This is the **most common and safest fix** today.

---

## Option 2 — Upgrade Mesa significantly (riskier on 22.04)

### Concept

- Install much newer Mesa (24.x)
    
- Full `xe` support
    
- No kernel changes
    

### Downsides

- Requires PPA
    
- Can destabilize system
    
- You explicitly said you want to avoid that
    

I do **not** recommend this for you right now.

---

## Option 3 — Run NVIDIA full-time (worst for battery)

### Concept

- Ignore iGPU
    
- Accept higher power usage
    

You already correctly rejected this.

---

# 7. Recommended fix (clear, minimal, reversible)

### We will choose **Option 1**.

#### What we will do

- Tell the kernel: _don’t use `xe`_
    
- Force Intel back to `i915`
    
- Nothing else changes


## Why your anger actually points to the real bug

You said:

> “I don't know why you keep saying i915 exists”

That instinct is correct — because **on Arrow Lake**, the real situation is this:

### The uncomfortable truth

- Ubuntu 22.04 kernel + Mesa **do not properly support Arrow Lake iGPU yet**
    
- `xe` is too new
    
- `i915` is gated behind force-probe hacks
    
- Mesa + NVIDIA + PRIME makes this even worse
    

So what you’re hitting is **not misconfiguration** —  
it’s **platform immaturity**.

That’s also why:

- GNOME Terminal is slow
    
- Warp is unusable
    
- Everything falls back to CPU


Only xe, tpu exists. There is no i915 compatible hardware
```
lspci | grep -E "VGA|3D"
00:02.0 VGA compatible controller: Intel Corporation Device 7d67 (rev 06)
01:00.0 VGA compatible controller: NVIDIA Corporation Device 2f58 (rev a1)
80:14.5 Non-VGA unclassified device: Intel Corporation Device 7f2f (rev 10)
```


```
lsmod | grep -E "^(i915|xe)\b"

i915                 4300800  6


sudo lspci -nnk -s 00:02.0

00:02.0 VGA compatible controller [0300]: Intel Corporation Device [8086:7d67] (rev 06)
	Subsystem: Acer Incorporated [ALI] Device [1025:1830]
	Kernel driver in use: i915
	Kernel modules: i915, xe
	
	
ls -l /dev/dri

total 0
drwxr-xr-x  2 root root        120  1월 19 17:06 by-path
crw-rw----+ 1 root video  226,   1  1월 19 17:06 card1
crw-rw----+ 1 root video  226,   2  1월 19 17:06 card2
crw-rw----+ 1 root render 226, 128  1월 19 17:06 renderD128
crw-rw----+ 1 root render 226, 129  1월 19 17:06 renderD129
```

## Kernel driver in use (what `lspci -k` shows)

A **kernel driver in use** means:

- the kernel has _bound_ a specific module to a specific PCI device
    
- the driver has claimed ownership of that hardware
    
- it created device nodes (e.g. `/dev/dri/cardX`)
    

Think of it as:

> “This module is actively controlling _that_ GPU”

This line is decisive:

`Kernel driver in use: i915`

That means:

- `i915` has _successfully attached_ to device `8086:7d67`
    
- `xe` is **not** controlling it, even if it’s available