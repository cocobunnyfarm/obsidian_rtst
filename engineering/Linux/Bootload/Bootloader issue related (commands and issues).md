
Looking at your output, I can explain what's happening and provide solutions. This is a common UEFI boot entry management issue.

## **What's Happening:**

1. **UEFI Shell entry keeps reappearing**: This happens because UEFI Shell is often set up as a **fallback boot option** or **Boot#### entries with high numbers (2000+) are built-in/hardware boot options** that the firmware regenerates automatically.

2. **Your Boot#### entries:**
   - `Boot0000`, `Boot0002`: User-defined/UEFI applications (these can be managed)
   - `Boot2001-2003`: Hardware-based boot options (USB, DVD, Network - managed by firmware)
   - `BootCurrent: 0000` means you're currently booted to UEFI Shell

3. **Why Ubuntu24 disappears from BIOS boot order**: Some UEFI firmware interfaces only show "active" or "verified" boot entries in their visual menus. The shell entry might be overriding what's displayed.

## **How UEFI Boot Order Works:**

```
BootOrder: 0000,0002,2001,2002,2003
```
- The firmware tries `0000` (UEFI Shell) first
- If that fails, tries `0002` (Ubuntu24)
- Then tries hardware options

## **Solutions:**

### **1. Fix Boot Order Permanently:**
```bash
# Delete the shell entry
sudo efibootmgr -b 0 -B

# Set Ubuntu as first boot option
sudo efibootmgr -o 0002,2001,2002,2003

# Optional: Disable UEFI Shell auto-recovery
# Check if shell exists as a fallback file
ls -la /boot/efi/EFI/UEFISHELL/
```

### **2. Delete the Shell Files (Most Effective):**
The shell keeps reappearing because the file exists on your ESP (EFI System Partition):
```bash
# Mount ESP if not already mounted
sudo mount /dev/sda1 /boot/efi  # adjust partition if needed

# Remove or rename the shell directory
sudo mv /boot/efi/EFI/UEFISHELL /boot/efi/EFI/UEFISHELL.disabled

# Update boot entries
sudo efibootmgr -o 0002,2001,2002,2003
```

### **3. Use efibootmgr to Create a New Ubuntu Entry:**
```bash
# Create a new Ubuntu entry
sudo efibootmgr -c -d /dev/sda -p 1 -L "Ubuntu" -l \\EFI\\ubuntu24\\grubx64.efi

# Set it as first boot
sudo efibootmgr -o NEW_BOOT_NUMBER  # replace with actual number
```

### **4. Check and Fix in UEFI Shell:**
From UEFI Shell, you can:
```shell
# List all boot entries
bcfg boot dump

# Remove the problematic entry (usually 0)
bcfg boot rm 0

# Add Ubuntu
bcfg boot add 0 FS0:\EFI\ubuntu24\grubx64.efi "Ubuntu"

# Set boot order
bcfg boot mv 0 0  # This varies by shell version
```

## **Where to Look:**
1. **Check ESP contents**: `/boot/efi/EFI/` - look for `UEFISHELL` directory
2. **Check NVRAM variables directly**: 
   ```bash
   # View all EFI variables
   ls /sys/firmware/efi/efivars/
   ```
3. **BIOS/UEFI Settings**: Enter firmware setup (F2/Del during boot) and:
   - Disable "Boot from EFI Shell"
   - Disable "Network Boot" if not needed
   - Enable "Secure Boot" might restrict shell (but could affect Ubuntu)

## **Why You See Both in UEFI Shell Reset Menu:**
The UEFI Shell's boot manager (accessed via `reset` or boot menu) reads **all** EFI applications from the ESP, while your motherboard's BIOS boot menu might filter based on security/certificates.

## **Quick Fix:**
```bash
# 1. Mount ESP
sudo mount /dev/sda1 /boot/efi

# 2. Disable shell
sudo mv /boot/efi/EFI/UEFISHELL /boot/efi/EFI/UEFISHELL.disabled

# 3. Update boot order
sudo efibootmgr -o 0002,2001,2002,2003

# 4. Reboot
sudo reboot
```

If the problem persists after reboot, your motherboard firmware might have a "restore default boot entries" feature that you need to disable in BIOS settings.