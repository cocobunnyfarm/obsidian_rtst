
# **Complete UEFI Boot Troubleshooting Guide**

## **Current Situation Analysis**

### **Folder Structure on ESP (EFI System Partition):**
```
/EFI/
├── BOOT/
│   ├── bootx64.efi    # (Possibly broken/missing)
│   ├── fbx64.efi      # Fallback bootloader
│   └── mmx64.efi      # Memory test utility
├── ubuntu24/
│   └── grubx64.efi    # Working GRUB bootloader
└── UEFISHELL/
    ├── bootia32.efi   # 32-bit UEFI Shell
    └── bootx64.efi    # 64-bit UEFI Shell
```

### **Understanding UEFI Boot Process:**
1. **UEFI firmware** starts → checks NVRAM boot entries
2. **Boots first valid entry** in boot order (position 00)
3. If all fail → tries `/EFI/BOOT/bootx64.efi` (fallback)
4. If that fails → drops to UEFI Shell

## **Key Concepts Clarified**

### **1. Boot Order vs Variables:**
- **Position numbers** (00, 01, 02): Order in which UEFI attempts boot
- **Variable names** (Boot0000, Boot0001): Internal UEFI identifiers
- **No problem** if Boot0002 is at position 01 - variables can have gaps

### **2. UEFI Shell Location:**
Your UEFI Shell is at: `fs0:\EFI\UEFISHELL\bootx64.efi`

## **Complete Fix Procedure**

### **Step 1: From UEFI Shell - Fix Boot Order**
```
# 1. List current boot order
bcfg boot dump

# 2. Remove UEFI Shell from first position (if at 00)
bcfg boot rm 0

# 3. Add Ubuntu as first boot option
bcfg boot add 0 fs0:\EFI\ubuntu24\grubx64.efi "Ubuntu"

# 4. OPTIONAL: Add UEFI Shell as backup (position 99 = last)
bcfg boot add 99 fs0:\EFI\UEFISHELL\bootx64.efi "UEFI Shell"

# 5. Verify new order
bcfg boot dump
```

### **Step 2: Create Fallback Bootloader**
```
# Copy working GRUB to standard fallback location
cp fs0:\EFI\ubuntu24\grubx64.efi fs0:\EFI\BOOT\bootx64.efi
```

### **Step 3: Reboot and Test**
```
reset
```

## **Common Commands Reference**

### **UEFI Shell Commands:**
```
map -r                     # List all filesystems (fs0:, fs1:, etc.)
fs0:                       # Switch to first filesystem
ls                         # List files
bcfg boot dump             # Show boot entries
bcfg boot add N <path> "Name"  # Add boot entry at position N
bcfg boot rm N             # Remove entry at position N
bcfg boot mv N M           # Move entry from position N to M
```

### **Linux Commands (Once Booted):**
```bash
# View EFI boot variables
sudo efibootmgr -v

# Reinstall GRUB properly
sudo grub-install --target=x86_64-efi --efi-directory=/boot/efi --bootloader-id=ubuntu24

# Update GRUB configuration
sudo update-grub

# Check ESP mount
mount | grep efi
```

## **Troubleshooting Matrix**

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Goes to UEFI Shell | Boot order has Shell first | `bcfg boot mv 1 0` |
| "Failed to open" error | Missing/corrupt bootx64.efi | Copy GRUB to fallback location |
| efibootmgr changes ignored | Firmware bug | Use UEFI Shell's `bcfg` |
| Boot0002 at position 01 | Normal variable gap | Ignore, focus on positions |

## **Why Your Setup Will Work**

1. **UEFI firmware** → Checks position 00 first
2. **Position 00** = GRUB bootloader (`grubx64.efi`)
3. **GRUB loads** → Shows menu → Boots Ubuntu
4. **If GRUB fails** → Falls back to `bootx64.efi` (your copied GRUB)
5. **If that fails** → Last resort UEFI Shell (position 99)

## **Quick Recovery Script (Save to USB)**
Create file `fixboot.nsh` on FAT32 USB:
```bash
# Fix UEFI boot script
bcfg boot rm 0
bcfg boot add 0 fs0:\EFI\ubuntu24\grubx64.efi "Ubuntu"
cp fs0:\EFI\ubuntu24\grubx64.efi fs0:\EFI\BOOT\bootx64.efi
reset
```

## **Important Notes**
1. **UEFI Shell is a tool**, not a boot target (usually position 99 if at all)
2. **Position 00 MUST be your OS bootloader** (Ubuntu/Windows)
3. **Fallback path** (`/EFI/BOOT/bootx64.efi`) is critical for recovery
4. **Variable names** (Boot000X) can have gaps - only positions matter
5. **Some firmwares** ignore `efibootmgr` - use UEFI Shell directly

## **Final Check Before Rebooting**
From UEFI Shell, verify:
```
bcfg boot dump
# Should show:
# BootOption: 00. Variable: Boot000X
#   Desc    - Ubuntu
#   DevPath - .../grubx64.efi
```

If correct, type `reset` and your system should boot directly to GRUB/Ubuntu!