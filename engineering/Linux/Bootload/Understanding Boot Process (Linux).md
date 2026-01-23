
UEFI works

Grub works

Multiple OS setup
1. One grubi
2. multple grubs (not recommended)


Basically, Have one grub in EFI partition. And subsequent OS installs should not add another bootloaders like grub in EFI. 

Whenever new OS is added of old OS is deleted, update the GRUb with `sudo update-grub`
command in the main Ubuntu install that installed the grub


How to fix UEFI pointing at the wrong grub thus at the boot, you no longer see grub menu.
In this case, you'll see UEFI shell.

Commands you can use:

- you can list things. You will see fs0, blocks and so on.
  type fs0: and it will let you go into that directory
- Use dir, ls and so on. Find boot file that you can execute and go to your grub menu or directly to linux due to grub settings. Run the file with \.filename not ./filename

### **For GUI management (closest to menu viewer):**

- **Grub Customizer** (from PPA) - **This is your best bet**
    
    bash
    
    sudo add-apt-repository ppa:danielrichter2007/grub-customizer
    sudo apt update
    sudo apt install grub-customizer
    
    This lets you:
    
    - View all boot entries
        
    - Reorder them
        
    - Edit kernel parameters
        
    - Change default OS
        
    - Configure timeout

This shows what you see at the boot grub menu. Otherwise you have to do text parsing of grub configuration file which is not readable.

check which partition is EFI partition:

```
df -h
Filesystem       Size  Used Avail Use% Mounted on
tmpfs            3.1G  2.6M  3.1G   1% /run
/dev/nvme0n1p2    46G   16G   28G  37% /
tmpfs             16G   42M   16G   1% /dev/shm
tmpfs            5.0M   12K  5.0M   1% /run/lock
efivarfs         268K  231K   32K  88% /sys/firmware/efi/efivars
tmpfs             16G     0   16G   0% /run/qemu
/dev/nvme0n1p1  1020M  8.0M 1013M   1% /boot/efi
/dev/nvme0n1p4    45G   27G   17G  62% /home
/dev/nvme0n1p10  499G  244M  473G   1% /mnt/shared_global
tmpfs            3.1G  124K  3.1G   1% /run/user/1000
```

See the current status of UEFI listing
```
sudo efibootmgr -v
BootCurrent: 0000
Timeout: 10 seconds
BootOrder: 0000,2001,2002,2003
Boot0000* UEFI_SHELL	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\UEFISHELL\bootx64.efi)RC
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 3a 00 5c 00 45 00 46 00 49 00 5c 00 55 00 45 00 46 00 49 00 53 00 48 00 45 00 4c 00 4c 00 5c 00 62 00 6f 00 6f 00 74 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
    data: 52 43
Boot2001* EFI USB Device	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2002* EFI DVD/CDROM	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2003* EFI Network	RC
      dp: 7f ff 04 00
    data: 52 43
```


main commands (need to understand these commands and their differences carefully)
```
sudo efibootmgr -v
BootCurrent: 0000
Timeout: 10 seconds
BootOrder: 0000,0001,2001,2002,2003
Boot0000* UEFI_SHELL	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\UEFISHELL\bootx64.efi)RC
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 3a 00 5c 00 45 00 46 00 49 00 5c 00 55 00 45 00 46 00 49 00 53 00 48 00 45 00 4c 00 4c 00 5c 00 62 00 6f 00 6f 00 74 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
    data: 52 43
Boot0001* Ubuntu	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\ubuntu24\shimx64.efi)
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 38 00 5c 00 45 00 46 00 49 00 5c 00 75 00 62 00 75 00 6e 00 74 00 75 00 32 00 34 00 5c 00 73 00 68 00 69 00 6d 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
Boot2001* EFI USB Device	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2002* EFI DVD/CDROM	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2003* EFI Network	RC
      dp: 7f ff 04 00
    data: 52 43
    
    
sudo efibootmgr -c -d /dev/nvme0n1 -p 1 -L "Ubuntu24" -l \\EFI\\ubuntu24\\grubx64.efi
BootCurrent: 0000
Timeout: 10 seconds
BootOrder: 0002,0000,0001,2001,2002,2003
Boot0000* UEFI_SHELL	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\UEFISHELL\bootx64.efi)RC
Boot0001* Ubuntu	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\ubuntu24\shimx64.efi)
Boot2001* EFI USB Device	RC
Boot2002* EFI DVD/CDROM	RC
Boot2003* EFI Network	RC
Boot0002* Ubuntu24	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\ubuntu24\grubx64.efi)


sudo efibootmgr -v
BootCurrent: 0000
Timeout: 10 seconds
BootOrder: 0002,0000,0001,2001,2002,2003
Boot0000* UEFI_SHELL	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\UEFISHELL\bootx64.efi)RC
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 3a 00 5c 00 45 00 46 00 49 00 5c 00 55 00 45 00 46 00 49 00 53 00 48 00 45 00 4c 00 4c 00 5c 00 62 00 6f 00 6f 00 74 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
    data: 52 43
Boot0001* Ubuntu	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\ubuntu24\shimx64.efi)
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 38 00 5c 00 45 00 46 00 49 00 5c 00 75 00 62 00 75 00 6e 00 74 00 75 00 32 00 34 00 5c 00 73 00 68 00 69 00 6d 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
Boot0002* Ubuntu24	HD(1,GPT,f273ed7c-84d4-458c-a24a-07b70521741a,0x800,0x200000)/File(\EFI\ubuntu24\grubx64.efi)
      dp: 04 01 2a 00 01 00 00 00 00 08 00 00 00 00 00 00 00 00 20 00 00 00 00 00 7c ed 73 f2 d4 84 8c 45 a2 4a 07 b7 05 21 74 1a 02 02 / 04 04 38 00 5c 00 45 00 46 00 49 00 5c 00 75 00 62 00 75 00 6e 00 74 00 75 00 32 00 34 00 5c 00 67 00 72 00 75 00 62 00 78 00 36 00 34 00 2e 00 65 00 66 00 69 00 00 00 / 7f ff 04 00
Boot2001* EFI USB Device	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2002* EFI DVD/CDROM	RC
      dp: 7f ff 04 00
    data: 52 43
Boot2003* EFI Network	RC
      dp: 7f ff 04 00
    data: 52 43
```

## **shimx64.efi vs grubx64.efi**

### **1. `shimx64.efi` (Secure Boot)**

- **Purpose**: Secure Boot "shim" loader
    
- **Chain**: `shim → grubx64.efi → GRUB menu`
    
- **When to use**: Systems with **Secure Boot enabled**
    
- **Size**: Smaller, just a stub
    

### **2. `grubx64.efi` (Direct GRUB)**

- **Purpose**: Direct GRUB bootloader
    
- **Chain**: `grubx64.efi → GRUB menu` (direct)
    
- **When to use**: Systems with **Secure Boot disabled**
    
- **Size**: Full GRUB binary