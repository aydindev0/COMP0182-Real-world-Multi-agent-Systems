# COMP0182 Real-world Multi-agent Systems: Lab Sheet Eight

Before begin this lab, we need to downgrade our ubuntu system to 20.04. For convience, each group just needs one person who has a windows pc to do this.

# Task 1: Installation of Ubuntu 20.04

Adapted from:
https://www.youtube.com/watch?v=-iSAyiicyQY&t=562s&ab_channel=KskRoyal

**0. Optional**:
If you want to or you don't have enough disk space, you can delete the previous Ubuntu 22.04 from your disk management, then select the partition for Ubuntu 22.04 and press delete volume. 

**1. Pre-requisites**:
- A Windows 11 or higher computer
- At least 25 GB of free disk space for Ubuntu 20.04
- A USB drive with at least 8GB
- Internet connection

**2. Disable Secure Boot**:

For some computers, `Secure Boot` will still prevent you from installing or booting new operating systems. To disable this:
    - Turn your computer OFF. Then, turn it back ON and press the BIOS entry key during the boot process. This varies between hardware types, but is generally F1, F2, F12 …
    - Find the **Secure Boot** option. If possible, set it to **Disabled**. It is usually found in the Security tab, Boot tab, or Authentication tab.
    - **Save and Exit**. Your system will reboot.

Reference:
    [How to Disable UEFI Secure Boot to Dual Boot Any System](https://www.makeuseof.com/tag/disable-secure-uefi-dual-boot/)

**3. Create a new partition on your Windows computer for Ubuntu**:
- Click ***Win + R*** and type ***cmd*** to open the terminal
- Type `diskmgmt.msc` and enter to open the Disk management
- Right click the last available partition and select ***Shrink volume***, shrink at least 25 GB
- An unallocated partition will come up once it is shrunk successfully

![Untitled](imgs/Untitled.png)

**4. Download Ubuntu 20.04 Image**:
Download ***Ubuntu 20.04*** from the official website, select ***64-bit PC (AMD64) desktop image***

[Ubuntu 20.04.6 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/)

**5. Download Rufus Software**:
***Rufus*** software is to make your USB drive bootable, select the ***Standard*** type with your computer platform.

[Rufus - Create bootable USB drives the easy way](https://rufus.ie/en/)

![image.png](imgs/image.png)

For most of you, choose the standard one with the platform Windows x64.

**6. Format the USB drive and make it bootable**:
- Insert your USB into the USB port
- In the File Explorer, right-click your USB and select ***Format*** option, and follow the instructions
- Right click the ***Rufus*** and run as an administrator
- Leave everything default, click ***SELECT*** option and select the ISO image file just downloaded (ubuntu-20.04.6-desktop-amd64.iso).
- Click ***START*** and follow the steps

![Untitled](imgs/Untitled%201.png)

**7. Install Ubuntu 20.04 Dual Boot**:
- Restart the computer. Press F11 (the button might be different for different computer brands, search for how to enter the ***Boot Menu*** for yours)
- Choose the USB drive as the boot device.

**8. Finish the Ubuntu installation setup**:

Follow the instructions to go ahead. 
- In ***Update and other software*** tab, it is recommended to select ***Normal installation*** and tick both of the two ***Other options***
- In ***Installation type*** tab, it is recommended to select ***Something else*** and continue
- In the next tab, you should see the disk info of your Windows system and the free space just created. Then select the free space and click on the ‘**+**’ button. Create a root partition (’**/**’) and allocate (recommended) 25 GB space for it, make sure you tick the same option as shown in the following figure

![Untitled](imgs/Untitled%202.png)

- Use the remaining free space to create a ***swap*** partition. It is recommended to allocate at least 4 GB for it.

![Untitled](imgs/Untitled%203.png)

- Lastly, still in the installation type tab, change the device for boot loader installation to the same device as your root partition. After all, click ***Install Now*** and follow the instructions on your screen

![Untitled](imgs/Untitled%204.png)

After restarting your computer, the Ubuntu dual boot menu will be shown.

Reference:
https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/
