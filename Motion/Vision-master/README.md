# Basic pointcloud processing with open3d and pcl

Download an example pointcloud [HERE](https://drive.google.com/open?id=1VgE8L1IlzrO0No8fxAvTlCrLiUaAgckR)

and run test.py.

# Install RGBD camera driver and basic image processing tools with Ubuntu 16.04 and python 2.7 and 3+ (install python 3.7-dev and pip3 instead)

## Preparation (Do not skip)

Step 1: Update the package manager

```bash
sudo apt-get update
sudo apt-get upgrade
```

Step 2: Install developer tools

```bash
sudo apt-get install build-essential cmake git pkg-config
```

Step 3: Install pip

```bash
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
```

Step 4: Install python 2.7 developer version

```bash
sudo apt-get install python2.7-dev
```

Step 5: Install the necessary dependencies for realsense camera driver and wrapper

```bash
sudo apt-get install libssl-dev git
sudo pip install cython pycparser numpy matplotlib
```

## Install basic computer vision softwares

Step 1: Install opencv and its contrib repository (2D image processing tools)

```bash
sudo pip install opencv-contrib-python
```

Step 2: Install open3d (3D data processing tool)

```bash
sudo pip install open3d-python
```

Step 3(optional): Install Point Cloud Library and its python wrapper

```bash
sudo apt-get install libpcl-dev -y
sudo pip instal python-pcl
```

## Install realsense driver and its python wrapper


#### Option1: To use with legacy camera models (R200, F200, SR300, LR200, ZR200)

Install librealsense legacy version v1.12.1 (https://github.com/IntelRealSense/librealsense/tree/v1.12.1),
and 3rd party python wrapper for librealsense v1.x (https://github.com/toinsson/pyrealsense)

First, check your kernel version and make sure you have 4.4 by running:

```bash
uname -r
```

If you install 16.04 recently, your kernel version is most likely higher than 4.4, (e.g., 4.8, 4.10, 4.15). In this case, you do not necessarily need to downgrade your kernel version as 4.4 is still supported on 16.04.

If you do not know how to boot into the 4.4 kernel, you need to access grub and boot into 4.4 using the following step:

```bash
sudo apt-get install linux-image-generic
sudo nano /etc/default/grub && sudo update-grub
```

Comment out GRUB_HIDDEN_TIMEOUT and GRUB_TIMEOUT=0, write the file Ctrl+o and exit Ctrl+x, update-grub will run. After update-grub, the grub menu should not be hidden when rebooting.

Reboot your computer, go to advanced options, and boot the version starting with 4.4.

Note: make sure you boot with this kernel version when you install the driver and use the camera in the future.

Install librealsense

```bash
git clone https://github.com/IntelRealSense/librealsense
git checkout v1.12.1
```

Make sure that the depth camera is unplugged, and follow the installation steps in
(https://github.com/IntelRealSense/librealsense/blob/v1.12.1/doc/installation.md)

#### NOTE

When you run ./scripts/patch-uvcvideo-16.04.simple.sh, you will get this error: /bin/bash: ./scripts/ubuntu-retpoline-extract-one: No such file or directory

There is no precaution for this, and you need to wait until this error appears to fix this by:

```bash
cd ubuntu-xenial
cp debian/scripts/retpoline-extract-one scripts/ubuntu-retpoline-extract-one
```

And then rerun ./scripts/patch-uvcvideo-16.04.simple.sh (press y when ask if re-apply patch again)

You will also get other errors if you didn’t follow the preparation step. After the errors have been fixed, resume with the remaining steps provided by librealsense.

Install pyrealsense driver

```bash
git clone https://github.com/toinsson/pyrealsense
cd pyrealsense
sudo python setup.py install
```
#### Option2: To use with newer camera models (SR300 and D series)

Install Intel realsense SDK 2.0 (https://github.com/IntelRealSense/librealsense),
and its official python wrapper pyrealsense2

Intel realsense SDK 2.0 can be installed easily with pre-build packages and supports multiple Ubuntu LTS kernels. Please follow the installation guide on their website.

After installing the 2.0 SDK, install its python wrapper by 

```bash
pip install pyrealsense2
```

Now you can try record the RGB-D images streaming from the realsense cameras to .jpg(color) and .png(depth) images by running record.py following a folder name. 







