# TTK4145-heis
Elevator project for TTK4145 - Real Time Programming

## Driver module
We use the driver interface provided and interface with it using ctypes.
A simple example is shown in example.py. The library is simply compiled using gcc
and then imported into the python file.

The following line shows how to compile the driver library.

``` cmake
gcc --std=gnu11 -shared -fPIC io.c elev.c -o driver.so /usr/lib/libcomedi.so
```

## ROS2 & Colcon

#### Installation
The project is running on ROS2 Eloquent and built with Colcon. To install ROS2 follow this guide:
https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

Colcon is installed with apt:

```
sudo apt install python3-colcon-common-extensions
sudo apt install libcomedi-dev
```

#### Ros Domain
To talk between ros nodes on different computers you need to set a correct domain id:
```
export ROS_DOMAIN_ID=42
```
## VirtualBox
#### 1 Download Oracle VM VirtualBox for Ubuntu 18.04
https://www.virtualbox.org/wiki/Linux_Downloads

#### 2 Følg denne guiden 
https://linuxhint.com/install_ubuntu_18-04_virtualbox/

#### 3 Følg denne guiden
https://www.techrepublic.com/blog/diy-it-guy/using-virtualbox-vms-on-your-networks-subnet/?fbclid=IwAR1mAdVPzRxTlLvwHgCc46JEf08iOLEGD9CQWDCLc_SSUJ_nK4lST82tbZc

#### 4 Installeringer
Start VM og last ned ROS og Colcon som beskrevet over på VM
