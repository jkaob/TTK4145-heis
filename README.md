# TTK4145-heis
Elevator project for TTK4145 - Real Time Programming

### Driver module
We use the driver interface provided and interface with it using ctypes.
A simple example is shown in example.py. The library is simply compiled using gcc
and then imported into the python file.

The following line shows how to compile the driver library.

``` cmake
gcc --std=gnu11 -shared -fPIC io.c elev.c -o driver.so /usr/lib/libcomedi.so
```

### ROS2
The project is running on ROS2 Eloquent. To install this follow this guide:
https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/
