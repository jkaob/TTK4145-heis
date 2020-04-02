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
## TODO
Skal vi lage en 'event'modul?

Altså lage en modul som inneholder
  NewButtonPushed - Som tar for seg:
     for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b, f)
                if (v and (v != elev.queue[elev.id][f][b])):
Osv...

