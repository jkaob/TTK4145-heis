# Elevator project for TTK4145 - Real Time Programming
By Lars Musæus, Knut Vågnes Eriksen and Jakob Eide Grepperud.
## Our Stack
#### Language
The system is written in **Python**, built with **colcon** and the hardware/simulator driver is written in C and interpreted with **Ctypes**.

#### Network
For our network module we use **ROS2** which is a open source library for both Pyhton and C++.

#### Modules & System overview
System overview can be found in *design-review/uml*.

## Driver module - C to Python
We use the driver interface provided and interface with it using ctypes.
A simple example is shown in *example.py*. The library is simply compiled using gcc
and then imported into the python file. (Shoutout to *sindrehan*).

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
To talk between ros nodes on different computers you need to have the same domain id:
```
export ROS_DOMAIN_ID=42
```

#### 4 Installeringer
Start VM og last ned ROS og Colcon som beskrevet over på VM

## Build and run
To build and run the system we have made scripts for this so you don't have to retype commands all day long:

#### Build
```
. build_elevator.sh
```
#### Start simulator (optional if your'e not on the Sanntidssal)
```
. simulator.sh
```
#### Start ROS and elevator program
```
. launch_elevator.sh
```

## Packetloss simulation
#### 1. Script
We have made scripts to simulate both packetloss and complete network loss, *packet_loss.sh* and *no_network.sh*, respectively.
NOTE: They accept ports used by discord to be able to use discord during packet_loss.

#### Simulate full network loss
```
. no_network.sh
```
#### Simulate 20% packetloss
```
. packet_loss.sh
```


If u want to do it yourself without using the scripts follow instruction below.

#### 2.  Manually
ROS/DDS uses 4 UDP ports which generates by a function of ROS_DOMAIN_ID(we use 42). More info about this can be read here:
https://community.rti.com/howto/statically-configure-firewall-let-omg-dds-traffic-through

Three out of four port are static, and in our case:
##### 17900
##### 17912
##### 17913

The last port can be found with the following shell command:
```
lsof -i
```
##### Stenge kommunikasjon mellom portene
When you have the four ports, you can block them completely (example):
```
sudo iptables -A INPUT -p udp --dport 17900 -j DROP
sudo iptables -A INPUT -p udp --dport 17912 -j DROP
sudo iptables -A INPUT -p udp --dport 17913 -j DROP
sudo iptables -A INPUT -p udp --dport 48904 -j DROP

```
To only "loose" a certain amount of packages(here 20%) use this instead:
```
sudo iptables -A INPUT -p udp --dport 17900 -m statistic --mode random --probability 0.2 -j DROP
sudo iptables -A INPUT -p udp --dport 17912 -m statistic --mode random --probability 0.2 -j DROP
sudo iptables -A INPUT -p udp --dport 17913 -m statistic --mode random --probability 0.2 -j DROP
sudo iptables -A INPUT -p udp --dport 48904 -m statistic --mode random --probability 0.2 -j DROP

```
