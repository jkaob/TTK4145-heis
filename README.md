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

## Pakketapssimulering
#### Finn de rette portene
ROS/DDS benytter seg av 4 UDP porter som genereses som en fuksjon av ROS_DOMAIN_ID(42 i vårt tilfelle). Mer info om dette finner man her: https://community.rti.com/howto/statically-configure-firewall-let-omg-dds-traffic-through

Tre av fire porter er fast bestemt og er i vårt tilfelle:
##### 17900
##### 17912
##### 17913

Den siste porten finner man ved å bruke følgende kommando:
```
lsof -i
```
#### Stenge kommunikasjon mellom portene
Når du har dine fire porter, stenger man enten alle pakker som sendes gjennom de med eksempel:

```
sudo iptables -A INPUT -p udp --dport 17900 -j DROP
sudo iptables -A INPUT -p udp --dport 17912 -j DROP
sudo iptables -A INPUT -p udp --dport 17913 -j DROP
sudo iptables -A INPUT -p udp --dport 48904 -j DROP

```
For å kun "miste" en bestemt andel(her 20%) av pakkene kan man bruke eksempelvis:
```
sudo iptables -A INPUT -p udp --dport 17900 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 17912 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 17913 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 48904 -j ACCEPT

sudo iptables -A INPUT -m statistic --mode random --probability 0.2 -j DROP
```

## TODO
#### Offline cab orders
Når man mister nett skjer følgende:
- Om man spammer masse knapper, så får den en ny ordre når den reconnecter og kjører ut av banen
- Får ikke fjernet ros helt, men sånn får det bare bl
- Den kjører aldri noe form for reconnect



#### Er dette gjort?:
Skal vi lage en 'event'modul?

Altså lage en modul som inneholder
  NewButtonPushed - Som tar for seg:
     for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b, f)
                if (v and (v != elev.queue[elev.id][f][b])):
Osv...
