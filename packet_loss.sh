#!/bin/bash
#Accept simulator and discord
sudo iptables -A INPUT -p tcp --dport 15657 -j ACCEPT #Sim
sudo iptables -A INPUT -p tcp --sport 15657 -j ACCEPT #Sim
sudo iptables -A INPUT -p tcp --dport 53080 -j ACCEPT #Sim
sudo iptables -A INPUT -p tcp --sport 53080 -j ACCEPT #Sim

sudo iptables -A INPUT -p tcp --dport 33564 -j ACCEPT #Discord
sudo iptables -A INPUT -p tcp --sport 33564 -j ACCEPT #Discord

sudo iptables -A INPUT -p tcp --dport 58766 -j ACCEPT #Discord
sudo iptables -A INPUT -p tcp --sport 58766 -j ACCEPT #Discord

sudo iptables -A INPUT -p tcp --dport 36598 -j ACCEPT #Discord
sudo iptables -A INPUT -p tcp --sport 36598 -j ACCEPT #Discord

sudo iptables -A INPUT -p tcp --dport 6463 -j ACCEPT #Discord
sudo iptables -A INPUT -p tcp --sport 6463 -j ACCEPT #Discord

sudo iptables -A INPUT -p udp --dport 47861 -j ACCEPT #Discord
sudo iptables -A INPUT -p udp --sport 47861 -j ACCEPT #Discord

#Drop remaining ports with 20% packetloss
sudo iptables -A INPUT -m statistic --mode random --probability 0.2 -j DROP
