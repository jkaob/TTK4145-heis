#!/bin/bash
#Drop only ros
#sudo iptables -A INPUT -p udp --dport 17900 -j DROP
#sudo iptables -A INPUT -p udp --dport 17912 -j DROP
#sudo iptables -A INPUT -p udp --dport 17913 -j DROP
#sudo iptables -A INPUT -p udp --sport 17900 -j DROP
#sudo iptables -A INPUT -p udp --sport 17912 -j DROP
#sudo iptables -A INPUT -p udp --sport 17913 -j DROP
#sudo iptables -A INPUT -p udp --dport 40309 -j DROP
#sudo iptables -A INPUT -p udp --sport 40309 -j DROP

#lsof -i
#echo ""
#echo ""
#echo "Dropped 3/4, need to drop last one is the port under 17913"
#echo "sudo iptables -A INPUT -p udp --dport PORT_HERE -j DROP"

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

#Drop remaining ports
sudo iptables -A INPUT -j DROP
