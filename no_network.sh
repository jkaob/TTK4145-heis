#!/bin/bash

sudo iptables -A INPUT -p udp --dport 17900 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 17912 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 17913 -j ACCEPT
sudo iptables -A INPUT -p udp --sport 17900 -j ACCEPT
sudo iptables -A INPUT -p udp --sport 17912 -j ACCEPT
sudo iptables -A INPUT -p udp --sport 17913 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 40309 -j ACCEPT
sudo iptables -A INPUT -p udp --sport 40309 -j ACCEPT

lsof -i
echo ""
echo ""
echo "Dropped 3/4, need to drop last one is the port under 17913"
echo "sudo iptables -A INPUT -p udp --dport PORT_HERE -j DROP"
