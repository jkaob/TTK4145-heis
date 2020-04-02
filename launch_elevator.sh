#!/bin/bash         
 
echo "  _______ _______ _  ___  _  __ _  _   _____  "; 
echo " |__   __|__   __| |/ / || |/_ | || | | ____| ";
echo "    | |     | |  | ' /| || |_| | || |_| |__   ";
echo "    | |     | |  |  < |__   _| |__   _|___ \  ";
echo "    | |     | |  | . \   | | | |  | |  ___) | ";
echo "    |_|     |_|  |_|\_\  |_| |_|  |_| |____/  ";

echo "  ______ _                 _               _           _      ";
echo " |  ____| |               | |             | |         | |     ";
echo " | |__  | | _____   ____ _| |_ ___  _ __  | |     __ _| |__   ";
echo " |  __| | |/ _ \ \ / / _  | __/ _ \|  __| | |    / _  |  _ \  ";
echo " | |____| |  __/\ V / (_| | || (_) | |    | |___| (_| | |_) | ";
echo " |______|_|\___| \_/ \__,_|\__\___/|_|    |______\__,_|_.__/  ";
echo ""
echo ""

echo "Welcome to TDT4145 Elevator Lab!"
sleep .5
echo ""
while true; do
    read -p "Start Elevator? (y/n): " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Very well, goodbye!";exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo ""

echo "Starting Elevator..."
sleep .5

cd dev_ws
. install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run elevator elev

echo ""
echo ""
echo "Elevator finished, goodbye!"
cd ..
