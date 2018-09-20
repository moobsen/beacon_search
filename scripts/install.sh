#!/bin/bash
sudo cp beacon-search-init /etc/init.d/beacon-search
sudo cp wpa_supplicant.conf /etc/wpa_supplicant.conf
#sudo update-rc.d beacon-search defaults
sudo apt install python-yaml
sudo systemctl enable beacon-search
