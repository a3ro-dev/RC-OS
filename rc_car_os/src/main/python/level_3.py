import socket
from time import sleep
from machine import Pin, PWM

import network

def connect_to_wifi(ssid, password):
    # Initialize the WiFi station interface
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # If not already connected, then connect
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect(ssid, password)

        # Wait for the board to connect to the WiFi
        while not wlan.isconnected():
            pass

    print('Network config:', wlan.ifconfig())

# Replace with your network's SSID and password
connect_to_wifi('a3rodev', 'a3rodev')