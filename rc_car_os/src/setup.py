import os
import subprocess
import platform

class Setup:
    """
    The `Setup` class represents the setup configuration for the RC car OS.

    This class provides methods and attributes to configure the RC car OS before running it.

    Methods:
    """

    def __init__(self):
        """
        Initializes the `Setup` class.
        """

        self._os = platform.system()
        self._os_version = platform.version()

        self.battery_size = int(input("Please enter the size of your car's battery (in Ah): "))
        self.battery_voltage = float(input("Please enter the voltage of your car's battery (in Volts): "))
        self.number_of_motors = int(input("Please enter the number of motors in your RC vehicle: "))

        if self._os == "Linux":
            self._install_dependencies()
        else:
            print("Unsupported operating system. Please use a Linux kernel.")
        
        # Check if the machine has a wireless network interface
        interfaces = subprocess.check_output(["nmcli", "device", "status"]).decode("utf-8").split("\n")
        wireless_interface = [line for line in interfaces if "wifi" in line.lower()]

        if wireless_interface:
            self._connectWifi()
        else:
            raise Exception("No wireless network interface found. Please ensure your machine has a wireless network interface.")

    def _install_dependencies(self):
        """
        Installs the necessary dependencies for the RC car OS on a Linux-based system.
        """

        print("Installing dependencies...")
        # Update the package list and install the necessary dependencies
        try:
            subprocess.run(["sudo", "apt-get", "update"], check=True)
            subprocess.run(["sudo", "apt-get", "install", "-y", "python3-pip", "python3-dev", "network-manager"], check=True)
            print("Dependencies installed successfully.")
        except Exception as e:
            print(f"Error installing dependencies: {e}")

    def _connectWifi(self):
        """
        Connects the RC car OS to a WiFi network.
        """

        ssid = input("Please enter the SSID of the WiFi network you want to connect to: ")
        password = input("Please enter the password of the WiFi network you want to connect to: ")
        try:
            subprocess.run(["nmcli", "device", "wifi", "connect", ssid, "password", password])
            print(f"Connected to WiFi network: {ssid}")
        except Exception as e:
            print(f"Error connecting to WiFi network: {e}")
