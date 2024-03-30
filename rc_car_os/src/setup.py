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

    def _install_dependencies(self):
        """
        Installs the necessary dependencies for the RC car OS on a Linux-based system.
        """

        print("Installing dependencies...")

        subprocess.run(["sudo", "apt-get", "update"])
        subprocess.run(["sudo", "apt-get", "install", "python3-pip", "python3-dev", "nmcli"])

        print("Dependencies installed successfully.")

    def _connectWifi(self):
        """
        Connects the RC car OS to a WiFi network.
        """

        ssid = input("Please enter the SSID of the WiFi network you want to connect to: ")
        password = input("Please enter the password of the WiFi network you want to connect to: ")

        subprocess.run(["nmcli", "device", "wifi", "connect", ssid, "password", password])
