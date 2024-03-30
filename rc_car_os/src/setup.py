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

        # Check if the device has a wlan card
        wlan_check = subprocess.run(['iwconfig'], capture_output=True, text=True)
        if 'no wireless extensions' in wlan_check.stdout:
            print("No WLAN card detected. Please connect a WLAN card and try again.")
            return

        # Ask for the SSID and password of the WiFi network
        self.ssid = input("Enter the SSID of the WiFi network: ")
        self.password = input("Enter the password of the WiFi network: ")
        # Save the SSID and password in a local_config.py file
        config_dir = os.path.join(os.path.dirname(__file__), '..', 'resources', 'config')
        os.makedirs(config_dir, exist_ok=True)
        with open(os.path.join(config_dir, 'local_config.py'), 'w') as f:
            f.write(f"SSID = '{self.ssid}'\n")
            f.write(f"PASSWORD = '{self.password}'\n")

        # Check if the system is Linux
        if platform.system() == 'Linux':
            # Connect to the WiFi network
            os.system(f'nmcli dev wifi connect {self.ssid} password {self.password}')

            # Always connect to this network until another network is given
            os.system(f'nmcli connection modify {self.ssid} connection.autoconnect yes')

            # Connect to the WiFi network
            os.system(f'nmcli dev wifi connect {self.ssid} password {self.password}')
        else:
            print("Unsupported Kernel.")
            return

        self.num_motors = int(input("Enter the number of motors: "))
        self.battery_voltage = float(input("Enter the battery voltage: "))
        self.battery_capacity = float(input("Enter the battery capacity: "))

        if self.num_motors == 1:
            self.os_level = 1
            print("OS Level 1: Rear wheel drive without any steering system.")
        elif self.num_motors == 2:
            self.os_level = 2
            print("OS Level 2: Front steering and rear wheel drive.")
        else:
            self.os_level = 3
            print("OS Level 3: Individual motors for each wheel and front steering.")

        self.confirm_and_download()

    def confirm_and_download(self):
        """
        Confirm the OS level and download the necessary dependencies.
        """
        response = input(f"The OS Level {self.os_level} is supported as per your hardware. Do you wish to continue and download the operating system dependencies? (yes/no): ")
        if response.lower() == 'yes':
            print("Downloading dependencies...")
            # Download the necessary dependencies
        else:
            print("Setup cancelled.")