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

        self.battery_size=int(input("please enter the battery size-- "))
        self.battery_voltage=float(input("Input the voltage of the battery in your car: "))
        self.number_of_motors= int(input("please enter the number of motors in"))