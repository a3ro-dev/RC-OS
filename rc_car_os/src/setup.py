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

