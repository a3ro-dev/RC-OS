# Operating System for RC Car

This repository contains the source code for a custom operating system designed specifically for radio-controlled (RC) cars. The operating system is lightweight, efficient, and provides a high level of control over the RC car's functionality.

## Setup Instructions

Setting up this operating system on your RC car involves the following steps:# Operating System for RC Car

This repository contains the source code for a custom operating system designed specifically for radio-controlled (RC) cars. The operating system is lightweight, efficient, and provides a high level of control over the RC car's functionality.

## Hardware Requirements

To set up this operating system on your RC car, you will need the following hardware:

- 2 or 1 rwd Motor
- 1 Servo Motor (3-pin)
- 1 RC Car Chassis
- 1 Motor Driver (compatible with DC Motor, e.g., L289N)
- 1 Web Browser Enabled Device
- 1 Battery Pack (compatible with DC Motor)
- Various Wires and Connectors
- 1 Raspberry Pi Pico (W) or any other similar microcontroller

## Setup Instructions

1. **Prepare the Hardware**: Connect the DC motors to the motor driver, and connect the motor driver to the Raspberry Pi Pico. Connect the servo motor to the appropriate pins on the Raspberry Pi Pico. Ensure that all connections are secure.

2. **Clone the Repository**: Clone this repository to your local machine using the following command in your terminal:
    ```bash
    git clone github.com/a3ro-dev/RC-OS/
    ```

3. **Compile the Source Code**: Navigate to the directory containing the source code (`level_2.py`) and compile it. Use thonny or any other MicroPico Extension(vscode) to compile the code.

4. **Transfer the Compiled Code**: Transfer the compiled code to the Raspberry Pi Pico.

5. **Install the Operating System**: With the compiled code on the Raspberry Pi Pico

6. **Configure the Operating System**: Once installed, you'll need to configure the source-code to suit your specific RC car. This might involve setting control parameters, duty cycles, adjusting speed limits, etc.

7. **Test the Setup**: Finally, test the setup to ensure everything is working as expected. This might involve running some test drives, checking the response of the car to different commands, etc.

Please refer to the `OS-LEVEL-2.md` file for more detailed instructions and information about the operating system.

## Support

If you encounter any issues during the setup process, please open an issue in this repository. We're here to help!

## Contributing

Contributions to this project are welcome! Please refer to the `CONTRIBUTING.md` file for more information on how to contribute.

## License

This project is licensed under the MIT License. Please see the `LICENSE` file for more details.
