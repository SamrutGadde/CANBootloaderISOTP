## An STM32F4 Bootloader over CAN Bus!

### Overview
---

This project uses [CAN ISO-TP](https://en.wikipedia.org/wiki/ISO_15765-2) to do file transfer over CAN bus!
Courtesy of [this C library](https://github.com/SimonCahill/isotp-c) for the easy setup of ISO-TP on an MCU.

Using a simple python script, you can flash an STM32F4 based device!

### Dependencies
---

  - Linux kernel with SocketCAN
  - CAN Adapter (CANable or other)
  - STM32F407VET6 MCU Target
  - Platformio (for building projects)
  - Python 3.7+
  - A CAN bus (of course)

### How to Use
---

1. Build and flash your STM32F4 device with the given Bootloader Platformio project

2. Set up your CAN interface
    ```shell
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
    ```
3. Install Python dependencies
    ```shell
    pip install python-can
    pip install can-isotp
    ```
4. Run the script!
    ```shell
    python linux_send_isotp.py /path/to/your/binary.bin
    ```

### TODO
  - Add common API for app to jump back to bootloader
  - Optimize bootloader size
  - Add CRC verficiation





