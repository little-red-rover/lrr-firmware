# Little Red Rover Firmware

Firmware for the ESP32-S3-MINI-1 built into the rover's body.

This project uses the Espressif IoT Development Framework (espIDF) [read more](https://github.com/espressif/esp-idf).

## Birds Eye View

Little Red Rover's firmware is responsible for three tasks:
1. Establishing Wi-Fi connectivity
2. Reading information from sensors (LiDAR, IMU, wheel encoders)
3. Writing commands to actuators (wheel motors)
4. Relaying sensor data and recieving commands over a TCP connection

Because all high level algorithms are run on the base station computer (see [lrr_ros](https://github.com/little-red-rover/lrr-ros)), the rover itself has very little control logic.
Instead, it's optimized for network transmisison and reception.

## API

The rover hosts a TCP server listening on port 8001.
When connected to the rover's hotspot, the ESP32-S3 has the IP address `192.168.4.1`, making the TCP server address `192.168.4.1:8001`.

Send commands using the format defined in the protobuf schema [messages.proto](https://github.com/little-red-rover/lrr-firmware/blob/16-refactor-as-c%2B%2B/components/communication_stack/socket_manager/messages.proto).

To recieve data, the client must first send a SubscribeRequest message with the relevant OutgoingMessageID.
The rover will then begin sending data to that port.

TODO: update link on C++ PR merge.

## Composite Technologies

### FreeRTOS

The system is based on FreeRTOS, a realtime embedded operating system (RTOS).
FreeRTOS provides scheduling and mutlithreading utilities that are crucial for running efficiently in an asynchronous networking environment.

### NanoPB

In order for data to be transmitted over the network, it needs to be serialized appropriately.
This project uses [Google's ProtocolBuffers](https://protobuf.dev/).
Specifically, the firmware uses an embedded focused implementation of ProtocolBuffers called [NanoPB](https://jpa.kapsi.fi/nanopb/).

## Project Structure

* `/main`
  * `lrr_main.cc`: The firmware entry point. Initializes drivers.
* `/components`
  * `/communication_stack`
      * `/wifi_manager`:
         * Initializes wifi
         * Handles provisioning (connecting to Wi-Fi using remotely provided credentials)
         * Connects to Wi-Fi
         * Sets up NAPT so that users can connect to the internet through the rover
      * `/socket_manager`:
          * Multithreaded TCP server implementation
          * Manages multiple client connections, data subscriptions, and general network IO
  * `/hardware_drivers`
      * `/battery`:
          * Measures battery using the ADC
          * Telemeters battery level data
      * `/drive_base`:
          * Integrates encoder pulses using the ESP32-S3 pulse counter module (PCNT) 
          * Drives PID control for the motors
          * Accepts joint state commands as JointCmd messages (see protobuf message definition)
          * Telemeters joint state information as JointState messages
      * `/imu`:
          * Reads data from the IMU over I2C
          * Telemeters IMU data as IMU messages
      * `/lidar`:
          * Reads data from the LD20 LiDAR over UART
          * Telemeters LiDAR data as LaserScan messages
      * `/status_led`:
          * Provides a function to set the status LEDs in response to system events

## Development Environment

The `/docker` and `/.devcontainer` folder provide a devcontainer environment for development.
The docker container uses privilaged mode to flash the ESP32-S3 over USB. The privilaged flag is not supported on MacOS or Windows, so this may not work for you.

Cross platform build instructions are a WIP.
