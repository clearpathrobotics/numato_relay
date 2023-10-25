# numato_relay

> [!IMPORTANT]  
> This branch is for [ROS 2 Humble](https://docs.ros.org/en/humble/index.html).

> [!NOTE]  
> This driver has been tested with [Numato Labs](https://numato.com/) USB relay boards with 2, 4, 8 and relays.

<br />

## Installation

1.  Create a workspace and src directory, `~/ros2_ws/src/`.
2.  Clone this git repository to the directory `~/ros2_ws/src/`.
    You now have a directory structure of:

    ```bash
    ~/ros2_ws/
        └── src/
            └── numato_relay/
                ├── debian/
                |   ├── ...
                |   └── ...
                ├── numato_relay/
                |   ├── ...
                |   └── ...
                ├── numato_relay_interfaces/
                |   ├── ...
                |   └── ...
                └── README.md
    ```

3.  Install the udev rule:
    ```
    sudo cp ~/ros2_ws/src/numato_relay/debian/udev /etc/udev/rules.d/41-numato.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
4.  Build the workspace:
    ```
    cd ~/ros2_ws
    colcon build
    ```

5.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```

<br />

## Usage

### Running the Driver

1.  Open a new terminal.
2.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```
3.  Start the node with either:

    ```
    ros2 launch numato_relay numato_relay_launch.py
    ```

    or

    ```
    ros2 run numato_relay service
    ```

    > [!NOTE]  
    > `ros2 run` may have issues if the Numato is connected as a device other than the default `/dev/ttyACM0`.
    > The launch file includes line to change this parameter.

<br />

### Controlling a relay

After starting the package per the steps in Running the driver:
1.  Open a new terminal.
2.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```
3.  Calling the Service
    ```
    ros2 service call set_relay numato_relay_interfaces/srv/SetRelay "{relay_channel: 3, relay_state: False}"
    ```
    
    -   Changing the integer beside `relay_channel:` to match the relay that you want to control.
    -   Changing the boolean beside `relay_state:` to:
        -  True for closing the relay (_powered_)
        -  False for opening the relay (_unpowered_)


<br />

### Topics and Services

A topic is created for each available relay.
`numato_relay.py` structures 8 publishers, but only publishes the ones that are supported on the connected Numato PCBA.
So, with a 4 channel PCBA, only topics 0 - 3 will be published.

Each topic publishes a boolean:
-   True = on = relay closed = relay powered
-   False = off = relay open = relay not powered

<br />

The topic names are:
-   `/numato_relay_state_0`
-   `/numato_relay_state_1`
-   `/numato_relay_state_2`
-   `/numato_relay_state_3`
-   `/numato_relay_state_4`
-   `/numato_relay_state_5`
-   `/numato_relay_state_6`
-   `/numato_relay_state_7`

<br />

## Hardware details
-   Use a Numato Labs USB relay board:
    | Numato Labs item                                                  | Description          | Cable Required                 |
    | :---------------------------------------------------------------- | :------------------- | :----------------------------- |
    | [RL20001](https://numato.com/product/2-channel-usb-relay-module/) | 2 relay channel PCBA | USB 2.0, Type-A to Type-B-mini |
    | [RL40001](https://numato.com/product/4-channel-usb-relay-module/) | 4 relay channel PCBA | USB 2.0, Type-A to Type-B-mini |
    | [RL80001](https://numato.com/product/8-channel-usb-relay-module/) | 8 relay channel PCBA | USB 2.0, Type-A to Type-B      |

-   Computer with Ubuntu and ROS 2.
    This has been tested with Ubuntu 22.04 Jammy Jellyfish, and ROS 2 Humble.
-   The Numato PCBA connects as device `/dev/ttyACM0`.
-   The Numato PCBA communicates at 19200 baud.
