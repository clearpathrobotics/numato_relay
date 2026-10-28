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

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | Serial device path. |
| `baud` | `19200` | Serial baud rate. |
| `toggle_duration` | `2.0` | Seconds to hold the opposite state during a power-cycle request. |
| `relay_timer_period` | `1.0` | Relay state publishing period in seconds. |
| `gpio_timer_period` | `3.0` | GPIO polling/publishing period in seconds. GPIO polling remains disabled. |

Set parameters at startup with `--ros-args -p`, for example:

```
ros2 run numato_relay service --ros-args -p relay_timer_period:=0.5
```

### Controlling a relay

The `SetRelay` request contains a zero-based `uint32 relay_channel` and the desired
`bool relay_state` (`True` = on/closed, `False` = off/open). The response contains
`bool success`, `bool relay_state`, and `string message`. On success, `relay_state`
is the driver's tracked state after the serial command has been written and flushed;
the hardware is not read back for confirmation. On failure, `success` is false and
`relay_state` is a placeholder false value, not an indication of the physical state.
Invalid channels are logged and reported in `message`; serial I/O failures are logged
and returned as failed responses. If the cached state already matches the requested
state, the service returns success without sending a serial command. This optimization
relies on the driver's cached state and cannot detect changes made directly to the
hardware or by another controller.

**Interface migration:** `relay_channel` changed from `uint8` to `uint32`, and the
response fields changed from `relay_response_bool`/`relay_response_string` to
`success`/`relay_state`/`message`. Rebuild the interface package and update and rebuild
dependent clients to use the new response fields. The service name remains `set_relay`.

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

### Reading a relay

Call the `get_relay` service to retrieve the driver's tracked state without
sending a command to the hardware:

```
ros2 service call get_relay numato_relay_interfaces/srv/GetRelay "{relay_channel: 3}"
```

The response uses the same `success`, `relay_state`, and `message` fields as
`SetRelay`. The state is driver-tracked, not confirmed by reading back the relay
hardware. When `success` is false, `relay_state` is a placeholder and must not
be treated as the physical state.

### Power-cycling a relay

Call `toggle_relay` with the desired final state:

```
ros2 service call toggle_relay numato_relay_interfaces/srv/ToggleRelay "{relay_channel: 0, relay_state: True}"
```

If the relay is already in the requested state, it switches to the opposite
state for `toggle_duration` seconds and then restores the requested state. If
the relay is not already in that state, it is set directly without a power-cycle.
`toggle_duration` is configured at startup (default `2.0` seconds), for example
with `--ros-args -p toggle_duration:=5.0`.

The timed power-cycle blocks the node's executor, pausing other services and
timer callbacks. Serial failures return `success: false` and an explanatory
message. In particular, if restoring the state fails, the relay may remain in
the opposite state; the response's `relay_state` is then a failure placeholder,
not the physical state.


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
