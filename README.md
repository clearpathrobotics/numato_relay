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
| `toggle_duration` | `2.0` | Seconds to hold the opposite relay state during a toggle, configured at startup. |
| `relay_timer_period` | `1.0` | Relay state publishing period in seconds, configured at startup. |
| `gpio_timer_period` | `3.0` | GPIO polling/publishing period in seconds, configured at startup. GPIO polling remains disabled. |

Override parameters with `--ros-args -p`, for example:

```
ros2 run numato_relay service --ros-args -p relay_timer_period:=0.5
```

### Service contract

All three services use a zero-based `uint32 relay_channel`. `SetRelay` and
`ToggleRelay` also take `bool relay_state`, the desired final state
(True = on/closed, False = off/open).

All responses contain:

```text
bool success
bool relay_state
string message
```

- `success` means the operation completed; an already-satisfied set request also succeeds.
- `relay_state` is the driver's tracked final state, valid **only when `success` is True**.
  Writes are not verified by reading back the hardware.
- `message` is a human-readable result or error explanation, not a machine-readable status.
- Invalid channels and serial I/O failures return `success: False`, `relay_state: False`,
  and an explanatory message, and are logged. The False state on failure is a placeholder,
  not evidence that the physical relay is off.
- Tracked state is updated only after a command is written and flushed successfully.
  Failed writes may still have reached the hardware, so physical state may be uncertain.
  A failed restore during a toggle is reported as a failure; the relay may remain in
  the opposite state.

**Interface migration:** All request channel fields changed from `uint8` to `uint32`,
and `GetRelay` now returns `success` and `message` in addition to `relay_state`.
Rebuild the interface package and all dependent clients, and restart service nodes
and clients using the updated interfaces. Service names are unchanged. The wider
channel type does not extend the connected board's hardware capabilities.

### Controlling a relay

After starting the package per the steps in Running the driver:
1.  Open a new terminal.
2.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```
3.  Calling the Service
    ```
    ros2 service call /numato_relay/set_relay numato_relay_interfaces/srv/SetRelay "{relay_channel: 3, relay_state: False}"
    ```
    
    -   Changing the integer beside `relay_channel:` to match the relay that you want to control.
    -   Changing the boolean beside `relay_state:` to:
        -  True for closing the relay (_powered_)
        -  False for opening the relay (_unpowered_)
    -   The response has `success`, the resulting `relay_state`, and a `message`.
        `success` is True when the driver's tracked state ends in the requested state,
        including when it was already in that state.
        Requesting a channel the connected PCBA does not have returns `success: False` and logs a warning.

### Reading a relay

```
ros2 service call /numato_relay/get_relay numato_relay_interfaces/srv/GetRelay "{relay_channel: 3}"
```

-   On success, `relay_state` is True when the relay is tracked as closed (_powered_)
    and False when it is tracked as open (_unpowered_).
-   This returns the driver's tracked state, the same value at `states[relay_channel]` on `/numato_relay/relay_states`.
-   Requesting a channel the connected PCBA does not have returns `success: False`,
    `relay_state: False`, and an error message, and logs a warning.

### Power-cycling a relay

```
ros2 service call /numato_relay/toggle_relay numato_relay_interfaces/srv/ToggleRelay "{relay_channel: 0, relay_state: True}"
```

-   If the relay is already in the requested `relay_state`, it is switched to the opposite state, held there for `toggle_duration` seconds, then switched back.
-   If the relay is not in the requested `relay_state`, it is switched to it, with no toggle.
-   This means "ensure the desired state, power-cycling if already there," not simply "invert."
-   The response has `success`, the final `relay_state`, and a `message` describing what happened.
    Requesting a channel the connected PCBA does not have returns `success: False` and logs a warning.
-   `toggle_duration` is a node parameter (default `2.0` seconds), e.g. `ros2 run numato_relay service --ros-args -p toggle_duration:=5.0`.
    The node is blocked while the relay is held, so the state topics and other services pause for that time.


<br />

### Topics and Services

All relay states are published together on `/numato_relay/relay_states`, using
`numato_relay_interfaces/msg/RelayStates`:

```text
bool[] states
```

The array index is the zero-based relay channel. The array length is the number
of discovered relays; no separate length field is needed. There is no eight-channel
limit in the publisher. For example, a four-channel board might publish
`states: [true, false, false, true]`. An empty array means no channels were discovered.

Each array entry is a boolean:
-   True = on = relay closed = relay powered
-   False = off = relay open = relay not powered

GPIO uses `/numato_relay/gpio_states` with `numato_relay_interfaces/msg/GpioStates`,
also containing `bool[] states`, indexed by GPIO channel (True = high, False = low).
The GPIO publisher is created at startup, but periodic GPIO polling/publishing
remains disabled.

**Migration:** The individual `numato_relay_state_N` and `numato_gpio_state_N`
topics have been removed. Subscribers must use the array topics and select
`states[N]` after checking the array length. Service names are unchanged; see the
service contract above for interface changes.

To inspect relay states:

```
ros2 topic echo /numato_relay/relay_states
```

<br />

All topics and services are private to the node, named `numato_relay`, so they are under
`<namespace>/numato_relay/`. Without a namespace the services are `/numato_relay/set_relay`,
`/numato_relay/get_relay` and `/numato_relay/toggle_relay`. To run it under a namespace, e.g. `r100_0169`:
```
ros2 run numato_relay service --ros-args -r __ns:=/r100_0169
```
which gives `/r100_0169/numato_relay/set_relay`, `/r100_0169/numato_relay/relay_states`, etc.

The topic names (without a namespace) are:
-   `/numato_relay/relay_states`
-   `/numato_relay/gpio_states` (periodic publishing disabled)

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
