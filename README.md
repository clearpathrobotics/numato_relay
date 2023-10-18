# numato_relay_interface

> [!IMPORTANT]  
> This branch is for [ROS Noetic](http://wiki.ros.org/noetic), and is not actively maintained.

> [!NOTE]  
> This driver has been tested with [Numato Labs](https://numato.com/) USB relay boards with 2, 4, 8 and relays.

<br />

## Installation

1.  To manually install the udev rule:
    ```
    sudo cp debian/udev /etc/udev/rules.d/41-numato.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
2.  Clone this repository to your ROS workspace's `src` folder, and build the workspace.

<br />

## Usage

### Running the Driver

```
roslaunch numato_relay_interface numato_relay_interface.launch
```

OR

```
rosrun numato_relay_interface numato_relay_interface
```

#### turning on relay 0:

```
rosservice call set_relay_0 "data: true"
```

#### turning off relay 0:

```
rosservice call set_relay_0 "data: false"
```

<br />

## Topics and Services

The node provides 8 numbered topics and 8 numbered services for querying and controlling the relays.  Note that
not all Numato relay boards have 8 relays on them, so some topics/services may not do anything.

* `set_relay_N` (`std_srvs/SetBool`) The service used to control the state of relay N, where N is a number from 0 to 7
* `relay_state_N` (`std_msgs/Bool`) The topic that publishes the current state of relay N, where N is a number from 0
  to 7.

For easy of use, it is recommended to use the `remap` feature of `roslaunch` to change the topics and services to
something semantically meaningful for your application.  For example:

```
<node pkg="numato_relay_interface" type="numato_relay_interface" name="numato_relay_control_server">
  <param name="port" value="$(arg port)" />
  <param name="baud" value="$(arg baud)" />
  <param name="check_on_write" value="false" />

  <!--
    Pins 0-3 are used for the stack light
  -->
  <remap from="set_relay_0" to="/lights/set_strobe_on" />
  <remap from="relay_state_0" to="/lights/strobe_on" />
  <remap from="set_relay_1" to="/lights/set_stack_red" />
  <remap from="relay_state_1" to="/lights/stack_red_on" />
  <remap from="set_relay_2" to="/lights/set_stack_grn" />
  <remap from="relay_state_2" to="/lights/stack_grn_on" />
  <remap from="set_relay_3" to="/lights/set_stack_blu" />
  <remap from="relay_state_3" to="/lights/stack_blu_on" />

  <!--
    Pin 4 controls the backup alarm
  -->
  <remap from="set_relay_4" to="/backup_alarm/set_sound_on" />
  <remap from="relay_state_4" to="/backup_alarm/sounding" />

  <!--
    Pins 5 and 6 control the driving lights
  -->
  <remap from="set_relay_5" to="/lights/set_front_on" />
  <remap from="relay_state_5" to="/light/front_on" />
  <remap from="set_relay_6" to="/lights/set_rear_on" />
  <remap from="relay_state_6" to="/lights/rear_on" />
</node>
```

<br />

## Parameters

The node checks the following `rosparam` values:
* `~port` (default: `/dev/ttyACM0`) The serial device to open.
* `~baud` (default: `19200`) The baud rate for the serial connection
* `~check_on_write` (default: `true`) If true, after writing a relay state the state is read back to confirm it was
  set correctly.  Disabling this will dramatically speed up response times, but risks leaving errors uncaught.
* `~gpio_to_read` (default: `[]`) GPIO pins to read from the board
* `~freq` (default: `20.0`) The frequency in Hz that the GPIO pins are read (if configured) and the current relay states
  are published on their corresponding topics