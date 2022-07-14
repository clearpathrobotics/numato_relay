# numato_relay_interface


## Installation

To manually install the udev rule:
```
sudo cp debian/udev /etc/udev/rules.d/41-numato.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

## Running the Driver

```
roslaunch numato_relay_interface numato_relay_interface.launch
```

OR

```
rosrun numato_relay_interface numato_relay_interface
```

### turning on relay 0:

```
rosservice call set_relay_0 "data: true"
```

### turning off relay 0:

```
rosservice call set_relay_0 "data: false"
```
