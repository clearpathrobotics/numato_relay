# numato_relay_interface

## Installation

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
