from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  numato_relay_node = Node(
    package='numato_relay',
    executable='service',
    parameters=[
      {
        'port': '/dev/ttyACM0',
        'baud': 19200
      }
    ]
  )

  ld = LaunchDescription()
  ld.add_action(numato_relay_node)

  return ld
