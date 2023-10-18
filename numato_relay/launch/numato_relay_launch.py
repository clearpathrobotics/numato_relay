from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  numato_relay_node = Node(
      package='numato_relay',
      executable='service'
  )

  ld = LaunchDescription()
  ld.add_action(numato_relay_node)

  return ld
