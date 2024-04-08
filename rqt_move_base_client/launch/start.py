from launch import LaunchDescription
from launch_ros.actions import Node

# Note that this is on Eloquent
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='rqt_move_base_clietn',
      node_executable='rqt_move_base_client',
      node_name='rqt_move_base_client',
      output='screen'
    )
  ])
