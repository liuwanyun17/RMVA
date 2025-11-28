from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sphere_vision_node = Node(
        package='player_pkg',
        executable='sphere_vision_node',
        name='sphere_vision_node',
    )

    rec_vision_node = Node(
        package='player_pkg',
        executable='rec_vision_node',
        name='rec_vision_node',
    )  

    armor_vision_node = Node(
        package='player_pkg',
        executable='armor_vision_node',
        name='armor_vision_node'
    )       

    multi_armor_vision_node = Node(
        package='player_pkg',
        executable='multi_armor_vision_node',                       
        name='multi_armor_vision_node'
    )
    
    multi_armor_vision_node1 = Node(
        package='player_pkg',
        executable='multi_armor_vision_node1',                       
        name='multi_armor_vision_node1'
    )
    
    race_stage_subscriber_node = Node(
        package='player_pkg',
        executable='race_stage_subscriber',
        name='race_stage_subscriber_node'
    )
    return LaunchDescription([
        sphere_vision_node,
        rec_vision_node,
        armor_vision_node,
        multi_armor_vision_node
    ])           