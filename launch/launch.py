from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_stamped_vel = LaunchConfiguration('use_stamped_vel')

    # Declare arguments
    declare_use_stamped_vel_arg = DeclareLaunchArgument(
        'use_stamped_vel', default_value='false',
        description='Flag of use stamped twist'
    )

    return LaunchDescription([
        declare_use_stamped_vel_arg,
        Node(
            package='whi_custom_interaction_example',
            executable='whi_custom_interaction_example',
            name='whi_custom_interaction_example',
            parameters=[
                {'use_stamped_vel': LaunchConfiguration('use_stamped_vel')} # do not define in yaml if it is dynamic through argument
            ],
            output='screen'
        )
    ])
