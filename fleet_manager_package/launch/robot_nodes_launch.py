from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='robot namespace')

    motion_cmd = Node(
        package='motion_planner',
        executable='dubins',
        name='motion_dubins',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    # parameters=[{'name': namespace}]

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(motion_cmd)

    return ld
