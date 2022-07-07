import os
from re import T

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    try:
        pkg_dir = get_package_share_directory('fleet_manager_package')
    except:
        print("ERROR: fleet_manager_package share directory not found\n")
        exit()
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_dir, 'models', 'scenarios') + ":" + os.path.join(pkg_dir,
                                                                                                        'models',
                                                                                                        'bots')
    if not os.path.exists(os.path.join(pkg_dir, 'models')):
        print("ERROR: model path not found in share directory\n")
        exit()

    # select scenario
    f = open(os.path.join(pkg_dir, 'worlds', 'tmp.model'), 'a')
    f.truncate(0)

    if not os.path.exists(os.path.join(pkg_dir, 'worlds', 'empty_world.model')):
        print("ERROR: worlds/empty_world.model file not found in share directory\n")
        exit()
    empty_world = open(os.path.join(pkg_dir, 'worlds', 'empty_world.model'), 'r')

    inp = 'a'
    while inp != '':
        inp = empty_world.readline()
        if inp == '</world>\n':
            break
        f.write(inp)
    empty_world.close()
    print("Enter 'quit' anytime to quit the launcher")

    if not os.path.exists(os.path.join(pkg_dir, 'models', 'scenarios', 'povo')):
        print("ERROR: models/povo model directory not found in share directory\n")
    else:
        f.write(
            "<model name=\"scenario\"> <static>1</static> <include> <uri>model://povo</uri></include></model>")

    robots = []
    inp = input("Enter number of robots: ")
    # read position from file
    with open('/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/fleet_manager_package/maps/povoGraph.txt') as graph:
        graph.readline()

        for i in range(int(inp)):
            # read from file
            line = graph.readline()
            coordinates = line.split(' ')

            model_name = 'turtlebot3_burger'
            try:
                float(coordinates[0])
            except:
                print('Failed to convert x into a float')
            try:
                float(coordinates[1])
            except:
                print('Failed to convert y into a float')
            try:
                float(coordinates[2])
            except:
                print('Failed to convert th into a float')
                res = False

            name = 'bot' + str(i)
            robots.append(
                {'name': name, 'x_pose': coordinates[0], 'y_pose': coordinates[1], 'z_pose': '0.01', 'model': model_name, 'th': coordinates[2]})

    f.write('</world>')
    f.write('</sdf>')
    f.close()

    print(robots)
    print('fin qui')


    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'tmp.model'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    robot_cmds = []
    spawn_cmds = []
    for robot in robots:
        robot_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'robot_nodes_launch.py')),
                launch_arguments={
                    'namespace': robot['name']
                }.items()
            )
        )

        spawn_cmds.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace=robot['name'],
                arguments=[
                    '-entity', robot['name'],
                    '-robot_namespace', robot['name'],
                    '-x', robot['x_pose'],
                    '-y', robot['y_pose'],
                    '-z', robot['z_pose'],
                    '-Y', robot['th'],
                    '-file', os.path.join(pkg_dir, 'models', 'bots', robot['model'], 'model.sdf')
                ]
            )
        )

    telegram_bot_node = Node(
        package='telegram_bot_package',
        executable='telegram_bot_node',
        name='telegram'
    )

    fleet_manager_node = Node(
        package="fleet_manager_package",
        executable="fleet_manager_node",
        name="fleetmanager",
        parameters=[{"n_robots": len(robots)}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for cmd in robot_cmds:
        ld.add_action(cmd)
    # for cmd in plansys2_cmds:
    #    ld.add_action(cmd)

    for cmd in spawn_cmds:
        ld.add_action(cmd)

    ld.add_action(telegram_bot_node)
    ld.add_action(fleet_manager_node)

    return ld
