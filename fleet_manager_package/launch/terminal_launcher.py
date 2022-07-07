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

    res = False
    while not res:
        res = True
        inp = input("Insert scenario model name: ")
        if inp == 'quit':
            print('Launcher exited\n')
            exit()
        inp = inp.rstrip()
        if not os.path.exists(os.path.join(pkg_dir, 'models', 'scenarios', inp)):
            print("ERROR: models/" + inp + " model directory not found in share directory\n")
            res = False
            continue
        else:
            f.write(
                "<model name=\"scenario\"> <static>1</static> <include> <uri>model://" + inp + "</uri></include></model>")
        world_yaml = os.path.join(inp, inp + ".yaml")
        if not os.path.exists(os.path.join(pkg_dir, 'maps', world_yaml)):
            print("ERROR: maps/" + world_yaml + " file not found in share directory\n")
            res = False

    robots = []

    print("Enter bot model name or 's' to start simulation")
    id = 0
    while True:
        inp = input("bot model name|s: ")
        if inp == 's':
            break
        if inp == 'quit':
            print('Launcher exited')
            exit()
        inp = inp.rstrip()
        if not os.path.exists(os.path.join(pkg_dir, 'models', 'bots', inp)):
            model_name = 'turtlebot3_burger'
            # print("ERROR: bot model file not found")
            # continue
        else:
            model_name = inp
        res = False
        while not res:
            res = True
            inpx = input("x: ")
            try:
                float(inpx)
            except:
                print('Failed to convert x into a float')
                res = False
                continue
            inpy = input("y: ")
            try:
                float(inpy)
            except:
                print('Failed to convert y into a float')
                res = False

        res = False
        while not res:
            res = True
            inpth = input("th: ")
            try:
                float(inpth)
            except:
                print('Failed to convert th into a float')
                res = False

        name = 'bot' + str(id)
        id += 1
        robots.append(
            {'name': name, 'x_pose': inpx, 'y_pose': inpy, 'z_pose': '0.01', 'model': model_name, 'th': inpth})

    f.write('</world>')
    f.write('</sdf>')
    f.close()

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
    ld.add_action(central_node)

    return ld
