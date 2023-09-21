from simple_launch import SimpleLauncher
import yaml

# in this launch file SimpleLauncher is declared in the main body
sl = SimpleLauncher()

sl.declare_arg('robot', 'bb8', description = 'Robot name (bb8, bb9, d0, d9)')
sl.declare_arg('use_nav', 'true', description = 'Whether to use the nav stack or manual command')
sl.declare_arg('vx_max', 0.5)
sl.declare_arg('vx_min', 0.)

# the actual logic lies in a specific function where all launch arguments can be retrieved as Python types
def launch_setup():

    robot = sl.arg('robot')
    # extract robot type (bb / d)
    robot_type = ''.join(filter(str.isalpha, robot))

    with sl.group(ns = robot):

        # static_tf to True means the simulation will publish the map->odom ground truth: perfect localization
        sl.node('lab4_navigation', 'vel2joints.py', parameters = {'static_tf': True})

        # get robot type to generate description
        sl.robot_state_publisher('lab4_navigation', robot_type+'.xacro', 'urdf', xacro_args={'name': robot})

        if not sl.arg('use_nav'):
            cmd_file = sl.find('lab4_navigation', 'cmd_sliders.yaml')
            sl.node('slider_publisher', 'slider_publisher', name='cmd_vel_manual', arguments=[cmd_file])

        else:
            # nav stack

            # get required nav2 nodes depending on the current distro
            with open(sl.find('lab4_navigation', 'nav2_nodes.yaml')) as f:
                nav2_nodes = yaml.safe_load(f)[sl.ros_version()]
                node_names = [executable for pkg, executable in nav2_nodes]

            # get default configuration file from nav2 (copied into lab4 package)
            nav2_params = sl.find('lab4_navigation', f'nav2_params_{sl.ros_version()}.yaml')

            # TODO: adapt the default parameters to this robot: namespace, links, radius
            configured_params = nav2_params



            # TODO: remap some topics, some nav2 nodes assume a local map topic or an absolute scan topic
            remappings = {}

            # launch navigation nodes
            for pkg,executable in nav2_nodes:
                sl.node(pkg, executable,name=executable,
                    parameters=[configured_params],
                    remappings=remappings)

            # also run overall manager
            sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager',
            output='screen', parameters={'autostart': True,'node_names': node_names})

    return sl.launch_description()


# the actual launch description is a wrapper around the defined function
generate_launch_description = sl.launch_description(launch_setup)
    
