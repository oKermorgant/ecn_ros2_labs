from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value=sl.find('lab4_navigation', 'batS.yaml'))
    sl.declare_arg('map_server', default_value=True)
    sl.include('map_simulator', 'simulation2d_launch.py', launch_arguments= sl.arg_map('map','map_server'))
    
    # run RViz2
    rviz_config_file = sl.find('lab4_navigation', 'config.rviz')
    sl.node('rviz2','rviz2', output='log',
            arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'error'])

    return sl.launch_description()
