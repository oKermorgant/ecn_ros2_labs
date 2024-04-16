from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('namespace', default_value='turret')
sl.declare_arg('gazebo_world_name', 'none')


def launch_setup():

    # run Gazebo
    sl.gz_launch('empty.sdf')


    # spawn turret
    with sl.group(ns='turret'):

        sl.robot_state_publisher('lab5_gazebo', 'turret.urdf')
                    
        # URDF spawner to Gazebo, defaults to relative robot_description topic
        sl.spawn_gz_model('turret')
    
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
