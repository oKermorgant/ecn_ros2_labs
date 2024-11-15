from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('mode', 'velocity')
sl.declare_arg('gui', True)


def launch_setup():

    mode = sl.arg('mode')

    gz_args = ''  # run the simulation unpaused
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('lab5_gazebo', 'world.sdf'), gz_args)

    name = 'turret'

    with sl.group(ns = name):

        sl.robot_state_publisher('lab5_gazebo', 'turret.xacro',
                                 xacro_args = sl.arg_map('mode'))

        sl.spawn_gz_model(name)

        # build bridges
        bridges = [GazeboBridge.clock()]

        # cmd_vel for joints with explicit GazeboBridge
        for joint in ('joint1', 'joint2', 'joint3'):

            if mode == 'velocity':
                bridges.append(GazeboBridge(f'{name}/{joint}_cmd_vel', f'{joint}_cmd_vel',
                                        'std_msgs/Float64', GazeboBridge.ros2gz))
            else:
                bridges.append(GazeboBridge(f'/model/turret/joint/{joint}/0/cmd_pos',
                                            f'{joint}_cmd_pos',
                                            'std_msgs/Float64',
                                            GazeboBridge.ros2gz))

        # joint state feedback on /world/<world>/model/<model>/joint_state
        gz_js_topic = GazeboBridge.model_prefix(name) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

        # image with lazy construction (no GazeboBridge, only tuple with arguments)
        bridges.append(GazeboBridge(f'{name}/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

        # ground truth pose if not fixed joint world -> base_link in URDF
        #bridges.append(GazeboBridge(f'/model/{name}/pose', 'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        #sl.node('pose_to_tf', parameters={'child_frame': f'{name}/base_link'})

        #sl.create_gz_bridge(bridges)

        # manual control
        sl.node('slider_publisher', arguments = [sl.find('lab5_gazebo', mode + '_manual.yaml')])


    # also display in RViz
    sl.rviz(sl.find('lab5_gazebo', 'layout.rviz'))

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
