from simple_launch import SimpleLauncher

def generate_launch_description():
    '''    
    Example of AMCL (pure localization) for BB8
    '''

    sl = SimpleLauncher()
    
    robot = 'bb8'
        
    with sl.group(ns=robot):
                        
        sl.node('lab4_navigation', 'vel2joints.py', parameters = [{'static_tf': False}])
                       
        # generate description
        sl.robot_state_publisher('lab4_navigation', 'bb.xacro', 'urdf', xacro_args={'name': robot})

        # fire up slider publisher for cmd_vel
        cmd_file = sl.find('lab4_navigation', 'cmd_sliders.yaml')
        sl.node('slider_publisher', 'slider_publisher', name='cmd_vel_manual', arguments=[cmd_file])
        
        # launch AMCL node with parameter file
        # TODO add some remappings or change the topics in the param file
        sl.node('nav2_amcl', 'amcl',name='amcl',
                parameters=[sl.find('lab4_navigation', 'amcl_param.yaml')],
                arguments='--ros-args --log-level warn')

        # run lifecycle manager just for AMCL
        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager',
        parameters={'autostart': True, 'node_names': ['amcl']})
                    
    return sl.launch_description()
