from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
        
    capture_key_remap = {'/key_typed': 'key_typed'}
    sl.node('capture_key', 'capture_key', output='screen', remappings = capture_key_remap.items())
    
    move_joint_params = {}
    move_joint_params['joint_name'] = 'left_s0'
    move_joint_remap = {'/key_hit': 'key_typed', '/joint_command': '/robot/limb/left/joint_command'}
    
    sl.node('move_joint', 'move_joint', output='screen', 
            parameters = [move_joint_params], remappings = move_joint_remap.items())
    
    return sl.launch_description()
