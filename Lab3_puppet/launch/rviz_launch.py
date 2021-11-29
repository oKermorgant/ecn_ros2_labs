from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.node('rviz2', 'rviz2', arguments = ['-d',sl.find('lab3_puppet', 'config.rviz')])
    
    sl.include('baxter_description', 'baxter_state_publisher_launch.py')
    
    return sl.launch_description() 
