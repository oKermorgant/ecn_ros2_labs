from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.node('slider_publisher', 'slider_publisher', arguments = [sl.find('move_joint', 'single_joint.yaml')])
    
    return sl.launch_description() 
