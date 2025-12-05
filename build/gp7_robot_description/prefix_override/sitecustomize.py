import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pankaja/projects/Arm_Simulation/install/gp7_robot_description'
