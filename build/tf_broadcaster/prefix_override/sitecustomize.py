import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/csrobot/workspaces/ur_gazebo/src/install/tf_broadcaster'
