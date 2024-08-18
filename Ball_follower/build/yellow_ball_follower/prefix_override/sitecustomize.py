import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohamad/Inmind/Robotics-Session13-Assignment/Ball_follower/install/yellow_ball_follower'
