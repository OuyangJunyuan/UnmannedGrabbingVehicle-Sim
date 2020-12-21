import rospy
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import PointCloud2 as pc2
import time

a = tuple({(1, 1), (2, 2), (3, 3)})
b = [1,2,3,4]
c = np.array(a)
print(c[c[:, 1] > 2])
