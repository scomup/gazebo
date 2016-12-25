from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
import tf


def update_v(rows,cols,map_data,v_data):
    for i in range(rows):
        for j in range(cols):
            v_data[i,j] = map_data[i][j].h                      


v_data = np.zeros((100,100))
rospy.init_node('odom_listener', anonymous=True)
listener = tf.TransformListener()
fig,ax=plt.subplots()
ax.set_aspect('equal')
ax.pcolor(v_data,cmap=plt.cm.Reds,edgecolors='k')
plt.show()
