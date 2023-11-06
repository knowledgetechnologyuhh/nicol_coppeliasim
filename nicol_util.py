# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

import numpy as np

from nicol_base import NicolPose

def get_random_pose(left=False):
    if left:
        x = np.random.uniform(0.25, 0.6)
        y = np.random.uniform(0.0, 0.55)
        z = np.random.uniform(0.9, 1.3)
    else:
        x = np.random.uniform(0.25, 0.6)
        y = np.random.uniform(0.0, -0.55)
        z = np.random.uniform(0.9, 1.3)

    x_orientation = 1.0
    y_orientation = 1.0

    while (x_orientation ** 2 + y_orientation ** 2 > 1.0):
        x_orientation = np.random.uniform(-1.0, 1.0)
        y_orientation = np.random.uniform(-1.0, 1.0)

    z_orientation = x_orientation ** 2 + y_orientation ** 2

    u_orientation = 1.0
    v_orientation = 1.0

    while (u_orientation ** 2 + v_orientation ** 2 > 1.0):
        u_orientation = np.random.uniform(-1.0, 1.0)
        v_orientation = np.random.uniform(-1.0, 1.0)

    w_orientation = u_orientation ** 2 + v_orientation ** 2

    s_orientation = np.sqrt((1 - z_orientation) / w_orientation)

    # return random Quaternion
    return NicolPose(position=[x, y, z], orientation=[0,0,np.pi/4, np.pi/4])
    #return NicolPose(position=[x, y, z], orientation=[x_orientation, y_orientation, s_orientation * u_orientation, s_orientation * v_orientation])
