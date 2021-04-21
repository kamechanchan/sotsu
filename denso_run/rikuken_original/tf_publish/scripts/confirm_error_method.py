#!/usr/bin/env python3
from pyquaternion import Quaternion
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation 
from tf.transformations import quaternion_matrix, euler_matrix
import tf


q = tf.transformations.quaternion_from_euler(ai, aj, ak)
my_quat = Quaternion(axis=[0, 1, 0], angle=pi)
v = np.array([0, 0, 1])
print(v)
v_prime = my_quat.rotate(v)
print(v_prime)
u = my_quat.axis
print(u)
R = my_quat.rotation_matrix
print(R)
