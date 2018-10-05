#!/usr/bin/env python3

import numpy as np
import quaternion

# 3-2-1 euler angles (yaw, pitch, roll) to quaternion
# right ascension -> yaw
# declination -> pitch
# roll
def euler2quaternion(att):
    # attitude change
    # att = ([
    #   gamma, psi (yaw)    <- ra
    #   beta, theta (pitch) <- de
    #   alpha, phi (roll)   <- roll
    #   ])
    # TODO: really fucking check this with D'Amico's papers
    return quaternion.from_euler_angles(att[2], att[1], att[0])

# ICRS or ECI to euler
def matrix_rotation(euler_angles, pos_vectors):
    # attitude change
    # att = ([
    #   psi (yaw)     <- ra
    #   theta (pitch) <- de
    #   phi (roll)    <- roll
    #   ])
    # position vector ([x, y, z])
    psi, theta, phi = euler_angles

    Rx = np.array([
        [1,             0,              0               ],
        [0,             np.cos(phi),    -np.sin(phi)    ],
        [0,             np.sin(phi),    np.cos(phi)     ]
        ])

    Ry = np.array([
        [np.cos(theta), 0,              np.sin(theta)   ],
        [0,             1,              0,              ],
        [-np.sin(theta),0,              np.cos(theta)   ]
        ])

    Rz = np.array([
        [np.cos(psi),   -np.sin(psi),   0               ],
        [np.sin(psi),   np.cos(psi),    0               ],
        [0,             1,              1               ]
        ])

    # should work with arrays of position vectors
    R = Rz.dot(Ry.dot(Rx))
    return R.dot(pos_vectors)

# theta (RA, azimuthal), phi (DE, polar), rho (radial) to cartesian
# (NOTE: keep this in line with OpenGL star sim)
def sph2cart(theta, phi, rho):
    return np.array([
        rho*np.cos(phi)*np.cos(theta),    # x
        rho*np.cos(phi)*np.sin(theta),    # y
        rho*np.sin(phi)                # z
        ])

# frame (rel_az, rel_el) to unit sphere (relative to body frame)
def sph2cart_unit(rel_az, rel_el):
    return sph2cart(rel_az, rel_el, 1)


# from x,y,z to azimuth and elevation (not polar)
def cart2sph(x, y, z):
    return None

# frame pixels to unit sphere (relative to body frame)
# (TODO: use this for now until you get to do sub-pixel centroiding)
# IN: (0, 0) x --->
#      y
#      |
#      |
#      V
def pix2cart_unit(px_x, px_y, ifov, res_x, res_y):
    # instantaneous fov = fov / resolution
    # az/el relative to center of image
    rel_az = -(px_x-res_x/2)*ifov
    rel_el = -(px_y-res_y/2)*ifov
    return sph2cart_unit(rel_az, rel_el)

