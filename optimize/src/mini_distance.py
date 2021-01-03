import numpy as np
import math
import quadprog
from parameters import veh
import prep_track


def mini_distance(trk: np.ndarray) -> np.ndarray:

    track_smooth_xy_cl = np.vstack((trk[:, :2], trk[0, :2]))
    cx, cy, ca, vect = prep_track.src.spline_coeffs.spline_coeffs(track=track_smooth_xy_cl)

    num_points = trk.shape[0]

    H = np.zeros((num_points, num_points))
    Q = np.zeros(num_points)

    for i in range(num_points):
        if i < num_points - 1:
            H[i, i] += 2 *(vect[i, 0]**2 + vect[i, 1]**2)
            H[i, i + 1] = -2* (vect[i, 0] * vect[i + 1, 0] +vect[i, 1] * vect[i + 1, 1])
            H[i + 1, i] = H[i, i + 1]
            H[i + 1, i + 1] = 2 *(vect[i + 1, 0]**2 + vect[i + 1, 1]**2)

            Q[i] += 2 * (vect[i, 0] * trk[i, 0] - vect[i, 0] * trk[i + 1, 0] +
                         vect[i, 1] * trk[i, 1] - vect[i, 1] * trk[i + 1, 1])

            Q[i + 1] = 2* (- vect[i + 1, 0] * trk[i, 0]- vect[i + 1, 1]* trk[i, 1]
                           +vect[i + 1, 0] * trk[i + 1, 0] + vect[i + 1, 1] * trk[i + 1, 1])

        else:
            H[i, i] += 2 *(vect[i, 0]**2 + vect[i, 1]**2)
            H[i, 0] = -2*(vect[i, 0] * vect[0, 0] + vect[i, 1] * vect[0, 1])
            H[0, i] = H[i, 0]
            H[0, 0] += 2* (vect[0, 0]**2 + vect[0, 1]**2)

            Q[i] += 2 * (vect[i, 0] * trk[i, 0] - vect[i, 0] * trk[0, 0]+
                         vect[i, 1] * trk[i, 1] - vect[i, 1] * trk[0, 1])
            Q[0] += 2* (-vect[0, 0] * trk[i, 0] - vect[0, 1] * trk[i, 1]+
                        vect[0, 0] * trk[0, 0] + vect[0, 1] * trk[0, 1])


    n_right = trk[:, 2] - veh.width / 2 * veh.k_safe
    n_left = trk[:, 3] - veh.width / 2 * veh.k_safe

    n_left[n_left < 0.001] = 0.001
    n_right[n_right < 0.001] = 0.001


    G = np.vstack((np.eye(num_points), -np.eye(num_points)))
    h = np.ones(2 * num_points) * np.append(n_right, n_left)

    epsilon = quadprog.solve_qp(H, -Q, -G.T, -h, 0)[0]

    return epsilon
