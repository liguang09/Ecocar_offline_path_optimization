import numpy as np
import math
import quadprog
from parameters import veh


def mini_distance(track: np.ndarray,
                  vectors: np.ndarray
                  ) -> np.ndarray:


    num_points = track.shape[0]

    H = np.zeros((num_points, num_points))
    Q = np.zeros(num_points)

    for i in range(num_points):
        if i < num_points - 1:
            H[i, i] += 2 * (math.pow(vectors[i, 0], 2) + math.pow(vectors[i, 1], 2))
            H[i, i + 1] = 0.5 * 2 * (-2 * vectors[i, 0] * vectors[i + 1, 0]
                                     - 2 * vectors[i, 1] * vectors[i + 1, 1])
            H[i + 1, i] = H[i, i + 1]
            H[i + 1, i + 1] = 2 * (math.pow(vectors[i + 1, 0], 2) + math.pow(vectors[i + 1, 1], 2))

            Q[i] += 2 * vectors[i, 0] * track[i, 0] - 2 * vectors[i, 0] * track[i + 1, 0] \
                    + 2 * vectors[i, 1] * track[i, 1] - 2 * vectors[i, 1] * track[i + 1, 1]
            Q[i + 1] = -2 * vectors[i + 1, 0] * track[i, 0] \
                       - 2 * vectors[i + 1, 1] * track[i, 1] \
                       + 2 * vectors[i + 1, 0] * track[i + 1, 0] \
                       + 2 * vectors[i + 1, 1] * track[i + 1, 1]

        else:
            H[i, i] += 2 * (math.pow(vectors[i, 0], 2) + math.pow(vectors[i, 1], 2))
            H[i, 0] = 0.5 * 2 * (-2 * vectors[i, 0] * vectors[0, 0] - 2 * vectors[i, 1] * vectors[0, 1])
            H[0, i] = H[i, 0]
            H[0, 0] += 2 * (math.pow(vectors[0, 0], 2) + math.pow(vectors[0, 1], 2))

            Q[i] += 2 * vectors[i, 0] * track[i, 0] - 2 * vectors[i, 0] * track[0, 0] \
                    + 2 * vectors[i, 1] * track[i, 1] - 2 * vectors[i, 1] * track[0, 1]
            Q[0] += -2 * vectors[0, 0] * track[i, 0] - 2 * vectors[0, 1] * track[i, 1] \
                    + 2 * vectors[0, 0] * track[0, 0] + 2 * vectors[0, 1] * track[0, 1]


    dev_right = track[:, 2]- veh.width /2 *veh.k_safe
    dev_left = track[:, 3]-  veh.width /2 *veh.k_safe

    dev_right[dev_right < 0.001] = 0.001
    dev_left[dev_left < 0.001] = 0.001

    G = np.vstack((np.eye(num_points), -np.eye(num_points)))
    h = np.ones(2 * num_points) * np.append(dev_right, dev_left)

    alpha = quadprog.solve_qp(H, -Q, -G.T, -h, 0)[0]


    return alpha
