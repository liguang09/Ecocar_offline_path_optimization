import numpy as np
import math
import quadprog
from parameters import veh


def mini_distance(track: np.ndarray,
                  normvectors: np.ndarray
                  ) -> np.ndarray:


    no_points = track.shape[0]

    H = np.zeros((no_points, no_points))
    f = np.zeros(no_points)

    for i in range(no_points):
        if i < no_points - 1:
            H[i, i] += 2 * (math.pow(normvectors[i, 0], 2) + math.pow(normvectors[i, 1], 2))
            H[i, i + 1] = 0.5 * 2 * (-2 * normvectors[i, 0] * normvectors[i + 1, 0]
                                     - 2 * normvectors[i, 1] * normvectors[i + 1, 1])
            H[i + 1, i] = H[i, i + 1]
            H[i + 1, i + 1] = 2 * (math.pow(normvectors[i + 1, 0], 2) + math.pow(normvectors[i + 1, 1], 2))

            f[i] += 2 * normvectors[i, 0] * track[i, 0] - 2 * normvectors[i, 0] * track[i + 1, 0] \
                    + 2 * normvectors[i, 1] * track[i, 1] - 2 * normvectors[i, 1] * track[i + 1, 1]
            f[i + 1] = -2 * normvectors[i + 1, 0] * track[i, 0] \
                       - 2 * normvectors[i + 1, 1] * track[i, 1] \
                       + 2 * normvectors[i + 1, 0] * track[i + 1, 0] \
                       + 2 * normvectors[i + 1, 1] * track[i + 1, 1]

        else:
            H[i, i] += 2 * (math.pow(normvectors[i, 0], 2) + math.pow(normvectors[i, 1], 2))
            H[i, 0] = 0.5 * 2 * (-2 * normvectors[i, 0] * normvectors[0, 0] - 2 * normvectors[i, 1] * normvectors[0, 1])
            H[0, i] = H[i, 0]
            H[0, 0] += 2 * (math.pow(normvectors[0, 0], 2) + math.pow(normvectors[0, 1], 2))

            f[i] += 2 * normvectors[i, 0] * track[i, 0] - 2 * normvectors[i, 0] * track[0, 0] \
                    + 2 * normvectors[i, 1] * track[i, 1] - 2 * normvectors[i, 1] * track[0, 1]
            f[0] += -2 * normvectors[0, 0] * track[i, 0] - 2 * normvectors[0, 1] * track[i, 1] \
                    + 2 * normvectors[0, 0] * track[0, 0] + 2 * normvectors[0, 1] * track[0, 1]


    dev_max_right = track[:, 2] - (veh.width/1.5) / 2
    dev_max_left = track[:, 3] - (veh.width/1.5) / 2

    dev_max_right[dev_max_right < 0.001] = 0.001
    dev_max_left[dev_max_left < 0.001] = 0.001

    G = np.vstack((np.eye(no_points), -np.eye(no_points)))
    h = np.ones(2 * no_points) * np.append(dev_max_right, dev_max_left)

    alpha = quadprog.solve_qp(H, -f, -G.T, -h, 0)[0]


    return alpha
