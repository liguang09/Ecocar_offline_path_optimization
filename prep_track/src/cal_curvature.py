import numpy as np
# import trajectory_planning_helpers as tph

def cal_curvature (track: np.ndarray,
                   s_length: np.ndarray) -> tuple:

    num_points= track.shape[0]
    track_temp= np.vstack((track[-1:], track, track[:-1]))
    tan_vec= np.stack((track_temp[1:, 0]- track_temp[:-1, 0],
                       track_temp[1:, 1]- track_temp[:-1, 1]), axis=1)

    psi_temp= np.arctan2(tan_vec[:,1], tan_vec[:,0])- np.pi/2
    psi = np.sign(psi_temp) * np.mod(np.abs(psi_temp), 2 * np.pi)

    if type(psi) is np.ndarray:
        psi[psi >= np.pi] -= 2 * np.pi
        psi[psi < -np.pi] += 2 * np.pi

    else:
        if psi >= np.pi:
            psi -= 2 * np.pi
        elif psi < -np.pi:
            psi += 2 * np.pi

    s_points_cl = np.insert(s_length, 0, 0.0)
    s_points = s_points_cl[:-1]
    s_points_cl_reverse = np.flipud(-np.cumsum(np.flipud(s_length)))
    s_points_temp = np.insert(s_points, 0, s_points_cl_reverse[-1:])
    s_points_temp = np.append(s_points_temp, s_points_cl[-1] + s_points[:1])

    psi_temp = np.insert(psi, 0, psi[-1:])
    psi_temp = np.append(psi_temp, psi[:1])
    delta_psi = np.zeros(num_points)

    for i in range(num_points):
        delta_psi[i]= psi_temp[i + 1]- psi_temp[i]

    kappa = delta_psi / (s_points_temp[1:] - s_points_temp[:-1])

    return psi, kappa