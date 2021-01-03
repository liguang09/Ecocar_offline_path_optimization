import numpy as np
import math
from typing import Union


def calc_curv(path: np.ndarray,
              el_lengths: np.ndarray) -> tuple:

    step_psi_preview: float = 1.0
    step_psi_review: float = 1.0
    step_curv_preview: float = 2.0
    step_curv_review: float = 2.0

    no_points = path.shape[0]

    ind_step_preview_psi = round(step_psi_preview / float(np.average(el_lengths)))
    ind_step_review_psi = round(step_psi_review / float(np.average(el_lengths)))
    ind_step_preview_curv = round(step_curv_preview / float(np.average(el_lengths)))
    ind_step_review_curv = round(step_curv_review / float(np.average(el_lengths)))

    ind_step_preview_psi = max(ind_step_preview_psi, 1)
    ind_step_review_psi = max(ind_step_review_psi, 1)
    ind_step_preview_curv = max(ind_step_preview_curv, 1)
    ind_step_review_curv = max(ind_step_review_curv, 1)

    steps_tot_psi = ind_step_preview_psi + ind_step_review_psi
    steps_tot_curv = ind_step_preview_curv + ind_step_review_curv


    # calculate tangent vectors for every point
    path_temp = np.vstack((path[-ind_step_review_psi:], path, path[:ind_step_preview_psi]))
    tangvecs = np.stack((path_temp[steps_tot_psi:, 0] - path_temp[:-steps_tot_psi, 0],
                         path_temp[steps_tot_psi:, 1] - path_temp[:-steps_tot_psi, 1]), axis=1)

    psi = np.arctan2(tangvecs[:, 1], tangvecs[:, 0]) - math.pi / 2
    psi = normalize_psi(psi)

    psi_temp = np.insert(psi, 0, psi[-ind_step_review_curv:])
    psi_temp = np.append(psi_temp, psi[:ind_step_preview_curv])

    # calculate delta psi
    delta_psi = np.zeros(no_points)

    for i in range(no_points):
        delta_psi[i] = normalize_psi(psi_temp[i + steps_tot_curv] - psi_temp[i])

    # calculate kappa
    s_points_cl = np.cumsum(el_lengths)
    s_points_cl = np.insert(s_points_cl, 0, 0.0)
    s_points = s_points_cl[:-1]
    s_points_cl_reverse = np.flipud(-np.cumsum(np.flipud(el_lengths)))  # should not include 0.0 as last value

    s_points_temp = np.insert(s_points, 0, s_points_cl_reverse[-ind_step_review_curv:])
    s_points_temp = np.append(s_points_temp, s_points_cl[-1] + s_points[:ind_step_preview_curv])

    kappa = delta_psi / (s_points_temp[steps_tot_curv:] - s_points_temp[:-steps_tot_curv])

    return psi, kappa

def normalize_psi(psi: Union[np.ndarray, float]) -> np.ndarray:

    psi_out = np.sign(psi) * np.mod(np.abs(psi), 2 * math.pi)

    if type(psi_out) is np.ndarray:
        psi_out[psi_out >= math.pi] -= 2 * math.pi
        psi_out[psi_out < -math.pi] += 2 * math.pi

    else:
        if psi_out >= math.pi:
            psi_out -= 2 * math.pi
        elif psi_out < -math.pi:
            psi_out += 2 * math.pi

    return psi_out