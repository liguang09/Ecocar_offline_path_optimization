import numpy as np
import math
import prep_track.src.normalize_psi


def calc_curv(path: np.ndarray,
              el_lengths: np.ndarray) -> tuple:

    stepsize_psi_preview: float = 1.0
    stepsize_psi_review: float = 1.0
    stepsize_curv_preview: float = 2.0
    stepsize_curv_review: float = 2.0
    calcurve: bool = True
    is_closed: bool = True

    # get number if points
    no_points = path.shape[0]

    if is_closed:

        # calculate how many points we look to the front and rear of the current position for the head/curv calculations
        ind_step_preview_psi = round(stepsize_psi_preview / float(np.average(el_lengths)))
        ind_step_review_psi = round(stepsize_psi_review / float(np.average(el_lengths)))
        ind_step_preview_curv = round(stepsize_curv_preview / float(np.average(el_lengths)))
        ind_step_review_curv = round(stepsize_curv_review / float(np.average(el_lengths)))

        ind_step_preview_psi = max(ind_step_preview_psi, 1)
        ind_step_review_psi = max(ind_step_review_psi, 1)
        ind_step_preview_curv = max(ind_step_preview_curv, 1)
        ind_step_review_curv = max(ind_step_review_curv, 1)

        steps_tot_psi = ind_step_preview_psi + ind_step_review_psi
        steps_tot_curv = ind_step_preview_curv + ind_step_review_curv

        # --------------------------------------------------------------------------------------------------------------
        # HEADING ------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # calculate tangent vectors for every point
        path_temp = np.vstack((path[-ind_step_review_psi:], path, path[:ind_step_preview_psi]))
        tangvecs = np.stack((path_temp[steps_tot_psi:, 0] - path_temp[:-steps_tot_psi, 0],
                             path_temp[steps_tot_psi:, 1] - path_temp[:-steps_tot_psi, 1]), axis=1)

        # calculate psi of tangent vectors (pi/2 must be substracted due to our convention that psi = 0 is north)
        psi = np.arctan2(tangvecs[:, 1], tangvecs[:, 0]) - math.pi / 2
        psi = prep_track.src.normalize_psi.normalize_psi(psi)

        # --------------------------------------------------------------------------------------------------------------
        # CURVATURE ----------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        if calcurve:
            psi_temp = np.insert(psi, 0, psi[-ind_step_review_curv:])
            psi_temp = np.append(psi_temp, psi[:ind_step_preview_curv])

            # calculate delta psi
            delta_psi = np.zeros(no_points)

            for i in range(no_points):
                delta_psi[i] = prep_track.src.normalize_psi.\
                    normalize_psi(psi_temp[i + steps_tot_curv] - psi_temp[i])

            # calculate kappa
            s_points_cl = np.cumsum(el_lengths)
            s_points_cl = np.insert(s_points_cl, 0, 0.0)
            s_points = s_points_cl[:-1]
            s_points_cl_reverse = np.flipud(-np.cumsum(np.flipud(el_lengths)))  # should not include 0.0 as last value

            s_points_temp = np.insert(s_points, 0, s_points_cl_reverse[-ind_step_review_curv:])
            s_points_temp = np.append(s_points_temp, s_points_cl[-1] + s_points[:ind_step_preview_curv])

            kappa = delta_psi / (s_points_temp[steps_tot_curv:] - s_points_temp[:-steps_tot_curv])

        else:
            kappa = 0.0

    return psi, kappa
