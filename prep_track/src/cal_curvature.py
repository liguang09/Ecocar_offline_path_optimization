import numpy as np
import math
import trajectory_planning_helpers.normalize_psi

def calc_curv(path: np.ndarray,
              el_lengths: np.ndarray) -> tuple:

    is_closed= True
    stepsize_psi_preview= 1.0
    stepsize_psi_review = 1.0
    stepsize_curv_preview = 2.0
    stepsize_curv_review = 2.0
    calc_curv= True

    # check inputs
    if is_closed and path.shape[0] != el_lengths.size:
        raise ValueError("path and el_lenghts must have the same length!")

    elif not is_closed and path.shape[0] != el_lengths.size + 1:
        raise ValueError("path must have the length of el_lengths + 1!")

    # get number if points
    no_points = path.shape[0]

    # ------------------------------------------------------------------------------------------------------------------
    # CASE: CLOSED PATH ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if is_closed:

        # --------------------------------------------------------------------------------------------------------------
        # PREVIEW/REVIEW DISTANCES -------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

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
        psi = trajectory_planning_helpers.normalize_psi.normalize_psi(psi)

        # --------------------------------------------------------------------------------------------------------------
        # CURVATURE ----------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        if calc_curv:
            psi_temp = np.insert(psi, 0, psi[-ind_step_review_curv:])
            psi_temp = np.append(psi_temp, psi[:ind_step_preview_curv])

            # calculate delta psi
            delta_psi = np.zeros(no_points)

            for i in range(no_points):
                delta_psi[i] = trajectory_planning_helpers.normalize_psi. \
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

    # ------------------------------------------------------------------------------------------------------------------
    # CASE: UNCLOSED PATH ----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    else:

        # --------------------------------------------------------------------------------------------------------------
        # HEADING ------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # calculate tangent vectors for every point
        tangvecs = np.zeros((no_points, 2))

        tangvecs[0, 0] = path[1, 0] - path[0, 0]  # i == 0
        tangvecs[0, 1] = path[1, 1] - path[0, 1]

        tangvecs[1:-1, 0] = path[2:, 0] - path[:-2, 0]  # 0 < i < no_points - 1
        tangvecs[1:-1, 1] = path[2:, 1] - path[:-2, 1]

        tangvecs[-1, 0] = path[-1, 0] - path[-2, 0]  # i == -1
        tangvecs[-1, 1] = path[-1, 1] - path[-2, 1]

        # calculate psi of tangent vectors (pi/2 must be substracted due to our convention that psi = 0 is north)
        psi = np.arctan2(tangvecs[:, 1], tangvecs[:, 0]) - math.pi / 2
        psi = trajectory_planning_helpers.normalize_psi.normalize_psi(psi)

        # --------------------------------------------------------------------------------------------------------------
        # CURVATURE ----------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        if calc_curv:
            # calculate delta psi
            delta_psi = np.zeros(no_points)

            delta_psi[0] = psi[1] - psi[0]  # i == 0
            delta_psi[1:-1] = psi[2:] - psi[:-2]  # 0 < i < no_points - 1
            delta_psi[-1] = psi[-1] - psi[-2]  # i == -1

            # normalize delta_psi
            delta_psi = trajectory_planning_helpers.normalize_psi.normalize_psi(delta_psi)

            # calculate kappa
            kappa = np.zeros(no_points)

            kappa[0] = delta_psi[0] / el_lengths[0]  # i == 0
            kappa[1:-1] = delta_psi[1:-1] / (el_lengths[1:] + el_lengths[:-1])  # 0 < i < no_points - 1
            kappa[-1] = delta_psi[-1] / el_lengths[-1]  # i == -1

        else:
            kappa = 0.0

    return psi, kappa

    # testing --------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    pass