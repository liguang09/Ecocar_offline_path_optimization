import numpy as np
import math


def spline_lengths(coeffs_x: np.ndarray,
                   coeffs_y: np.ndarray) -> np.ndarray:

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    no_interp_points: int = 15

    # catch case with only one spline
    if coeffs_x.size == 4 and coeffs_x.shape[0] == 4:
        coeffs_x = np.expand_dims(coeffs_x, 0)
        coeffs_y = np.expand_dims(coeffs_y, 0)

    # get number of splines and create output array
    no_splines = coeffs_x.shape[0]
    spline_lengths = np.zeros(no_splines)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LENGHTS ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # loop through all the splines and calculate intermediate coordinates
    t_steps = np.linspace(0.0, 1.0, no_interp_points)
    spl_coords = np.zeros((no_interp_points, 2))

    for i in range(no_splines):
        spl_coords[:, 0] = coeffs_x[i, 0] \
                            + coeffs_x[i, 1] * t_steps \
                            + coeffs_x[i, 2] * np.power(t_steps, 2) \
                            + coeffs_x[i, 3] * np.power(t_steps, 3)
        spl_coords[:, 1] = coeffs_y[i, 0] \
                            + coeffs_y[i, 1] * t_steps \
                            + coeffs_y[i, 2] * np.power(t_steps, 2) \
                            + coeffs_y[i, 3] * np.power(t_steps, 3)

        spline_lengths[i] = np.sum(np.sqrt(np.sum(np.power(np.diff(spl_coords, axis=0), 2), axis=1)))

    return spline_lengths
