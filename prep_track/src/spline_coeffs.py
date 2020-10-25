import numpy as np
from prep_track.src import track_length

def spline_coeffs(track: np.ndarray) -> tuple:

    if np.all(np.isclose(track[0], track[-1])):
        is_closed = True
    else:
        is_closed = False

    if not is_closed:
        raise ValueError("Headings must be provided for unclosed spline calculation!")

    s_cum, s_seg= track_length.track_length(track[:, :2])

    use_scale= True
    num_points= track.shape[0]-1

    if use_scale and is_closed:
        s_seg= np.append(s_seg, s_seg[0])

    if use_scale:
        scale= s_seg[:-1]/ s_seg[1:]
    else:
        scale= np.ones(num_points-1)

    M = np.zeros((num_points* 4, num_points* 4))
    X = np.zeros((num_points* 4, 1))
    Y = np.zeros((num_points* 4, 1))

    M_temp = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                       [1, 1, 1, 1, 0, 0, 0, 0],
                       [0, 1, 2, 3, 0, -1, 0, 0],
                       [0, 0, 2, 6, 0, 0, -2, 0]])

    for i in range(num_points):
        j = i * 4

        if i < num_points- 1:
            M[j: j+ 4, j: j+ 8] = M_temp
            M[j+ 2, j+ 5] *= scale[i]
            M[j+ 3, j+ 6] *= np.power(scale[i], 2)

        else:
            # no curvature and heading bounds on last element (handled afterwards)
            M[j: j+ 2, j: j+ 4] = [[1,  0,  0,  0], [1,  1,  1,  1]]

        X[j: j+ 2] = [[track[i, 0]], [track[i+ 1, 0]]]
        Y[j: j+ 2] = [[track[i, 1]], [track[i+ 1, 1]]]

    if not is_closed:

        M[-2, 1] = 1  # heading start point (evaluated at t = 0)

        s_seg_first = s_seg[0]

        psi_s= None
        psi_e= None
        X[-2] = np.cos(psi_s+ np.pi/ 2)* s_seg[0]
        Y[-2] = np.sin(psi_s+ np.pi/ 2)* s_seg[0]

        # heading end point
        M[-1, -4:] = [0, 1, 2, 3]  # heading end point (evaluated at t = 1)

        X[-1] = np.cos(psi_e+ np.pi/ 2)* s_seg[-1]
        Y[-1] = np.sin(psi_e+ np.pi/ 2)* s_seg[-1]

    else:
        # heading boundary condition (for a closed spline)
        M[-2, 1] = scale[-1]
        M[-2, -3:] = [-1, -2, -3]

        # curvature boundary condition (for a closed spline)
        M[-1, 2]= 2* np.power(scale[-1], 2)
        M[-1, -2:]= [-2, -6]

    x_les = np.squeeze(np.linalg.solve(M, X))
    y_les = np.squeeze(np.linalg.solve(M, Y))

    # get coefficients of every piece into one row -> reshape
    coeffs_x = np.reshape(x_les, (num_points, 4))
    coeffs_y = np.reshape(y_les, (num_points, 4))

    # get normal vector (behind used here instead of ahead for consistency with other functions) (second coefficient of
    # cubic splines is relevant for the heading)
    normvec_temp = np.stack((coeffs_y[:, 1], -coeffs_x[:, 1]), axis=1)

    # normalize normal vectors
    norm_factors = 1.0 / np.sqrt(np.sum(np.power(normvec_temp, 2), axis=1))
    normvec= np.expand_dims(norm_factors, axis=1)* normvec_temp

    return coeffs_x, coeffs_y, M, normvec

if __name__ == "__main__":
    pass