import numpy as np
from prep_track.src import spline_coeffs

def create_path(track: np.ndarray,
                devi: np.ndarray) -> tuple:

    track_xy= track[:, :2]
    track_xy_cl= np.vstack((track[:, :2], track[0, :2]))
    coeffs_x, coeffs_y, M, normvec= spline_coeffs.spline_coeffs(track= track_xy_cl)

    trajectory= track[:, :2]+ np.expand_dims(devi, 1)*normvec

    return trajectory

if __name__ == "__main__":
    pass