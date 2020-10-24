import numpy as np
import math
from scipy import interpolate

def interpol_track(track: np.ndarray) -> np.ndarray:

    track_closed= np.vstack((track, track[0]))
    s_track= np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_closed[:, :2], axis=0), 2), axis=1)))
    s_track= np.insert(s_track, 0, 0.0)

    step= 1.0
    num_interp_points= math.ceil(s_track[-1]/step)+ 1
    linspace= np.linspace(0.0, s_track[-1], num_interp_points)

    # Linear interpolation
    track_linear_cl= np.zeros((num_interp_points, 4))
    track_linear_cl[:, 0]= np.interp(linspace, s_track, track_closed[:, 0])
    track_linear_cl[:, 1]= np.interp(linspace, s_track, track_closed[:, 1])
    track_linear_cl[:, 2]= np.interp(linspace, s_track, track_closed[:, 2])
    track_linear_cl[:, 3]= np.interp(linspace, s_track, track_closed[:, 3])
    s_track_linear= np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_linear_cl[:, :2], axis=0), 2), axis=1)))
    s_track_linear= np.insert(s_track_linear, 0, 0.0)


    # Spline to smooth the track
    tck_cl, u_cl= interpolate.splprep([track_linear_cl[:, 0], track_linear_cl[:, 1]], k=3, s= 10, per=0)[:2]
    track_Bspline_cl= np.array(interpolate.splev(u_cl, tck_cl)).T
    s_track_Bspline= np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_Bspline_cl[:, :2], axis=0), 2), axis=1)))
    s_track_Bspline = np.insert(s_track_Bspline, 0, 0.0)


    return track_linear_cl, s_track_linear


if __name__ == "__main__":
    pass