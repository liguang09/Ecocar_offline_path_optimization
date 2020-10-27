import numpy as np
import math
from scipy import interpolate
from scipy import optimize
from scipy import spatial
from typing import Union


def smooth_track(track: np.ndarray) -> np.ndarray:

    track_raw_cl= np.vstack((track, track[0])) # close the track,

    #-------------------------------------------------------------------------------------------------------------------
    # Smooth center line
    #----------------------------------------------------------------------------------------------------------------
    s_raw_cl= np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_raw_cl[:, :2], axis=0), 2), axis=1)))
    s_raw_cl= np.insert(s_raw_cl, 0, 0.0) # insert start point 0.0

    step= 1.0
    num_interp_points= math.ceil(s_raw_cl[-1]/step)+ 1 # the interpolated points= total length/ step
    linspace= np.linspace(0.0, s_raw_cl[-1], num_interp_points) # insert interpolated points into [0 total length]

    # Linear interpolation
    track_linear_cl= np.zeros((num_interp_points, 4))
    track_linear_cl[:, 0]= np.interp(linspace, s_raw_cl, track_raw_cl[:, 0]) # x= fx(s), interpolate [[s1,x1],[s2,x2],[s3,x3],...[sn,xn]]
    track_linear_cl[:, 1]= np.interp(linspace, s_raw_cl, track_raw_cl[:, 1]) # y= fy(s), interpolate [[s1,y1],[s2,y2],[s3,y3],...[sn,yn]]
    track_linear_cl[:, 2]= np.interp(linspace, s_raw_cl, track_raw_cl[:, 2])
    track_linear_cl[:, 3]= np.interp(linspace, s_raw_cl, track_raw_cl[:, 3])

    # Smooth the track by spline
    tck_cl, u_cl= interpolate.splprep([track_linear_cl[:, 0], track_linear_cl[:, 1]], k=3, s= 10, per=0)[:2]
    smooth_space= np.linspace(0.0, 1.0, math.ceil(s_raw_cl[-1])*2)  # add more points to smooth the track
    track_smooth_cl= np.array(interpolate.splev(smooth_space, tck_cl)).T
    s_smooth_cl= np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_smooth_cl[:, :2], axis=0), 2), axis=1)))
    s_smooth_cl= np.insert(s_smooth_cl, 0, 0.0)

    step_smooth= 1
    num_point_smooth= math.ceil(s_smooth_cl[-1]/step_smooth)+1
    track_smooth_cl= np.array(interpolate.splev(np.linspace(0.0, 1.0, num_point_smooth), tck_cl)).T[:-1]
    #s_smooth_cl = np.cumsum(np.sqrt(np.sum(np.power(np.diff(track_smooth_cl[:, :2], axis=0), 2), axis=1)))
    #s_smooth_cl = np.insert(s_smooth_cl, 0, 0.0)

    #-------------------------------------------------------------------------------------------------------------------
    # Smooth n
    #-------------------------------------------------------------------------------------------------------------------
    num_p_track= track_raw_cl.shape[0]
    dist_cl= np.zeros(num_p_track)
    mini_dist_point= np.zeros((num_p_track,2))
    mini_dist_coeff= np.zeros(num_p_track)
    coeff_init= s_raw_cl/s_raw_cl[-1]

    for i in range (num_p_track):
        mini_dist_coeff[i]= optimize.fmin(dist2p,
                                          x0= coeff_init[i],
                                          args=(tck_cl, track_raw_cl[i, :2]),
                                          disp=0)
        mini_dist_point[i]= interpolate.splev(mini_dist_coeff[i], tck_cl)
        dist_cl[i]= spatial.distance.euclidean([mini_dist_point[i,0]- track_raw_cl[i, 0]],
                                               [mini_dist_point[i,1]- track_raw_cl[i, 1]])

    signs= np.zeros(num_p_track-1)

    for i in range(num_p_track-1):
        signs[i]= deter_signs(x= track_raw_cl[i, :2],
                              y= track_raw_cl[i+1, :2],
                              z= mini_dist_point[i])

    signs = np.hstack((signs, signs[0]))
    w_right_cl= track_raw_cl[:, 2]+ signs* dist_cl
    w_left_cl= track_raw_cl[:, 3]- signs* dist_cl

    w_right_smooth= np.interp(np.linspace(0.0, 1.0, num_point_smooth), mini_dist_coeff, w_right_cl)
    w_left_smooth = np.interp(np.linspace(0.0, 1.0, num_point_smooth), mini_dist_coeff, w_left_cl)

    track_smooth= np.column_stack((track_smooth_cl, w_right_smooth[:-1], w_left_smooth[:-1]))

    return track_smooth


def dist2p(coeff: np.ndarray, track: list, p: np.ndarray):
    s = interpolate.splev(coeff, track)
    return spatial.distance.euclidean(p, s)


def deter_signs(x: Union[tuple, np.ndarray],
                y: Union[tuple, np.ndarray],
                z: Union[tuple, np.ndarray]) -> float:
    signs = np.sign((y[0]- x[0])* (z[1]- x[1])- (y[1]- x[1])* (z[0]- x[0]))
    return signs

if __name__ == "__main__":
    pass