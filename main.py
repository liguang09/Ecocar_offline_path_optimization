import numpy as np
import os
import matplotlib.pyplot as plt
import prep_track
import optimize
from parameters import maximum

file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
file_paths["track_file"] = os.path.join(file_paths["module"], "tracks", "london_track" + ".csv")

#=======================================================================================================================
# import track
#=======================================================================================================================
track_raw= prep_track.src.import_track.import_track(file_path= file_paths["track_file"])

#=======================================================================================================================
# Smooth track
#=======================================================================================================================
track_smooth_cl= prep_track.src.smooth_track.smooth_track(track_raw)

#=======================================================================================================================
# Calculate lengths
#=======================================================================================================================
# use Euclidean distance directly
s_cumsum, s_seg= prep_track.src.track_length.track_length((track_raw[:, :2]))
s_cumsum= np.insert(s_cumsum, 0, 0.0)
s_seg= np.insert(s_seg, 0, 0.0)

# via spline equations
track_smooth_xy_cl= np.vstack((track_smooth_cl[:, :2], track_smooth_cl[0, :2]))
coeffs_x, coeffs_y, a_interp, normvec_interp = prep_track.src.spline_coeffs.spline_coeffs(track=track_smooth_xy_cl)
spline_lengths = prep_track.src.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x, coeffs_y=coeffs_y)
#=======================================================================================================================
# Calculate curvature
#=======================================================================================================================
# the kappa below is very slow
# kappa= prep_track.src.cal_curvature.cal_curvature(track= track_smooth_cl[:, :2], s_length= s_cumsum)[1]

kappa= prep_track.src.calc_head_curv_num.calc_head_curv_num(path= track_smooth_cl[:, :2],
                                                            el_lengths= spline_lengths,
                                                            is_closed=True)[1]
#=======================================================================================================================
# Call Optimization
#=======================================================================================================================
#%% Shortest distance
n_opt_mindist= optimize.src.mini_distance.mini_distance(track= track_smooth_cl, vectors= normvec_interp)

#%% Minimum time
x_mini_time, u_mini_time, t_mini_time= optimize.src.mini_time.mini_time(track= track_smooth_cl, kappa= kappa)

n_mini_time= -x_mini_time[:-1, 3]
v_mini_time= x_mini_time[:-1, 0]
t_orig= s_cumsum[-1]/maximum.speed


#=======================================================================================================================
# Crated optimized trajectory
#=======================================================================================================================
mini_time_path= optimize.src.create_path.create_path(track= track_smooth_cl, devi= n_mini_time)
mini_time_path_cl= np.vstack((mini_time_path[:, :2], mini_time_path[0, :2]))

mini_dist_path= optimize.src.create_path.create_path(track= track_smooth_cl, devi= n_opt_mindist)
mini_dist_path_cl= np.vstack((mini_dist_path[:, :2], mini_dist_path[0, :2]))

# time curvature
x_time, y_time, _, n_time = prep_track.src.spline_coeffs.spline_coeffs(track=mini_time_path_cl)
spline_lengths_time = prep_track.src.calc_spline_lengths.calc_spline_lengths(coeffs_x=x_time, coeffs_y=y_time)
spline_lengths_time= np.insert(spline_lengths_time, 0, 0.0)
s_time= np.cumsum(spline_lengths_time)
kappa_time= prep_track.src.calc_head_curv_num.calc_head_curv_num(path=mini_time_path_cl[:, :2],
                                                                 el_lengths= spline_lengths_time,
                                                                 is_closed=True)[1]

x_dist, y_dist, _, n_dist = prep_track.src.spline_coeffs.spline_coeffs(track=mini_dist_path_cl)
spline_lengths_dist = prep_track.src.calc_spline_lengths.calc_spline_lengths(coeffs_x=x_dist, coeffs_y=y_dist)
s_dist= np.cumsum(spline_lengths_dist)
spline_lengths_dist= np.insert(spline_lengths_dist, 0, 0.0)
kappa_dist= prep_track.src.calc_head_curv_num.calc_head_curv_num(path=mini_dist_path_cl[:, :2],
                                                                 el_lengths= spline_lengths_dist,
                                                                 is_closed=True)[1]

weight= 0.5
opt_path_cl= weight*mini_time_path_cl[:, :2]+ (1- weight)*mini_dist_path_cl[:, :2]
x_opt, y_opt, _, norm_opt = prep_track.src.spline_coeffs.spline_coeffs(track=opt_path_cl)
spline_lengths_opt = prep_track.src.calc_spline_lengths.calc_spline_lengths(coeffs_x=x_opt, coeffs_y=y_opt)
s_opt= np.cumsum(spline_lengths_opt)
spline_lengths_opt= np.insert(spline_lengths_opt, 0, 0.0)
kappa_opt= prep_track.src.calc_head_curv_num.calc_head_curv_num(path= opt_path_cl[:, :2],
                                                                el_lengths= spline_lengths_opt,
                                                                is_closed=True)[1]


bound_inner= track_smooth_cl[:, :2] + n_time * np.expand_dims(track_smooth_cl[:, 2], 1)
bound_outer= track_smooth_cl[:, :2] - n_time * np.expand_dims(track_smooth_cl[:, 3], 1)
#=======================================================================================================================
# Export result
#=======================================================================================================================
optimize.src.save_results.save_results(path_minitime= mini_time_path_cl,
                                       kappa_minitime= kappa_time,
                                       s_minitime= spline_lengths_time,
                                       x_minitime= x_mini_time,
                                       u_minitime= u_mini_time,
                                       t_minitime= t_mini_time,
                                       path_minidist= mini_dist_path_cl,
                                       kappa_minidist= kappa_dist,
                                       s_minidist= spline_lengths_dist,
                                       path_opt= opt_path_cl,
                                       kappa_opt= kappa_opt,
                                       s_opt= spline_lengths_opt,
                                       path_center=track_smooth_xy_cl,
                                       kappa_center= kappa,
                                       s_center= spline_lengths,
                                       bound_outer=bound_outer,
                                       bound_inner= bound_inner)

#=======================================================================================================================
# Plot result
#=======================================================================================================================
#%% Plot
plt.rcParams['savefig.dpi']= 400
plt.rcParams['figure.dpi']= 400
plt.figure()

plt.plot(track_smooth_cl[:,0], track_smooth_cl[:,1], 'b--', linewidth=0.7)
plt.plot(bound_outer[:,0], bound_outer[:,1], 'b-', linewidth= 0.7)
plt.plot(bound_inner[:,0], bound_inner[:,1], 'b-', linewidth= 0.7)
plt.plot(mini_time_path_cl[:, 0], mini_time_path_cl[:, 1], 'r-', linewidth=0.5)
plt.plot(mini_dist_path_cl[:, 0], mini_dist_path_cl[:, 1], 'g-', linewidth=0.5)
plt.plot(opt_path_cl[:, 0], opt_path_cl[:, 1], 'b-', linewidth=0.5)
plt.show()

print('original lap time:', t_orig,'s')
print('optimized lap time:', t_mini_time[-1], 's')
print('optimized average speed:', np.average(v_mini_time), 'm/s')
print('original center length: ', s_cumsum[-1])
print('Mini time path length: ', np.cumsum(spline_lengths_time)[-1])
print('Mini distance path length: ', np.cumsum(spline_lengths_dist)[-1])
print('Optimal path length: ', np.cumsum(spline_lengths_opt)[-1])

