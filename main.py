import numpy as np
import os
import matplotlib.pyplot as plt
import prep_track
import optimize
import trajectory_planning_helpers as tph
from parameters import maximum
from parameters import scale

file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
file_paths["track_file"] = os.path.join(file_paths["module"], "tracks", "rrc_track" + ".csv")
file_paths["track_outer_file"] = os.path.join(file_paths["module"], "tracks", "london_outer" + ".csv")
file_paths["track_inner_file"] = os.path.join(file_paths["module"], "tracks", "london_inner" + ".csv")

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

# coeffs_x, coeffs_y, a_interp, normvec_interp = tph.calc_splines.calc_splines(path=track_smooth_xy_cl)
coeffs_x, coeffs_y, a_interp, normvec_interp = prep_track.src.spline_coeffs.spline_coeffs(track=track_smooth_xy_cl)
spline_lengths = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x, coeffs_y=coeffs_y)
#=======================================================================================================================
# Calculate curvature
#=======================================================================================================================
# the kappa below is very slow
# kappa= prep_track.src.cal_curvature.cal_curvature(track= track_smooth_cl[:, :2], s_length= s_cumsum)[1]
kappa= tph.calc_head_curv_num.calc_head_curv_num(path= track_smooth_cl[:, :2],
                                                 el_lengths= spline_lengths,
                                                 is_closed=True)[1]
#=======================================================================================================================
# Call Optimization
#=======================================================================================================================
#%% Start Optimization
x_opt, u_opt, t_opt= optimize.src.mini_time.mini_time(track= track_smooth_cl, kappa= kappa)

n_opt= -x_opt[:-1, 3]
v_opt= x_opt[:-1, 0]
t_orig= s_cumsum[-1]/maximum.speed

#=======================================================================================================================
# Crated optimized trajectory
#=======================================================================================================================
# optimized path
opt_path= optimize.src.create_path.create_path(track= track_smooth_cl, devi= n_opt)

opt_path_cl= np.vstack((opt_path[:, :2], opt_path[0, :2]))

# optimized curvature
# coe_x_opt, coe_y_opt, a_interp, normvec_interp = tph.calc_splines.calc_splines(path=track_smooth_xy_cl)
coe_x_opt, coe_y_opt, _, _ = prep_track.src.spline_coeffs.spline_coeffs(track=opt_path_cl)
spline_lengths_opt = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=coe_x_opt, coeffs_y=coe_y_opt)

spline_lengths_opt= np.insert(spline_lengths_opt, 0, 0.0)
kappa_opt= tph.calc_head_curv_num.calc_head_curv_num(path= opt_path_cl[:, :2],
                                                     el_lengths= spline_lengths_opt,
                                                     is_closed=True)[1]

#=======================================================================================================================
# Export result
#=======================================================================================================================
optimize.src.save_results.save_results(trajectory_opt=opt_path_cl,
                                       track_smooth= track_smooth_xy_cl,
                                       s_opt= spline_lengths_opt,
                                       t_opt= t_opt,
                                       x_opt= x_opt,
                                       u_opt= u_opt,
                                       kappa_opt= kappa_opt,
                                       s_center= spline_lengths,
                                       kappa_center= kappa)


#=======================================================================================================================
# Plot result
#=======================================================================================================================
#%% Plot
track_outer_data = np.loadtxt(file_paths["track_outer_file"], comments='#', delimiter=',')
track_inner_data = np.loadtxt(file_paths["track_inner_file"], comments='#', delimiter=',')

plt.rcParams['savefig.dpi']= 400
plt.rcParams['figure.dpi']= 400
plt.figure()

plt.plot(track_smooth_cl[:,0], track_smooth_cl[:,1], 'r--', linewidth=0.7)
# plt.plot(track_outer_data[:,0], track_outer_data[:,1], 'r-', linewidth= 0.7)
# plt.plot(track_inner_data[:,0], track_inner_data[:,1], 'r-', linewidth= 0.7)
plt.plot(opt_path[:,0], opt_path[:,1], 'b-', linewidth=0.7)

plt.show()

print('original lap time:', t_orig,'s')
print('optimized lap time:', t_opt[-1],'s')
print('original center length: ', s_cumsum[-1])
print('optimized center length: ', np.cumsum(spline_lengths_opt)[-1])
print('optimized average speed:', np.average(v_opt), 'm/s')
