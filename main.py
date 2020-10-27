import numpy as np
import json
import os
import prep_track
import matplotlib.pyplot as plt
from parameters import maximum
import optimize
import trajectory_planning_helpers as tph


file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
file_paths["track_file"] = os.path.join(file_paths["module"], "tracks", "london_track" + ".csv")
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
s_cumsum, s_seg= prep_track.src.track_length.track_length((track_smooth_cl[:, :2]))
s_cumsum= np.insert(s_cumsum, 0, 0.0)
s_seg= np.insert(s_seg, 0, 0.0)

# via spline equations
track_smooth_xy_cl= np.vstack((track_smooth_cl[:, :2], track_smooth_cl[0, :2]))

coeffs_x, coeffs_y, a_interp, normvec_interp = tph.calc_splines.calc_splines(path=track_smooth_xy_cl)
# coeffs_x, coeffs_y, a_interp, normvec_interp = prep_track.src.spline_coeffs.spline_coeffs(track=track_smooth_xy_cl)
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
n_opt, v_opt, u_opt, t_opt= optimize.src.mini_time.mini_time(track= track_smooth_cl, kappa= kappa)

print(np.shape(v_opt))
print(np.shape(s_seg))
t_orig= s_cumsum[-1]/maximum.speed

#=======================================================================================================================
# Crated optimized trajectory
#=======================================================================================================================
opt_path= optimize.src.create_path.create_path(track= track_smooth_cl, devi= n_opt)

#=======================================================================================================================
# Plot result
#=======================================================================================================================
track_outer_data = np.loadtxt(file_paths["track_outer_file"], comments='#', delimiter=',')
track_inner_data = np.loadtxt(file_paths["track_inner_file"], comments='#', delimiter=',')

plt.rcParams['savefig.dpi']= 400
plt.rcParams['figure.dpi']= 400
plt.figure()

#plt.plot(track_raw[:,0], track_raw[:,1], 'r-', linewidth=0.9)
plt.plot(track_smooth_cl[:,0], track_smooth_cl[:,1], 'r-', linewidth=0.7)
plt.plot(track_outer_data[:,0], track_outer_data[:,1], 'r-', linewidth= 0.7)
plt.plot(track_inner_data[:,0], track_inner_data[:,1], 'r-', linewidth= 0.7)
plt.plot(opt_path[:,0], opt_path[:,1], 'b-', linewidth=0.7)

plt.figure()
plt.plot(s_cumsum, v_opt, 'r')
plt.title('Speed')

plt.figure()
plt.plot( s_cumsum, u_opt[:, 0],'g')
plt.title('Steer Angle')

plt.figure()
plt.plot(s_cumsum, u_opt[:, 1], 'b')
plt.title('Drive Effort')

plt.figure()
plt.plot(s_cumsum, u_opt[:, 2])
plt.title('Brake Effort')

plt.figure()
plt.plot(s_cumsum, kappa)
plt.title('Curvature')

plt.show()

print('original lap time:', t_orig,'s')
print('optimized lap time:', t_opt[-1],'s')
print('optimized average speed:', np.average(v_opt), 'm/s')
