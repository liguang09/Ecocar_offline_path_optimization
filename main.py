import numpy as np
import json
import os
import prep_track
import matplotlib.pyplot as plt
from parameters import scale
import optimize
import trajectory_planning_helpers as tph

file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
file_paths["track_file"] = os.path.join(file_paths["module"], "tracks", "london_track" + ".csv")
file_paths["track_outer_file"] = os.path.join(file_paths["module"], "tracks", "london_outer" + ".csv")
file_paths["track_inner_file"] = os.path.join(file_paths["module"], "tracks", "london_inner" + ".csv")


track_raw= prep_track.src.import_track.import_track(file_path= file_paths["track_file"])

track_smooth_cl= prep_track.src.smooth_track.smooth_track(track_raw)

s_cumsum, s_seg= prep_track.src.track_length.track_length((track_smooth_cl[:, :2]))
s_cumsum= np.insert(s_cumsum, 0, 0.0)

#kappa= prep_track.src.cal_curvature.cal_curvature(track= track_smooth_cl[:, :2], s_length= s_cumsum)[1]
kappa= tph.calc_head_curv_num.calc_head_curv_num(path= track_smooth_cl[:, :2],
                                                 el_lengths= s_cumsum,
                                                 is_closed=True)[1]

n_opt, v_opt, u_opt, t_opt= optimize.src.mini_time.mini_time(track= track_smooth_cl, kappa= kappa)
print(np.shape(n_opt))
print(np.shape(track_smooth_cl))

opt_path= optimize.src.create_path.create_path(track= track_smooth_cl, devi= n_opt)

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

plt.show()
