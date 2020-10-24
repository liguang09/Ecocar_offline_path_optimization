import numpy as np
import json
import os
import prep_track
import matplotlib.pyplot as plt
from parameters import scale
import optimize

file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
file_paths["track_file"] = os.path.join(file_paths["module"], "tracks", "london_track" + ".csv")

track_raw= prep_track.src.import_track.import_track(file_path= file_paths["track_file"])

track_smooth_cl, s_smooth_cl= prep_track.src.interpol_track.smooth_track(track_raw)
print(np.shape(track_smooth_cl))
#kappa= prep_track.src.cal_curvature.calc_curv(path= track_interp_cl[:, :2], el_lengths= s_interp_cl)[1]

#n_opt, v_opt, u_opt, t_opt= optimize.src.mini_time.mini_time(track= track_interp, kappa= kappa)

plt.rcParams['savefig.dpi']= 400
plt.rcParams['figure.dpi']= 400
plt.figure()
plt.plot(track_raw[:,0], track_raw[:,1], 'r-', linewidth=0.9)
plt.plot(track_smooth_cl[:,0], track_smooth_cl[:,1], 'b--', linewidth=0.7)
#plt.plot(track_Bspline_cl[:,0], track_Bspline_cl[:,1], 'g--', linewidth=0.7)

plt.show()
