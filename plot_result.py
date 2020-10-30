import os
import numpy as np
import matplotlib.pyplot as plt
from parameters import scale

#=======================================================================================================================
# create the paths
#=======================================================================================================================
file_paths= {}
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

# Track file
file_paths["track_file"] = os.path.join(file_paths["module"], "optimize", "src", "results", "track_smooth" + ".csv")
file_paths["track_outer_file"] = os.path.join(file_paths["module"], "tracks", "london_outer" + ".csv")
file_paths["track_inner_file"] = os.path.join(file_paths["module"], "tracks", "london_inner" + ".csv")
file_paths["trajectory_file"] = os.path.join(file_paths["module"], "optimize", "src", "results", "traj_opt" + ".csv")

# optimization results
file_paths["states_file"]= os.path.join(file_paths["module"], "optimize", "src", "results", "states" + ".csv")
file_paths["u_file"]= os.path.join(file_paths["module"], "optimize", "src", "results", "ctrl_inputs" + ".csv")
file_paths["path_opt_file"]= os.path.join(file_paths["module"], "optimize", "src", "results", "path_opt" + ".csv")
file_paths["path_center_file"]= os.path.join(file_paths["module"], "optimize", "src", "results", "path_center" + ".csv")
file_paths["kappa_opt_file"]= os.path.join(file_paths["module"], "optimize", "src", "results", "kappa_opt" + ".csv")

#=======================================================================================================================
# read the outputs
#=======================================================================================================================
x_opt= np.loadtxt(file_paths["states_file"], comments='#', delimiter=';')
u_opt= np.loadtxt(file_paths["u_file"], comments='#', delimiter=';')
kappa_opt= np.loadtxt(file_paths["kappa_opt_file"], comments='#', delimiter=';')
path_opt= np.loadtxt(file_paths["path_opt_file"], comments='#', delimiter=';')

s_opt= np.cumsum(path_opt)

# track and trajectory
track_smooth= np.loadtxt(file_paths["track_file"], comments='#', delimiter=';')
track_outer_data = np.loadtxt(file_paths["track_outer_file"], comments='#', delimiter=',')
track_inner_data = np.loadtxt(file_paths["track_inner_file"], comments='#', delimiter=',')
trajectory= np.loadtxt(file_paths["trajectory_file"], comments='#', delimiter=';')

#=======================================================================================================================
# Plot results
#=======================================================================================================================
plot_start= 50
plot_end= 100

plt.rcParams['savefig.dpi']= 400
plt.rcParams['figure.dpi']= 400

plt.figure()
#plt.plot(track_outer_data[:,0], track_outer_data[:,1], 'r-', linewidth= 0.7)
#plt.plot(track_inner_data[:,0], track_inner_data[:,1], 'r-', linewidth= 0.7)
plt.plot(track_smooth[:,0], track_smooth[:,1], 'r-', linewidth=0.7, label= "center")
plt.plot(trajectory[:,0], trajectory[:,1], 'b-', linewidth=0.7, label="optimal")
plt.legend(loc= "upper right")

# Steering angle profile
plt.figure()
plt.plot(s_opt[plot_start:plot_end], u_opt[plot_start:plot_end, 0]/scale.delta,'g')
plt.plot(s_opt[plot_start:plot_end], kappa_opt[plot_start:plot_end])
plt.title('Steer Angle & Curvature')

# Speed profile
plt.figure()
plt.plot(s_opt[plot_start:plot_end], x_opt[plot_start:plot_end, 0]/scale.speed,'r', marker="*", label="speed")
plt.plot(s_opt[plot_start:plot_end], u_opt[plot_start:plot_end, 0]/scale.delta,'b', marker="^", label="delta")
plt.plot(s_opt[plot_start:plot_end], kappa_opt[plot_start:plot_end], marker="x", label="kappa")
plt.title('Speed & Steer Angle')
plt.legend(loc='upper left')

plt.show()