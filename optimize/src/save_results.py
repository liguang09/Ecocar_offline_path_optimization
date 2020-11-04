import numpy as np
import os

# this function saves the optimized results into csv format, for plotting and analysis

def save_results(trajectory_opt: np.ndarray,
                 track_smooth: np.ndarray,
                 s_opt: np.ndarray,
                 t_opt: np.ndarray,
                 x_opt: np.ndarray,
                 u_opt: np.ndarray,
                 kappa_opt: np.ndarray,
                 s_center: np.ndarray,
                 kappa_center: np.ndarray) -> None:

    file_path = {}
    file_path["module"] = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(file_path["module"] + "/results", exist_ok=True)
    export_path = os.path.join(file_path["module"], "results")

    header_traj= "traj_x; traj_y"
    fmt_traj= "%.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'traj_opt.csv'), trajectory_opt, fmt=fmt_traj, header=header_traj)

    header_track = "track_x; track_y"
    fmt_track = "%.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'track_smooth.csv'), track_smooth, fmt=fmt_track, header=header_track)

    header_x= "v; beta; omega; n; xi;"
    fmt_x= "%.5f; %.5f; %.5f; %.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'states.csv'), x_opt, fmt=fmt_x, header=header_x)

    header_u= "delta; F_drive; F_brake; gamma_y"
    fmt_u = "%.5f; %.5f; %.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'ctrl_inputs.csv'), u_opt, fmt=fmt_u, header=header_u)

    header_s= "s_center"
    fmt_s= "%.5f"
    np.savetxt(os.path.join(export_path, 'length_center.csv'), s_center, fmt=fmt_s, header=header_s)

    header_s = "s_opt"
    fmt_s = "%.5f"
    # s= np.column_stack((s_opt, s_center))
    np.savetxt(os.path.join(export_path, 'length_opt.csv'), s_opt, fmt=fmt_s, header=header_s)

    header_k = "kappa_center"
    fmt_k = "%.5f"

    np.savetxt(os.path.join(export_path, 'kappa_center.csv'), kappa_center, fmt=fmt_k, header=header_k)

    header_k = "kappa_opt"
    fmt_k = "%.5f"
    np.savetxt(os.path.join(export_path, 'kappa_opt.csv'), kappa_opt, fmt=fmt_k, header=header_k)

    header_t = "t;"
    fmt_t = "%.5f"
    np.savetxt(os.path.join(export_path, 'time.csv'), t_opt, fmt=fmt_t, header=header_t)