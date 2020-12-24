import numpy as np
import os

# this function saves the optimized results into csv format, for plotting and analysis

def save_results(path_minitime: np.ndarray,
                 kappa_minitime: np.ndarray,
                 s_minitime: np.ndarray,
                 x_minitime: np.ndarray,
                 u_minitime: np.ndarray,
                 t_minitime: np.ndarray,
                 path_minidist: np.ndarray,
                 kappa_minidist: np.ndarray,
                 s_minidist: np.ndarray,
                 path_opt: np.ndarray,
                 kappa_opt: np.ndarray,
                 s_opt:np.ndarray,
                 path_center: np.ndarray,
                 kappa_center: np.ndarray,
                 s_center: np.ndarray,
                 bound_outer: np.ndarray,
                 bound_inner: np.ndarray) -> None:

    file_path = {}
    file_path["module"] = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(file_path["module"] + "/results", exist_ok=True)
    export_path = os.path.join(file_path["module"], "results")

#=======================================================================================================================
# Save time minimum trajectory results
#=======================================================================================================================
    header_traj= "path_x; path_y"
    fmt_traj= "%.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'path_minitime.csv'), path_minitime, fmt=fmt_traj, header=header_traj)
    np.savetxt(os.path.join(export_path, 'path_minidist.csv'), path_minidist, fmt=fmt_traj, header=header_traj)
    np.savetxt(os.path.join(export_path, 'path_opt.csv'), path_opt, fmt=fmt_traj, header=header_traj)
    np.savetxt(os.path.join(export_path, 'path_center.csv'), path_center, fmt=fmt_traj, header=header_traj)

    header_x= "v; beta; omega; n; xi;"
    fmt_x= "%.5f; %.5f; %.5f; %.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'states_minitime.csv'), x_minitime, fmt=fmt_x, header=header_x)

    header_u= "delta; F_drive; F_brake; gamma_y"
    fmt_u = "%.5f; %.5f; %.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'ctrl_inputs.csv'), u_minitime, fmt=fmt_u, header=header_u)

    header_s= "ds"
    fmt_s= "%.5f"
    np.savetxt(os.path.join(export_path, 'spline_center.csv'), s_center, fmt=fmt_s, header=header_s)
    np.savetxt(os.path.join(export_path, 'spline_minitime.csv'), s_minitime, fmt=fmt_s, header=header_s)
    np.savetxt(os.path.join(export_path, 'spline_minidist.csv'), s_minidist, fmt=fmt_s, header=header_s)
    np.savetxt(os.path.join(export_path, 'spline_opt.csv'), s_opt, fmt=fmt_s, header=header_s)

    header_k = "kappa"
    fmt_k = "%.5f"
    np.savetxt(os.path.join(export_path, 'kappa_center.csv'), kappa_center, fmt=fmt_k, header=header_k)
    np.savetxt(os.path.join(export_path, 'kappa_opt.csv'), kappa_opt, fmt=fmt_k, header=header_k)
    np.savetxt(os.path.join(export_path, 'kappa_minitime.csv'), kappa_minitime, fmt=fmt_k, header=header_k)
    np.savetxt(os.path.join(export_path, 'kappa_minidist.csv'), kappa_minidist, fmt=fmt_k, header=header_k)

    header_t = "t;"
    fmt_t = "%.5f"
    np.savetxt(os.path.join(export_path, 'time.csv'), t_minitime, fmt=fmt_t, header=header_t)

    header_outer = "outer_x; outer_y"
    fmt_outer = "%.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'bound_outer.csv'), bound_outer, fmt=fmt_outer, header=header_outer)

    header_inner = "inner_x; inner_y"
    fmt_inner = "%.5f; %.5f"
    np.savetxt(os.path.join(export_path, 'bound_inner.csv'), bound_inner, fmt=fmt_inner, header=header_inner)
