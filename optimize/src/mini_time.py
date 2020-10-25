import sys
import time
import numpy as np
import casadi as ca
from parameters import scale

def mini_time(track: np.ndarray,
              kappa: np.ndarray) -> tuple:

    num_point_track= track.shape[0]
    discr_point= np.arange(track.shape[0])
    discr_point = np.append(discr_point, track.shape[0])

    h=1
    step= [i for i in range(discr_point.size)]
    N= step[-1]

    kappa= np.append(kappa, kappa[0])
    w_tr_right= np.append(track[:, 2], track[0, 2])
    w_tr_left = np.append(track[:, 3], track[0, 3])


    kappa_interp= ca.interpolant('kappa_interp', 'linear', [step], kappa)
    w_tr_right_interp= ca.interpolant('w_tr_right_interp', 'linear', [step], w_tr_right)
    w_tr_left_interp= ca.interpolant('w_tr_left_interp', 'linear', [step], w_tr_left)



    # ------------------------------------------------------------------------------------------------------------------
    # DIRECT GAUSS-LEGENDRE COLLOCATION --------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    d = 3
    collocate_points = np.append(0, ca.collocation_points(d, 'legendre'))
    C = np.zeros((d + 1, d + 1))
    D = np.zeros(d + 1)
    B = np.zeros(d + 1)

    for i in range(d + 1):
        p = np.poly1d([1])
        for j in range(d + 1):
            if j != i:
                p *= np.poly1d([1, -collocate_points[j]]) / (collocate_points[i] - collocate_points[j])

        D[i] = p(1.0)

        dp = np.polyder(p)
        for j in range(d + 1):
            C[i, j] = dp(collocate_points[j])

        intp = np.polyint(p)
        B[i] = intp(1.0)

    # ------------------------------------------------------------------------------------------------------------------
    # STATE VARIABLES --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # States variables
    v = ca.SX.sym('v')  # velocity
    beta = ca.SX.sym('beta')  # slip angle
    omega = ca.SX.sym('omega')  # yaw rate
    n = ca.SX.sym('n')  # lateral distance to ref line (left:+)
    xi = ca.SX.sym('xi')  # relative angle to tangent on ref line

    x = ca.vertcat(v, beta, omega, n, xi)
    nx= x.size()[0]

    # Control variables
    delta = ca.SX.sym('delta')
    F_drive = ca.SX.sym('F_drive')
    F_brake = ca.SX.sym('F_brake')
    gamma_y = ca.SX.sym('gamma_y')

    u = ca.vertcat(delta, F_drive, F_brake, gamma_y)
    nu= u.size()[0]

    v_s= scale.speed* v
    beta_s= scale.beta* beta
    omega_s= scale.omega* omega
    n_s= scale.n* n
    xi_s= scale.xi* xi

    delta_s= scale.delta* delta
    F_drive_s= scale.F_drive* F_drive
    F_brake_s= scale.F_brake* F_brake
    gamma_y_s= scale.gamma_y* gamma_y

    x_sf= np.array([scale.speed, scale.beta, scale.omega, scale.n, scale.xi])
    u_sf= np.array([scale.delta, scale.F_drive, scale.F_brake, scale.gamma_y])

    # ------------------------------------------------------------------------------------------------------------------
    # MODEL EQUATIONS --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # Vehicle parameters
    lf= 1.516/2
    lr= 1.516/2
    L= lf+ lr
    m= 139+70
    g= 9.81
    twf= 1.08
    twr= 0.8
    hcg= 0.20

    Fd_coffe= 0.14
    Fl_coffe= 0.5
    fr= 0.013
    k_roll= 0.5

    # Pacejka's magic formula for Fy
    mu= 1.0
    eps_f= -0.1
    eps_r= -0.1
    Cf= 2.5
    Cr= 2.5
    Bf= 10.0
    Br= 10.0
    Ef= 0.5
    Er= 0.5
    Fz0= 1000
    Izz= 100

    k_drive= 0
    k_brake= 0.6


    Fd= 0.5* Fd_coffe* v_s**2
    # Fl= 0.5* Fl_coffe* (v_s* np.cos(beta_s))**2
    Fl = 0.5* Fl_coffe * (v_s*ca.cos(beta_s))** 2

    Fx_fl= 0.5* k_drive* F_drive_s+ 0.5* k_brake* F_brake_s- 0.5*fr*m*g*(lr/L)
    Fx_fr= Fx_fl
    Fx_rl= 0.5* (1- k_drive)* F_drive_s+ 0.5* (1- k_brake)* F_brake_s- 0.5*fr*m*g*(lf/L)
    Fx_rr= Fx_rl

    Fz_fl= 0.5* m* g* (lr/L)- 0.5* (hcg/L)* (F_drive_s+ F_brake_s- Fd- fr* m* g)- gamma_y_s* k_roll+ 0.5* Fl
    Fz_fr= 0.5* m* g* (lr/L)- 0.5* (hcg/L)* (F_drive_s+ F_brake_s- Fd- fr* m* g)+ gamma_y_s* k_roll+ 0.5* Fl
    Fz_rl= 0.5* m* g* (lf/L)+ 0.5* (hcg/L)* (F_drive_s+ F_brake_s- Fd- fr* m* g)- gamma_y_s* (1-k_roll)+0.5* Fl
    Fz_rr= 0.5* m* g* (lf/L)+ 0.5* (hcg/L)* (F_drive_s+ F_brake_s- Fd- fr* m* g)+ gamma_y_s* (1-k_roll)+0.5* Fl


    alpha_fl= delta_s- ca.atan((lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* twf* omega_s))
    alpha_fr= delta_s- ca.atan((lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* twf* omega_s))
    alpha_rl= ca.atan((lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* twr* omega_s))
    alpha_rr= ca.atan((lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* twr* omega_s))

    Fy_fl= mu* Fz_fl* (1+ eps_f* Fz_fl/ Fz0)* ca.sin(Cf* ca.atan(Bf* alpha_fl- Ef*(Bf* alpha_fl- ca.atan(Bf* alpha_fl))))
    Fy_fr= mu* Fz_fr* (1+ eps_f* Fz_fr/ Fz0)* ca.sin(Cf* ca.atan(Bf* alpha_fr- Ef*(Bf* alpha_fr- ca.atan(Bf* alpha_fr))))
    Fy_rl= mu* Fz_rl* (1+ eps_r* Fz_rl/ Fz0)* ca.sin(Cr* ca.atan(Br* alpha_rl- Er*(Br* alpha_rl- ca.atan(Br* alpha_rl))))
    Fy_rr= mu* Fz_rr* (1+ eps_r* Fz_rr/ Fz0)* ca.sin(Cr* ca.atan(Br* alpha_rr- Er*(Br* alpha_rr- ca.atan(Br* alpha_rr))))


    Fx= (Fx_fl+ Fx_fr)* ca.cos(delta_s)- (Fy_fl+ Fy_fr)* ca.sin(delta_s)+ (Fx_rl+ Fx_rr)
    Fy= (Fx_fl+ Fx_fr)* ca.sin(delta_s)+ (Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fy_rl+ Fy_rr)
    Mz= ((Fx_rr- Fx_rl)* twr/2- (Fy_rl+ Fy_rr)* lr+
                        ((Fx_fr- Fx_fl)* ca.cos(delta_s)+ (Fy_fl- Fy_fr)* ca.sin(delta_s))* twf/2+
                        ((Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fx_fl+ Fx_fr)* ca.sin(delta_s))* lf)

    # longitudinal acceleration [m/s²]
    ax = (Fx_rl+ Fx_rr+ (Fx_fl+ Fx_fr)* ca.cos(delta_s)- (Fy_fl+ Fy_fr)* ca.sin(delta_s)- Fd)/m

    # lateral acceleration [m/s²]
    ay = ((Fx_fl+ Fx_fr)* ca.sin(delta_s)+ Fy_rl+ Fy_rr+ (Fy_fl+ Fy_fr)* ca.cos(delta_s))/ m

    k_curv = ca.SX.sym("k_curv")

    SF= (1.0- n_s* k_curv)/(v_s* ca.cos(xi_s+ beta_s))

    dv= SF* (Fx* ca.cos(beta_s)+ Fy* ca.sin(beta_s)- Fd)/m
    dbeta= SF* (omega_s+ (Fy* ca.cos(beta_s)- Fx* ca.sin(beta_s))/(m* v_s))
    domega= SF* Mz/ Izz
    dn= SF* v_s* ca.sin(xi_s+ beta_s)
    dxi= SF* omega_s- k_curv

    dx = ca.vertcat(dv, dbeta, domega, dn, dxi)/x_sf

    # ------------------------------------------------------------------------------------------------------------------
    # BOUNDARIES -----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    delta_min = -1.0
    delta_max = 1.0
    f_drive_min = 0.0
    f_drive_max = 1.0
    f_brake_min = -1.0
    f_brake_max = 0.0
    gamma_y_min = -np.inf
    gamma_y_max = np.inf

    v_min = 0.0
    v_max = 1.0
    beta_min = -1.0
    beta_max = 1.0
    omega_z_min = -1.0
    omega_z_max = 1.0
    xi_min = -1.0
    xi_max = 1.0

    # ------------------------------------------------------------------------------------------------------------------
    # INITIAL
    # ------------------------------------------------------------------------------------------------------------------
    v_initial = 0.01

    # ------------------------------------------------------------------------------------------------------------------
    # FUNCTIONS
    # ------------------------------------------------------------------------------------------------------------------

    # continuous time dynamics
    f_dyn = ca.Function('f_dyn', [x, u, k_curv], [dx, SF], ['x', 'u', 'k_curv'], ['dx', 'SF'])

    # longitudinal tire forces [N]
    f_fx = ca.Function('f_fx', [x, u], [Fx_fl, Fx_fr, Fx_rl, Fx_rr], ['x', 'u'], ['Fx_fl', 'Fx_fr', 'Fx_rl', 'Fx_rr'])
    # lateral tire forces [N]
    f_fy = ca.Function('f_fy', [x, u], [Fy_fl, Fy_fr, Fy_rl, Fy_rr], ['x', 'u'], ['Fy_fl', 'Fy_fr', 'Fy_rl', 'Fy_rr'])
    # vertical tire forces [N]
    f_fz = ca.Function('f_fz', [x, u], [Fz_fl, Fz_fr, Fz_rl, Fz_rr], ['x', 'u'], ['Fz_fl', 'Fz_fr', 'Fz_rl', 'Fz_rr'])
    # longitudinal and lateral acceleration [m/s²]
    f_a = ca.Function('f_a', [x, u], [ax, ay], ['x', 'u'], ['ax', 'ay'])

    # ------------------------------------------------------------------------------------------------------------------
    # Build NLP solver
    # ------------------------------------------------------------------------------------------------------------------
    # initialize NLP vectors
    w = []      # store states
    w0 = []     # initial values
    lbw = []    # states lower bound
    ubw = []    # states upper bound
    J = 0
    g = []      # store inputs
    lbg = []    # inputs lower bound
    ubg = []    # inputs upper bound

    # initialize ouput vectors
    x_opt = []
    u_opt = []
    dt_opt = []
    tf_opt = []
    ax_opt = []
    ay_opt = []
    ec_opt = []

    # boundary constraint: lift initial conditions
    Xk = ca.MX.sym('X0', nx)
    w.append(Xk)

    n_min = (-w_tr_right_interp(0) + 3.4/2) /scale.n
    n_max = (w_tr_left_interp(0) - 3.4/2)/ scale.n

    lbw.append([v_min, beta_min, omega_z_min, n_min, xi_min])       # state lower bound
    ubw.append([v_max, beta_max, omega_z_max, n_max, xi_max])       # state upper bound

    w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])        # state initial values
    x_opt.append(Xk* x_sf)

    # loop along the racetrack and formulate path constraints & system dynamic
    # retrieve step-sizes of optimization along reference line
    # h = np.diff(s_opt)
    ds= h
    for k in range(N):
        # add decision variables for the control
        Uk = ca.MX.sym('U_' + str(k), nu)
        w.append(Uk)
        lbw.append([delta_min, f_drive_min, f_brake_min, gamma_y_min])
        ubw.append([delta_max, f_drive_max, f_brake_max, gamma_y_max])
        w0.append([0.0] * nu)

        # add decision variables for the state at collocation points
        Xc = []
        for j in range(d):
            Xkj= ca.MX.sym('X_'+ str(k)+ '_'+ str(j), nx)
            Xc.append(Xkj)
            w.append(Xkj)
            lbw.append([-np.inf] * nx)
            ubw.append([np.inf] * nx)

            w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])

        # loop over all collocation points
        Xk_end = D[0]* Xk
        sf_opt = []

        for j in range(1, d + 1):
            # calculate the state derivative at the collocation point
            xp = C[0, j] * Xk
            for r in range(d):
                xp = xp + C[r+ 1, j] * Xc[r]

            # interpolate kappa at the collocation point
            kappa_col = kappa_interp(k + collocate_points[j])

            # append collocation equations (system dynamic)
            fj, qj = f_dyn(Xc[j - 1], Uk, kappa_col)
            g.append(ds* fj- xp)
            lbg.append([0.0]* nx)
            ubg.append([0.0]* nx)

            # add contribution to the end state
            Xk_end = Xk_end+ D[j]* Xc[j - 1]

            # add contribution to quadrature function
            J = J+ B[j]* qj* ds

            # add contribution to scaling factor (for calculating lap time)
            sf_opt.append(B[j]* qj* ds)

        # calculate used energy
        dt_opt.append(sf_opt[0] + sf_opt[1] + sf_opt[2])
        ec_opt.append(Xk[0]* scale.speed* Uk[1]* scale.F_drive* dt_opt[-1])

        # add new decision variables for state at end of the collocation interval
        Xk = ca.MX.sym('X_'+ str(k + 1), nx)
        w.append(Xk)
        n_min = (-w_tr_right_interp(k + 1) + 3.4/ 2.0)/ scale.n
        n_max = (w_tr_left_interp(k + 1) - 3.4/ 2.0)/ scale.n

        lbw.append([v_min, beta_min, omega_z_min, n_min, xi_min])
        ubw.append([v_max, beta_max, omega_z_max, n_max, xi_max])
        w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])

        # add equality constraint
        g.append(Xk_end - Xk)
        lbg.append([0.0] * nx)
        ubg.append([0.0] * nx)

        # get tire forces
        f_x_flk, f_x_frk, f_x_rlk, f_x_rrk = f_fx(Xk, Uk)
        f_y_flk, f_y_frk, f_y_rlk, f_y_rrk = f_fy(Xk, Uk)
        f_z_flk, f_z_frk, f_z_rlk, f_z_rrk = f_fz(Xk, Uk)

        # get accelerations (longitudinal + lateral)
        axk, ayk = f_a(Xk, Uk)

        # path constraint: limitied engine power

        # path constraint: Kamm's Circle for each wheel

        # path constraint: lateral wheel load transfer

        # path constraint: f_drive * f_brake == 0 (no simultaneous operation of brake and accelerator pedal)

        # path constraint: actor dynamic

        # append outputs
        x_opt.append(Xk * x_sf)
        u_opt.append(Uk * u_sf)
        tf_opt.extend([f_x_flk, f_y_flk, f_z_flk, f_x_frk, f_y_frk, f_z_frk])
        tf_opt.extend([f_x_rlk, f_y_rlk, f_z_rlk, f_x_rrk, f_y_rrk, f_z_rrk])
        ax_opt.append(axk)
        ay_opt.append(ayk)


    # boundary constraint: start states = final states
    g.append(w[0] - Xk)

    lbg.append([0.0, 0.0, 0.0, 0.0, 0.0])
    ubg.append([0.0, 0.0, 0.0, 0.0, 0.0])


    # formulate differentiation matrix (for regularization)

    # regularization (delta)

    # regularization (f_drive + f_brake)

    # formulate objective
    #J = J + pars["optim_opts"]["penalty_F"] * Jp_f + pars["optim_opts"]["penalty_delta"] * Jp_delta

    # concatenate NLP vectors
    w = ca.vertcat(*w)
    g = ca.vertcat(*g)
    w0 = np.concatenate(w0)
    lbw = np.concatenate(lbw)
    ubw = np.concatenate(ubw)
    lbg = np.concatenate(lbg)
    ubg = np.concatenate(ubg)

    # concatenate output vectors
    x_opt = ca.vertcat(*x_opt)
    u_opt = ca.vertcat(*u_opt)
    tf_opt = ca.vertcat(*tf_opt)
    dt_opt = ca.vertcat(*dt_opt)
    ax_opt = ca.vertcat(*ax_opt)
    ay_opt = ca.vertcat(*ay_opt)
    ec_opt = ca.vertcat(*ec_opt)

    # ------------------------------------------------------------------------------------------------------------------
    # CREATE NLP SOLVER
    # ------------------------------------------------------------------------------------------------------------------

    # fill nlp structure
    nlp = {'f': J, 'x': w, 'g': g}

    # solver options
    print_debug= False
    opts = {"expand": True,
            "verbose": print_debug,
            "ipopt.max_iter": 3000,
            "ipopt.tol": 1e-7}

    # create solver instance
    solver = ca.nlpsol("solver", "ipopt", nlp, opts)

    # ------------------------------------------------------------------------------------------------------------------
    # SOLVE NLP
    # ------------------------------------------------------------------------------------------------------------------

    # start time measure
    t0 = time.perf_counter()

    # solve NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

    # end time measure
    tend = time.perf_counter()

    if solver.stats()['return_status'] != 'Solve_Succeeded':
        print('\033[91m' + 'ERROR: Optimization did not succeed!' + '\033[0m')
        sys.exit(1)

    # ------------------------------------------------------------------------------------------------------------------
    # EXTRACT SOLUTION -------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # helper function to extract solution for state variables, control variables, tire forces, time
    f_sol = ca.Function('f_sol', [w], [x_opt, u_opt, tf_opt, dt_opt, ax_opt, ay_opt, ec_opt],
                        ['w'], ['x_opt', 'u_opt', 'tf_opt', 'dt_opt', 'ax_opt', 'ay_opt', 'ec_opt'])


    # extract solution
    x_opt, u_opt, tf_opt, dt_opt, ax_opt, ay_opt, ec_opt = f_sol(sol['x'])

    # solution for state variables
    x_opt = np.reshape(x_opt, (N + 1, nx))

    # solution for control variables
    u_opt = np.reshape(u_opt, (N, nu))

    # solution for tire forces
    tf_opt = np.append(tf_opt[-12:], tf_opt[:])
    tf_opt = np.reshape(tf_opt, (N + 1, 12))

    # solution for time
    t_opt = np.hstack((0.0, np.cumsum(dt_opt)))

    # solution for acceleration
    ax_opt = np.append(ax_opt[-1], ax_opt)
    ay_opt = np.append(ay_opt[-1], ay_opt)
    atot_opt = np.sqrt(np.power(ax_opt, 2) + np.power(ay_opt, 2))

    # solution for energy consumption
    ec_opt_cum = np.hstack((0.0, np.cumsum(ec_opt))) / 3600.0

    return -x_opt[:-1, 3], x_opt[:-1, 0], u_opt, t_opt


# testing --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    pass
