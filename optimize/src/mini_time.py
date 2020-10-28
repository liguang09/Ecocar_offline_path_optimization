import sys
import time
import numpy as np
import casadi as ca
from parameters import maximum
from parameters import scale
from parameters import veh
from parameters import tire
from parameters import act

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
    # STATE VARIABLES
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

    # values after scaling
    v_s= scale.speed* v
    beta_s= scale.beta* beta
    omega_s= scale.omega* omega
    n_s= scale.n* n
    xi_s= scale.xi* xi

    delta_s= scale.delta* delta
    F_drive_s= scale.F_drive* F_drive
    F_brake_s= scale.F_brake* F_brake
    gamma_y_s= scale.gamma_y* gamma_y

    # stack as vector
    x_sf= np.array([scale.speed, scale.beta, scale.omega, scale.n, scale.xi])
    u_sf= np.array([scale.delta, scale.F_drive, scale.F_brake, scale.gamma_y])

    # ------------------------------------------------------------------------------------------------------------------
    # State & Control Boundaries
    # ------------------------------------------------------------------------------------------------------------------
    delta_min = -1
    delta_max = 1
    f_drive_min = 0.0
    f_drive_max = 1
    f_brake_min = -1
    f_brake_max = 0.0
    gamma_y_min = -np.inf
    gamma_y_max = np.inf

    v_min = 0
    v_max = 1
    beta_min = -1
    beta_max = 1
    omega_min = -1
    omega_max = 1
    xi_min = -1
    xi_max = 1

    # ------------------------------------------------------------------------------------------------------------------
    # MODEL EQUATIONS
    # ------------------------------------------------------------------------------------------------------------------
    Fd= 0.5* veh.Fd_coeff* v_s**2
    Fl = 0.5* veh.Fl_coeff * (v_s*ca.cos(beta_s))** 2
    #Fl = 0.5 * veh.Fl_coeff * v_s ** 2

    # tire x dimension
    Fx_fl= 0.5* veh.k_drive* F_drive_s+ 0.5* veh.k_brake* F_brake_s- 0.5*veh.fr*veh.m*veh.g*(veh.lr/veh.L)
    Fx_fr= Fx_fl
    Fx_rl= 0.5* (1- veh.k_drive)* F_drive_s+ 0.5* (1- veh.k_brake)* F_brake_s- 0.5*veh.fr*veh.m*veh.g*(veh.lf/veh.L)
    Fx_rr= Fx_rl

    # tire z dimension
    Fx_est= F_drive_s+ F_brake_s- Fd- veh.fr* veh.m* veh.g
    Fz_fl= 0.5* veh.m* veh.g* (veh.lr/veh.L)- 0.5* (veh.hcg/veh.L)* Fx_est- gamma_y_s* veh.k_roll+ 0.5* Fl
    Fz_fr= 0.5* veh.m* veh.g* (veh.lr/veh.L)- 0.5* (veh.hcg/veh.L)* Fx_est+ gamma_y_s* veh.k_roll+ 0.5* Fl
    Fz_rl= 0.5* veh.m* veh.g* (veh.lr/veh.L)+ 0.5* (veh.hcg/veh.L)* Fx_est- gamma_y_s* (1-veh.k_roll)+0.5* Fl
    Fz_rr= 0.5* veh.m* veh.g* (veh.lr/veh.L)+ 0.5* (veh.hcg/veh.L)* Fx_est+ gamma_y_s* (1-veh.k_roll)+0.5* Fl

    # tire y dimension
    alpha_fl= delta_s- ca.atan((veh.lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* veh.twf* omega_s))
    alpha_fr= delta_s- ca.atan((veh.lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* veh.twf* omega_s))
    alpha_rl= ca.atan((veh.lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* veh.twr* omega_s))
    alpha_rr= ca.atan((veh.lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* veh.twr* omega_s))

    # Pacejka's magic formula for Fy
    Fy_fl= tire.mu* Fz_fl* (1+ tire.eps_f* Fz_fl/ tire.Fz0)* ca.sin(tire.Cf* ca.atan(tire.Bf* alpha_fl- tire.Ef*(tire.Bf* alpha_fl- ca.atan(tire.Bf* alpha_fl))))
    Fy_fr= tire.mu* Fz_fr* (1+ tire.eps_f* Fz_fr/ tire.Fz0)* ca.sin(tire.Cf* ca.atan(tire.Bf* alpha_fr- tire.Ef*(tire.Bf* alpha_fr- ca.atan(tire.Bf* alpha_fr))))
    Fy_rl= tire.mu* Fz_rl* (1+ tire.eps_r* Fz_rl/ tire.Fz0)* ca.sin(tire.Cr* ca.atan(tire.Br* alpha_rl- tire.Er*(tire.Br* alpha_rl- ca.atan(tire.Br* alpha_rl))))
    Fy_rr= tire.mu* Fz_rr* (1+ tire.eps_r* Fz_rr/ tire.Fz0)* ca.sin(tire.Cr* ca.atan(tire.Br* alpha_rr- tire.Er*(tire.Br* alpha_rr- ca.atan(tire.Br* alpha_rr))))

    # Fx, Fy, Mz
    Fx= (Fx_fl+ Fx_fr)* ca.cos(delta_s)- (Fy_fl+ Fy_fr)* ca.sin(delta_s)+ (Fx_rl+ Fx_rr)
    Fy= (Fx_fl+ Fx_fr)* ca.sin(delta_s)+ (Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fy_rl+ Fy_rr)
    Mz= ((Fx_rr- Fx_rl)* veh.twr/2- (Fy_rl+ Fy_rr)* veh.lr+
                        ((Fx_fr- Fx_fl)* ca.cos(delta_s)+ (Fy_fl- Fy_fr)* ca.sin(delta_s))* veh.twf/2+
                        ((Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fx_fl+ Fx_fr)* ca.sin(delta_s))* veh.lf)

    # longitudinal acceleration [m/s²]
    ax = (Fx_rl+ Fx_rr+ (Fx_fl+ Fx_fr)* ca.cos(delta_s)- (Fy_fl+ Fy_fr)* ca.sin(delta_s)- Fd)/veh.m

    # lateral acceleration [m/s²]
    ay = ((Fx_fl+ Fx_fr)* ca.sin(delta_s)+ Fy_rl+ Fy_rr+ (Fy_fl+ Fy_fr)* ca.cos(delta_s))/ veh.m

    k_curv = ca.SX.sym("k_curv")

    # Derivatives
    SF= (1.0- n_s* k_curv)/(v_s* ca.cos(xi_s+ beta_s))
    dv= SF* (Fx* ca.cos(beta_s)+ Fy* ca.sin(beta_s)- Fd)/veh.m
    dbeta= SF* (omega_s+ (Fy* ca.cos(beta_s)- Fx* ca.sin(beta_s))/(veh.m* v_s))
    domega= SF* Mz/ veh.Izz
    dn= SF* v_s* ca.sin(xi_s+ beta_s)
    dxi= SF* omega_s- k_curv

    dx = ca.vertcat(dv, dbeta, domega, dn, dxi)/x_sf

    # ------------------------------------------------------------------------------------------------------------------
    # INITIAL
    # ------------------------------------------------------------------------------------------------------------------
    v_initial = 0.001

    # ------------------------------------------------------------------------------------------------------------------
    # FUNCTIONS
    # ------------------------------------------------------------------------------------------------------------------
    # continuous time dynamics
    f_dyn= ca.Function('f_dyn', [x, u, k_curv], [dx, SF], ['x', 'u', 'k_curv'], ['dx', 'SF'])
    f_a = ca.Function('f_a', [x, u], [ax, ay], ['x', 'u'], ['ax', 'ay'])

    # ------------------------------------------------------------------------------------------------------------------
    # DIRECT GAUSS-LEGENDRE COLLOCATION
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
    # Build NLP solver
    # ------------------------------------------------------------------------------------------------------------------
    # initialize NLP vectors
    w = []      # store states
    w0 = []     # initial values
    lbw = []    # states lower bound
    ubw = []    # states upper bound
    g = []      # store inputs
    lbg = []    # inputs lower bound
    ubg = []    # inputs upper bound
    J = 0

    # initialize ouput vectors
    x_opt = []
    u_opt = []
    dt_opt = []
    ax_opt = []
    ay_opt = []

    # boundary constraint: lift initial conditions
    Xk = ca.MX.sym('X0', nx)
    w.append(Xk)

    n_min = (-w_tr_right_interp(0)+ veh.width/2) /scale.n
    n_max = (w_tr_left_interp(0)- veh.width/2)/ scale.n

    lbw.append([v_min, beta_min, omega_min, n_min, xi_min])       # state lower bound
    ubw.append([v_max, beta_max, omega_max, n_max, xi_max])       # state upper bound
    w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])                      # state initial values
    x_opt.append(Xk* x_sf)

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

        # loop all collocation points
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
            g.append(ds*fj- xp)
            lbg.append([0.0]* nx)
            ubg.append([0.0]* nx)

            # add contribution to the end state
            Xk_end = Xk_end+ D[j]* Xc[j - 1]

            # add contribution to quadrature function
            J = J+ B[j]* qj* ds

            # add contribution to scaling factor (for calculating lap time)
            sf_opt.append(B[j]* qj* ds)

        # calculate used energy
        dt_opt.append(sf_opt[0]+ sf_opt[1]+ sf_opt[2])

        # add new decision variables for state at end of the collocation interval
        Xk= ca.MX.sym('X_'+ str(k+ 1), nx)
        w.append(Xk)
        n_min = (-w_tr_right_interp(k+ 1) + veh.width/2)/ scale.n
        n_max = (w_tr_left_interp(k+ 1) - veh.width/2)/ scale.n

        lbw.append([v_min, beta_min, omega_min, n_min, xi_min])
        ubw.append([v_max, beta_max, omega_max, n_max, xi_max])
        w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])

        # add equality constraint
        g.append(Xk_end - Xk)
        lbg.append([0.0] * nx)
        ubg.append([0.0] * nx)

        # get accelerations (longitudinal + lateral)
        axk, ayk = f_a(Xk, Uk)

        # limit engine power
        '''g.append(Xk[0] * Uk[1])
        lbg.append([0.0])
        ubg.append([maximum.power/ (scale.F_drive * scale.speed)])'''

        # Kamm's Circle for each wheel
        '''g.append(((Fx_flk / (tire.mu * Fz_flk)) ** 2 + (Fy_flk / (tire.mu * Fz_flk)) ** 2))
        g.append(((Fx_frk / (tire.mu * Fz_frk)) ** 2 + (Fy_frk / (tire.mu * Fz_frk)) ** 2))
        g.append(((Fx_rlk / (tire.mu * Fz_rlk)) ** 2 + (Fy_rlk / (tire.mu * Fz_rlk)) ** 2))
        g.append(((Fx_rrk / (tire.mu * Fz_rrk)) ** 2 + (Fy_rrk / (tire.mu * Fz_rrk)) ** 2))
        lbg.append([0.0] * 4)
        ubg.append([1.0] * 4)'''

        # lateral wheel load transfer
        '''g.append(((f_y_flk + f_y_frk) * ca.cos(Uk[0] * scale.delta) + f_y_rlk + f_y_rrk
                  +(f_x_flk + f_x_frk) * ca.sin(Uk[0] * scale.delta))* veh.hcg/ ((veh.twf + veh.twr)/2)- Uk[3]* scale.gamma_y)
        lbg.append([0.0])
        ubg.append([0.0])'''

        # no simultaneous of brake and drive
        g.append(Uk[1]* Uk[2])
        lbg.append([0])
        ubg.append([0.0])

        # actuator
        if k>0:
            SFk = (1 -kappa_interp(k)* Xk[3]* scale.n)/ (Xk[0]* scale.speed* ca.cos(Xk[4]*scale.xi+ Xk[1]*scale.beta))
            g.append((Uk- w[1+(k - 1)* nx])/(h* SFk))
            lbg.append([delta_min/ (act.steerT), -np.inf, f_brake_min/ (act.brakeT), -np.inf])
            ubg.append([delta_max/ (act.steerT), f_drive_max/(act.driveT), np.inf, np.inf])

        # limit corner speed
        if kappa_interp(k) !=0:
            mu= 0.4
            g.append(ca.power(Xk[0]*scale.speed,2)* ca.fabs(kappa_interp(k))- mu* veh.g)
            lbg.append([-np.inf])
            ubg.append([0])

        # append outputs
        x_opt.append(Xk* x_sf)
        u_opt.append(Uk* u_sf)
        ax_opt.append(axk)
        ay_opt.append(ayk)


    # start states = final states
    g.append(w[0]- Xk)
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
    dt_opt = ca.vertcat(*dt_opt)
    ax_opt = ca.vertcat(*ax_opt)
    ay_opt = ca.vertcat(*ay_opt)


    # ------------------------------------------------------------------------------------------------------------------
    # CREATE NLP SOLVER
    # ------------------------------------------------------------------------------------------------------------------

    # fill nlp structure
    nlp = {'f': J, 'x': w, 'g': g}

    # solver options
    opts = {"expand": True,
            "ipopt.max_iter": 3000,
            "ipopt.tol": 1e-7}

    # create solver instance
    solver = ca.nlpsol("solver", "ipopt", nlp, opts)

    # ------------------------------------------------------------------------------------------------------------------
    # SOLVE NLP
    # ------------------------------------------------------------------------------------------------------------------

    t_start = time.perf_counter()

    # solve NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

    t_end = time.perf_counter()

    if solver.stats()['return_status'] != 'Solve_Succeeded':
        print('\033[91m' + 'ERROR: Optimization fail!' + '\033[0m')
        sys.exit(1)

    # -----------------------------------------------------------------------------------------------------------------
    # EXTRACT SOLUTION -------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    f_sol = ca.Function('f_sol', [w], [x_opt, u_opt, dt_opt, ax_opt, ay_opt],
                        ['w'], ['x_opt', 'u_opt', 'dt_opt', 'ax_opt', 'ay_opt'])

    x_opt, u_opt, dt_opt, ax_opt, ay_opt= f_sol(sol['x'])

    # solution for state variables
    x_opt = np.reshape(x_opt, (N + 1, nx))

    # solution for control variables
    u_opt = np.reshape(u_opt, (N, nu))

    # solution for time
    t_opt = np.hstack((0.0, np.cumsum(dt_opt)))

    # solution for acceleration
    ax_opt = np.append(ax_opt[-1], ax_opt)
    ay_opt = np.append(ay_opt[-1], ay_opt)
    atot_opt = np.sqrt(np.power(ax_opt, 2) + np.power(ay_opt, 2))

    # solution for energy consumption
    #ec_opt_cum = np.hstack((0.0, np.cumsum(ec_opt))) / 3600.0

    return -x_opt[:-1, 3], x_opt[:-1, 0], u_opt, t_opt


# testing --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    pass
