import sys
import time
import numpy as np
import casadi as ca
from parameters import scale
from parameters import veh
from parameters import tire
from parameters import act
from parameters import trk
from parameters import resist
from parameters import optimize
from parameters import regular

def mini_time(track: np.ndarray,
              kappa: np.ndarray,
              grad_info: np.ndarray) -> tuple:

    dis_points= np.arange(track.shape[0])
    dis_points = np.append(dis_points, track.shape[0])

    h= 1    # optimized step
    step= [i for i in range(dis_points.size)]
    N= step[-1]

    kappa= np.append(kappa, kappa[0])
    n_right= np.append(track[:, 2], track[0, 2])
    n_left = np.append(track[:, 3], track[0, 3])

    kappa_interp= ca.interpolant('kappa_interp', 'linear', [step], kappa)
    n_right_interp= ca.interpolant('n_right', 'linear', [step], n_right)
    n_left_interp= ca.interpolant('n_left', 'linear', [step], n_left)

    # ==================================================================================================================
    # STATE VARIABLES
    # ==================================================================================================================
    # States variables
    v = ca.SX.sym('v')
    beta = ca.SX.sym('beta')
    omega = ca.SX.sym('omega')
    n = ca.SX.sym('n')
    xi = ca.SX.sym('xi')

    x = ca.vertcat(v, beta, omega, n, xi)
    num_x= x.size()[0]

    # Control variables
    delta = ca.SX.sym('delta')
    F_drive = ca.SX.sym('F_drive')
    F_brake = ca.SX.sym('F_brake')
    gamma_y = ca.SX.sym('gamma_y')
    grad= ca.SX.sym('grad')

    u = ca.vertcat(delta, F_drive, F_brake, gamma_y, grad)
    num_u= u.size()[0]

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
    grad_s= 1* grad

    # stack as vector
    x_sf= np.array([scale.speed, scale.beta, scale.omega, scale.n, scale.xi])
    u_sf= np.array([scale.delta, scale.F_drive, scale.F_brake, scale.gamma_y, 1])

    # ==================================================================================================================
    # State & Control Boundaries
    # ==================================================================================================================
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
    omega_min = -1.0
    omega_max = 1.0
    xi_min = -1.0
    xi_max = 1.0

    # ==================================================================================================================
    # Vehicle Model
    # ==================================================================================================================
    # drag force
    Fd= 0.5* resist.rho* resist.A* resist.Cd* v_s**2

    # Rolling resistance
    Fr = resist.Cr * veh.m * veh.g / veh.wheelR * 4

    # Down force
    #Fl = 0.5* resist.rho* resist.A* resist.Cl* v_s** 2
    Fl= 0.5* resist.rho* resist.A* resist.Cl* (v_s*ca.cos(beta_s))** 2


    # tire x dimension
    Fx_fl= 0.5* veh.k_drive* F_drive_s+ 0.5* veh.k_brake* F_brake_s- 0.5*(Fr)*(veh.lr/veh.L)
    Fx_fr= Fx_fl
    Fx_rl= 0.5* (1- veh.k_drive)* F_drive_s+ 0.5* (1- veh.k_brake)* F_brake_s- 0.5*(Fr)*(veh.lf/veh.L)
    Fx_rr= Fx_rl

    # tire z dimension
    Fx_est= F_drive_s+ F_brake_s- Fd- Fr
    Fz_fl= 0.5* veh.m* veh.g* (veh.lr/veh.L)- 0.5* (veh.hcg/veh.L)* Fx_est- gamma_y_s* veh.k_roll+ 0.5* Fl
    Fz_fr= 0.5* veh.m* veh.g* (veh.lr/veh.L)- 0.5* (veh.hcg/veh.L)* Fx_est+ gamma_y_s* veh.k_roll+ 0.5* Fl
    Fz_rl= 0.5* veh.m* veh.g* (veh.lr/veh.L)+ 0.5* (veh.hcg/veh.L)* Fx_est- gamma_y_s* (1-veh.k_roll)+ 0.5* Fl
    Fz_rr= 0.5* veh.m* veh.g* (veh.lr/veh.L)+ 0.5* (veh.hcg/veh.L)* Fx_est+ gamma_y_s* (1-veh.k_roll)+ 0.5* Fl

    # tire y dimension
    alpha_fl= delta_s- ca.atan((veh.lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* veh.twf* omega_s))
    alpha_fr= delta_s- ca.atan((veh.lf* omega_s+ v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* veh.twf* omega_s))
    alpha_rl= ca.atan((veh.lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)- 0.5* veh.twr* omega_s))
    alpha_rr= ca.atan((veh.lr* omega_s- v_s* ca.sin(beta_s))/ (v_s* ca.cos(beta_s)+ 0.5* veh.twr* omega_s))

    # Pacejka's magic formula for Fy
    Fy_fl= tire.mu* Fz_fl* (1+ tire.eps* Fz_fl/ tire.Fz0)* \
           ca.sin(tire.C* ca.atan(tire.B* alpha_fl- tire.E*(tire.B* alpha_fl- ca.atan(tire.B* alpha_fl))))
    Fy_fr= tire.mu* Fz_fr* (1+ tire.eps* Fz_fr/ tire.Fz0)* \
           ca.sin(tire.C* ca.atan(tire.B* alpha_fr- tire.E*(tire.B* alpha_fr- ca.atan(tire.B* alpha_fr))))
    Fy_rl= tire.mu* Fz_rl* (1+ tire.eps* Fz_rl/ tire.Fz0)* \
           ca.sin(tire.C* ca.atan(tire.B* alpha_rl- tire.E*(tire.B* alpha_rl- ca.atan(tire.B* alpha_rl))))
    Fy_rr= tire.mu* Fz_rr* (1+ tire.eps* Fz_rr/ tire.Fz0)* \
           ca.sin(tire.C* ca.atan(tire.B* alpha_rr- tire.E*(tire.B* alpha_rr- ca.atan(tire.B* alpha_rr))))

    # Fx, Fy, Mz
    Fx= (Fx_fl+ Fx_fr)* ca.cos(delta_s)- (Fy_fl+ Fy_fr)* ca.sin(delta_s)+ (Fx_rl+ Fx_rr)
    Fy= (Fx_fl+ Fx_fr)* ca.sin(delta_s)+ (Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fy_rl+ Fy_rr)
    Mz= (Fx_rr- Fx_rl)* veh.twr/2- (Fy_rl+ Fy_rr)* veh.lr+\
        ((Fx_fr- Fx_fl)* ca.cos(delta_s)+ (Fy_fl- Fy_fr)* ca.sin(delta_s))* \
        veh.twf/2+((Fy_fl+ Fy_fr)* ca.cos(delta_s)+ (Fx_fl+ Fx_fr)* ca.sin(delta_s))* veh.lf

    '''Mz = (Fx_rl - Fx_rr) * veh.twr/ 2 - (Fy_rl + Fy_rr) * veh.lr +\
         ((Fx_fl - Fx_fr) * ca.cos(delta_s) + (Fy_fl - Fy_fr) * ca.sin(delta_s)) * veh.twf / 2 +\
         ((Fy_fl + Fy_fr) * ca.cos(delta_s) + (Fx_fl + Fx_fr) * ca.sin(delta_s)) * veh.lf'''

    k_curv = ca.SX.sym("k_curv")

    # Derivatives
    SF= (1.0- n_s* k_curv)/(v_s* ca.cos(xi_s+ beta_s))  #dt/ds

    dn = SF* v_s* ca.sin(xi_s + beta_s)
    dxi = SF* omega_s- k_curv
    dv= SF* (Fx* ca.cos(beta_s)+ Fy* ca.sin(beta_s)- Fd*ca.cos(beta_s)-veh.m* veh.g* ca.atan(grad_s))/veh.m
    dbeta= SF* (omega_s+ (Fy* ca.cos(beta_s)- Fx* ca.sin(beta_s)+Fd*ca.sin(beta_s))/(veh.m* v_s))
    domega= SF* Mz/ veh.Izz

    dx = ca.vertcat(dv, dbeta, domega, dn, dxi)/x_sf

    v_initial = 0.1

    f_dyn= ca.Function('f_dyn', [x, u, k_curv], [dx, SF], ['x', 'u', 'k_curv'], ['dx', 'SF'])
    fx_wheels = ca.Function('fx_wheels', [x, u], [Fx_fl, Fx_fr, Fx_rl, Fx_rr], ['x', 'u'], ['Fx_fl', 'Fx_fr', 'Fx_rl', 'Fx_rr'])
    fy_wheels = ca.Function('fy_wheels', [x, u], [Fy_fl, Fy_fr, Fy_rl, Fy_rr], ['x', 'u'], ['Fy_fl', 'Fy_fr', 'Fy_rl', 'Fy_rr'])
    fz_wheels = ca.Function('fz_wheels', [x, u], [Fz_fl, Fz_fr, Fz_rl, Fz_rr], ['x', 'u'], ['Fz_fl', 'Fz_fr', 'Fz_rl', 'Fz_rr'])
    Fxy= ca.Function('Fxy',[x, u], [Fx, Fy], ['x', 'u'], ['Fx', 'Fy'])

    # ==================================================================================================================
    # DIRECT COLLOCATION
    # ==================================================================================================================
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

    # ==================================================================================================================
    # NLP solver
    # ==================================================================================================================
    # initialize NLP vectors
    w = []      # store states
    w0 = []     # initial values
    lbw = []    # states lower bound
    ubw = []    # states upper bound
    g = []      # store inputs
    lbg = []    # inputs lower bound
    ubg = []    # inputs upper bound


    J = 0

    # ouput vectors
    x_opt = []
    u_opt = []
    dt_opt = []

    delta_p = []
    F_p = []

    # states and controls initial boundaries
    Xk = ca.MX.sym('X0', num_x)
    w.append(Xk)

    n_min = (-n_right_interp(0)+ veh.width) /scale.n
    n_max = (n_left_interp(0)- veh.width)/ scale.n

    lbw.append([v_min, beta_min, omega_min, n_min, xi_min])       # state lower bound
    ubw.append([v_max, beta_max, omega_max, n_max, xi_max])       # state upper bound
    w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])                    # state initial values

    x_opt.append(Xk* x_sf)

    ds= h
    for k in range(N):
        # add decision variables for the control
        Uk = ca.MX.sym('U_' + str(k), num_u)
        w.append(Uk)
        lbw.append([delta_min, f_drive_min, f_brake_min, gamma_y_min, grad_info[k]])
        ubw.append([delta_max, f_drive_max, f_brake_max, gamma_y_max, grad_info[k]])
        w0.append([0.0]* num_u)


        # add decision variables for the state at collocation points
        Xc = []
        for j in range(d):
            Xkj= ca.MX.sym('X_'+ str(k)+ '_'+ str(j), num_x)
            Xc.append(Xkj)
            w.append(Xkj)
            lbw.append([-np.inf]* num_x)
            ubw.append([np.inf]* num_x)
            w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])

        # loop all collocation points
        Xk_end = D[0]* Xk
        t_opt = []

        for j in range(1, d + 1):
            # calculate the state derivative at the collocation point
            xp = C[0, j] * Xk
            for r in range(d):
                xp = xp + C[r+ 1, j] * Xc[r]

            # interpolate kappa at the collocation point
            kappa_col = kappa_interp(k + collocate_points[j])

            # append collocation equations
            fj, qj = f_dyn(Xc[j - 1], Uk, kappa_col)
            g.append(ds*fj- xp)
            lbg.append([0.0]* num_x)
            ubg.append([0.0]* num_x)

            # add contribution to the end state
            Xk_end = Xk_end+ D[j]* Xc[j - 1]

            J = J+ B[j]* qj* ds

            # add contribution to scaling factor (for calculating lap time)
            t_opt.append(B[j]* qj* ds)

        dt_opt.append(t_opt[0]+ t_opt[1]+ t_opt[2])

        # add new decision variables for state at end of the collocation interval
        Xk= ca.MX.sym('X_'+ str(k+ 1), num_x)
        w.append(Xk)
        n_min = (-n_right_interp(k+1)+ veh.width*veh.safe)/ scale.n
        n_max = (n_left_interp(k+1)- veh.width*veh.safe)/ scale.n

        lbw.append([v_min, beta_min, omega_min, n_min, xi_min])
        ubw.append([v_max, beta_max, omega_max, n_max, xi_max])
        w0.append([v_initial, 0.0, 0.0, 0.0, 0.0])

        # add equality constraint
        g.append(Xk_end- Xk)
        lbg.append([0.0]* num_x)
        ubg.append([0.0]* num_x)

        Fx_flk, Fx_frk, Fx_rlk, Fx_rrk = fx_wheels(Xk, Uk)
        Fy_flk, Fy_frk, Fy_rlk, Fy_rrk = fy_wheels(Xk, Uk)
        Fz_flk, Fz_frk, Fz_rlk, Fz_rrk = fz_wheels(Xk, Uk)
        Fxk,Fyk= Fxy(Xk,Uk)

        # gamma_y equal constraints
        Fy_k = (Fx_flk + Fx_frk) * ca.sin(Uk[0] * scale.delta) + \
               (Fy_flk + Fy_frk) * ca.cos(Uk[0] * scale.delta) + (Fy_rlk + Fy_rrk)
        g.append(Fy_k * veh.hcg / ((veh.twf + veh.twr) / 2) - Uk[3] * scale.gamma_y)
        lbg.append([0.0])
        ubg.append([0.0])

        # Kamm circle for individual wheel
        '''g.append(((Fx_flk / (trk.mu * Fz_flk)) ** 2 + (Fy_flk / (trk.mu * Fz_flk)) ** 2))
        g.append(((Fx_frk / (trk.mu * Fz_frk)) ** 2 + (Fy_frk / (trk.mu * Fz_frk)) ** 2))
        g.append(((Fx_rlk / (trk.mu * Fz_rlk)) ** 2 + (Fy_rlk / (trk.mu * Fz_rlk)) ** 2))
        g.append(((Fx_rrk / (trk.mu * Fz_rrk)) ** 2 + (Fy_rrk / (trk.mu * Fz_rrk)) ** 2))
        lbg.append([0.0] * 4)
        ubg.append([1.0] * 4)'''

        # Kamm circle for whole car
        g.append((Fxk**2+ Fyk**2)/((trk.mu * veh.m * veh.g)**2))
        lbg.append([0.0])
        ubg.append([1.0])

        # brake and drive are not activated at same time
        g.append(Uk[1]* Uk[2])
        lbg.append([0.0])
        ubg.append([1/8000])

        # actuator derivative
        if k>0:
            SFk = (1 -kappa_interp(k)* Xk[3]* scale.n)/ (Xk[0]* scale.speed* ca.cos(Xk[4]*scale.xi+ Xk[1]*scale.beta))
            U_tmp= (Uk- w[1+(k-1)* num_x])
            #U_tmp = (Uk - w[1+(k -1) * (num_x+1)]) #d=4
            g.append(U_tmp/(ds* SFk))
            lbg.append([delta_min/ (act.steerT), -np.inf, f_brake_min/ (act.brakeT), -np.inf, -np.inf])
            ubg.append([delta_max/ (act.steerT), f_drive_max/(act.driveT), np.inf, np.inf, np.inf])

        # engine constraint
        '''g.append((Uk[1]- 1)*(Uk[1]- 0.5)*Uk[1])
        lbg.append([0.0])
        ubg.append([0.0])'''

        delta_p.append(Uk[0]*scale.delta)
        F_p.append(Uk[1]*scale.F_drive+ Uk[2]*scale.F_brake)

        # append outputs
        x_opt.append(Xk* x_sf)
        u_opt.append(Uk* u_sf)

    # start states = final states
    g.append(w[0]- Xk)
    lbg.append([0.0]*num_x)
    ubg.append([0.0]*num_x)

    # Regularation
    D_matrix = np.eye(N)
    for i in range(N - 1):
        D_matrix[i, i + 1] = -1.0
    D_matrix[N - 1, 0] = -1.0

    delta_p = ca.vertcat(*delta_p)
    Jp_delta = ca.mtimes(ca.MX(D_matrix), delta_p)
    Jp_delta = ca.dot(Jp_delta, Jp_delta)

    F_p = ca.vertcat(*F_p)
    Jp_f = ca.mtimes(ca.MX(D_matrix), F_p)
    Jp_f = ca.dot(Jp_f, Jp_f)

    # formulate objective
    J = J+ regular.Q_F* Jp_f+ regular.Q_delta* Jp_delta

    w = ca.vertcat(*w)
    g = ca.vertcat(*g)
    w0 = np.concatenate(w0)
    lbw = np.concatenate(lbw)
    ubw = np.concatenate(ubw)
    lbg = np.concatenate(lbg)
    ubg = np.concatenate(ubg)

    x_opt = ca.vertcat(*x_opt)
    u_opt = ca.vertcat(*u_opt)
    dt_opt = ca.vertcat(*dt_opt)

    nlp = {'f': J, 'x': w, 'g': g}

    opts = {"expand": True,
            "ipopt.max_iter": optimize.iter,
            "ipopt.tol": optimize.accu}

    solver = ca.nlpsol("solver", "ipopt", nlp, opts)

    t_start = time.perf_counter()
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    t_end = time.perf_counter()

    print("INFO: NLP Time Cost: ", t_end- t_start)

    if solver.stats()['return_status'] != 'Solve_Succeeded':
        print('Optimization Fail! Try Again')
        sys.exit(1)

    # ==================================================================================================================
    # EXTRACT SOLUTION
    # ==================================================================================================================
    f_sol = ca.Function('f_sol', [w], [x_opt, u_opt, dt_opt], ['w'], ['x_opt', 'u_opt', 'dt_opt'])
    x_opt, u_opt, dt_opt= f_sol(sol['x'])

    x_opt = np.reshape(x_opt, (N+1, num_x))
    u_opt = np.reshape(u_opt, (N, num_u))
    t_opt = np.hstack((0.0, np.cumsum(dt_opt)))

    return  x_opt, u_opt[:, 0:4], t_opt