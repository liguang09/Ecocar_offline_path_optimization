import numpy as np

class Const(object):
    class ConstError(TypeError):
        pass

    class ConstCaseError(ConstError):
        pass

    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise self.ConstError("Can't change const.%s" % name)
        self.__dict__[name] = value


#-----------------------------------------------------------------------------------------------------------------------
# maximum and minimum values
#-----------------------------------------------------------------------------------------------------------------------
maximum= Const()
maximum.speed= 70
maximum.F_drive= 7000
maximum.F_brake= 20000
maximum.delta= 0.35
maximum.power= 230000

#-----------------------------------------------------------------------------------------------------------------------
# Scaling factors
#-----------------------------------------------------------------------------------------------------------------------
scale = Const()
scale.speed= 50
scale.beta= 0.5
scale.omega= 1
scale.n= 5
scale.xi= 1

scale.delta= 0.5
scale.F_drive= 7500
scale.F_brake= 20000.0
scale.gamma_y= 5000.0

#-----------------------------------------------------------------------------------------------------------------------
# Vehicle parameters
#-----------------------------------------------------------------------------------------------------------------------
veh= Const()
veh.lf = 1.516 / 2
veh.lr = 1.516 / 2
veh.L = veh.lf + veh.lr
veh.m = 139 + 70
veh.g = 9.81
veh.twf = 1.08
veh.twr = 0.8
veh.hcg = 0.20
veh.Izz = 100

veh.Fd_coeff = 0.14
veh.Fl_coeff = 0.5
veh.fr = 0.013
veh.k_roll = 0.5

veh.k_drive= 0
veh.k_brake= 0.6
#-----------------------------------------------------------------------------------------------------------------------
# Tire
#-----------------------------------------------------------------------------------------------------------------------
tire= Const()
tire.mu= 1.0
tire.eps_f = -0.1
tire.eps_r = -0.1
tire.Cf = 2.5
tire.Cr = 2.5
tire.Bf = 10.0
tire.Br = 10.0
tire.Ef = 0.5
tire.Er = 0.5
tire.Fz0 = 1000

