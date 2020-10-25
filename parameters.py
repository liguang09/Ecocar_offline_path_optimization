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
# Scaling factors
#-----------------------------------------------------------------------------------------------------------------------
scale = Const()
scale.speed= 7
scale.beta= np.pi
scale.omega= np.pi/2
scale.n= 4.0
scale.xi= np.pi/2
scale.delta= 15/180*np.pi
scale.F_drive= 2000.0
scale.F_brake= 2000.0
scale.gamma_y= 500.0

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