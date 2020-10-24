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

