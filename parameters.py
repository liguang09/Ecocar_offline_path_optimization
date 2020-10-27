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


#=======================================================================================================================
# maximum values
#=======================================================================================================================
maximum= Const()
maximum.speed= 25/3.6

maximum.delta= 15*np.pi/180
maximum.F_drive= 5000 # not sure
maximum.F_brake= 5000 # not sure

maximum.power= 20000 # not sure

#=======================================================================================================================
# Track
#=======================================================================================================================
track= Const()
track.width= 6.5

#=======================================================================================================================
# Scaling factors
#=======================================================================================================================
scale = Const()
# states
scale.speed= maximum.speed
scale.beta= 0.5* np.pi
scale.omega= np.pi
scale.n= track.width/2
scale.xi= np.pi

# control inputs
scale.delta= maximum.delta
scale.F_drive= maximum.F_drive
scale.F_brake= maximum.F_brake
scale.gamma_y= 5000.0

#=======================================================================================================================
# Vehicle parameters
#=======================================================================================================================
veh= Const()
veh.lf = 1.516/ 2
veh.lr = 1.516/ 2
veh.L = veh.lf + veh.lr
veh.m = 139 + 70 # 139-> car net weight, 70-> estimated driver weight
veh.g = 9.81
veh.twf = 1.08
veh.twr = 0.8
veh.hcg = 0.20
veh.width= 1.3*2
veh.length= 3.4
veh.Izz = 0.2*veh.m*(np.power(veh.width,2)+ np.power(veh.length,2))


veh.Fd_coeff = 0.14
veh.Fl_coeff = 0.5 # not sure
veh.fr = 0.013 # not sure
veh.k_roll = 0.5 # not sure

veh.k_drive= 0
veh.k_brake= 0.5
#=======================================================================================================================
# Tire
#=======================================================================================================================
tire= Const() # not sure
tire.mu= 1.0
tire.eps_f = -0.1
tire.eps_r = -0.1
tire.Cf = 2.5
tire.Cr = 2.5
tire.Bf = 1.65
tire.Br = 1.65
tire.Ef = 1.0
tire.Er = 1.0
tire.Fz0 = 550

#=======================================================================================================================
# Actuator constant
#=======================================================================================================================
act= Const()
act.steerT= 0.2
act.driveT= 0.05
act.brakeT= 0.05