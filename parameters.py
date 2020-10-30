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
maximum.beta= np.pi/2
maximum.omega= np.pi
maximum.xi= np.pi

maximum.delta= 15*np.pi/180
maximum.F_drive= 545 # not sure
maximum.F_brake= 450 # not sure

#maximum.power= 200 # not sure

#=======================================================================================================================
# Track
#=======================================================================================================================
trk = Const()
trk.width= 6.5
trk.lap= 1
trk.reverse= False
trk.mu= 0.4

#=======================================================================================================================
# Scaling factors
#=======================================================================================================================
scale = Const()
# states
scale.speed= maximum.speed
scale.beta= maximum.beta
scale.omega= maximum.omega
scale.n= trk.width/2
scale.xi= maximum.xi

# control inputs
scale.delta= maximum.delta
scale.F_drive= maximum.F_drive
scale.F_brake= maximum.F_brake
scale.gamma_y= 500.0

#=======================================================================================================================
# Vehicle parameters
#=======================================================================================================================
veh= Const()
veh.lf = 1.516/ 2
veh.lr = 1.516/ 2
veh.L = veh.lf + veh.lr
veh.m = 139 + 70                    # 139-> car net weight, 70-> estimated driver weight
veh.g = 9.81
veh.twf = 1.08
veh.twr = 0.8
veh.hcg = 0.167                     # Gravity center above ground
veh.width= 1.3*2.5                  # multiply by a safety factor
veh.length= 3.4
veh.wheelR= 0.55/2
veh.Izz = 0.5* veh.m* veh.wheelR**2


veh.Fd_coeff = 0.274                # drag coefficient
veh.Fl_coeff = 0.01                # lift coefficient
veh.Fr_coeff = 0.016                # static rolling resistance coefficient
veh.k_roll = 0.5                    # suspension related roll balance relationship

veh.k_drive= 0
veh.k_brake= 0.5
#=======================================================================================================================
# Tire
#=======================================================================================================================
tire= Const() # not sure
tire.mu= 0.0025
tire.eps= 0 #-0.1
tire.C= 0 #1.65
tire.B = 0 #0.0822
tire.E = 0 #1.0
tire.Fz0 = 515                      # mg/4

#=======================================================================================================================
# Actuator constant
#=======================================================================================================================
act= Const()
act.steerT= 0.2
act.driveT= 0.05
act.brakeT= 0.05