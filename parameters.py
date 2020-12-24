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
maximum.speed= 15/3.6
maximum.beta= np.pi/2
maximum.omega= np.pi
maximum.xi= np.pi

maximum.delta= np.deg2rad(15)     # 15*pi/180
maximum.F_drive= 328    # N;   90Nm/R tires
maximum.F_brake= 435    # N;   from brake system report

#maximum.power= 200 # not sure

#=======================================================================================================================
# Track
#=======================================================================================================================
trk= Const()
trk.width= 6      # m, 6 for london, 8 for rrc
trk.mu= 1.0      # 1.0-> ideal, 0.3-> snow, 0.5-> wet

trk.lap= 1
trk.reverse= False

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
scale.gamma_y= 250.0

#=======================================================================================================================
# Vehicle parameters
#=======================================================================================================================
veh= Const()
veh.lf = 1.516/ 2
veh.lr = 1.516/ 2
veh.L = 1.516
veh.m = 139+ 70                    # 139-> car net weight, 70-> estimated driver weight
veh.g = 9.81
veh.twf = 1.08
veh.twr = 0.8
veh.hcg = 0.167                     # Gravity center above ground
veh.wheelR= 0.55/2
veh.length= 3.4
veh.width= 1.3
veh.k_safe= 1.2                     # a safety factor for shortest distance computation

veh.Izz = 0.2* veh.m*(3.4**2+1.3**2)    # approximate by the inertia of oval

veh.k_roll = 0.5                    # suspension related roll balance relationship
veh.k_drive= 0
veh.k_brake= 0.5

#=======================================================================================================================
# Resistance
#=======================================================================================================================
resist= Const()

resist.rho= 1.15
resist.A= 1.7577
resist.Cd= 0.14          # drag coefficient, from DTU webpage
resist.Cr= 0.0025        # static rolling resistance coefficient, from wiki "rolling resistance"
resist.Cl= 0.25          # lift coefficient

#=======================================================================================================================
# Tire
#=======================================================================================================================
tire= Const()
tire.mu= trk.mu
a= 0.5

# Pacejka parameters
tire.eps= -0.1*a
tire.C= 2.5*a
tire.B = 10.0*a
tire.E = 0.5*a
tire.Fz0 = 515                    # mg/4

#=======================================================================================================================
# Actuator constant
#=======================================================================================================================
act= Const()
act.steerT= 3        # ideal-> 0.9
act.driveT= 1.25
act.brakeT= 0.05

#=======================================================================================================================
# Optimization parameters
#=======================================================================================================================
optimize= Const()
optimize.accu= 1e-07
optimize.iter= 8000
optimize.step= 1

regular= Const()
regular.Q_F= 0.0 #10
regular.Q_delta= 0.0 #0.01