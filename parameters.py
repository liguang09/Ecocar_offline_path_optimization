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

maximum.delta= np.deg2rad(15)     # 15*pi/180
maximum.F_drive= 1091    # N;   150Nm/R* 2 tires
maximum.F_brake= 1000    # N;   from brake system report

#maximum.power= 200 # not sure

#=======================================================================================================================
# Track
#=======================================================================================================================
trk= Const()
trk.width= 6      # m
trk.mu= 1

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
scale.gamma_y= 500.0

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
veh.width= 1.3                   # multiply by a safety factor

veh.Izz = 0.2* veh.m*(3.4**2+1.3**2)

veh.k_roll = 0.5                    # suspension related roll balance relationship
veh.k_drive= 0
veh.k_brake= 0.5

#=======================================================================================================================
# Resistance
#=======================================================================================================================
resist= Const()

resist.rho= 1.15
resist.A= 1.7577
resist.Cd= 0.14         #0.4254       #0.0693         # drag coefficient, from DTU webpage
resist.Cr= 0.0025       #0.016          # static rolling resistance coefficient, from wiki
resist.Cl= 0.2474                       # lift coefficient

#=======================================================================================================================
# Tire
#=======================================================================================================================
tire= Const()
tire.mu= trk.mu
a= 0.0

# Pacejka parameters
tire.eps= -0.1*a        #-0.1
tire.C= 2.5*a           #2.5
tire.B = 10.0*a         #10
tire.E = 0.5*a          #0.5
tire.Fz0 = 515                      # mg/4

#=======================================================================================================================
# Actuator constant
#=======================================================================================================================
act= Const()
act.steerT= 0.2 #0.2
act.driveT= 0.05
act.brakeT= 0.05

#=======================================================================================================================
# Regularation parameters
#=======================================================================================================================
regular= Const()
regular.Q_F= 0.0
regular.Q_delta= 0.0