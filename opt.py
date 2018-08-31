from gpkit import Model, parse_variables
from gpkitmodels import g

class Aircraft(Model):
	"""An aircraft
	Variables
	---------
	m 		[-] 	mass
	"""
	def setup(self):
		exec parse_variables(Aircraft.__doc__)
		self.wing = Wing()
		self.components = [self.wing]
		return [self.m >= sum(c.topvar("m") for c in self.components)]

class AircraftP(Model):
	"""AircraftP
	Variables
	---------
	P 		[kW]	total power draw
	CD		[-]		total CD, referenced to wing area
	"""
	def setup(self,aircraft,state):
		self.wing_perf = aircraft.wing.dynamic(state)
		self.fs = state
		self.perf_models = [self.wing_perf]
		constraints = [0.5*aircraft.wing.S*self.wing_perf.C_L*fs.rho*fs.V**2 >= aircraft.m]
		return constraints, self.perf_models

class Wing(Model):
	"""A wing
	Variables
	---------
	S 		4	[m^2]		aero area
	rho		1	[kg/m^2]	
	m 			[kg]		mass

	"""
	def setup(self):
		exec parse_variables(Wing.__doc__)
		return [m >=  S*rho]
	def dynamic(self,state):
		return WingP(self,state)

class WingP(Model):
	#clark Y airfoil
	"""Wing performance model
	Variables
	---------
	C_L			[-]	operating C_L
	C_Lmax	1.4	[-]	maximum C_L
	"""	
	def setup(self,wing,state):
		exec parse_variables(WingP.__doc__)
		return [C_L<=C_Lmax]

class FlightState(Model):
    """ Flight State

    Variables
    ---------
    rho         1.225       [kg/m**3]       air density
    mu          1.789e-5    [N*s/m^2]       air viscosity
    V                       [kts]           speed
    qne                     [kg/s^2/m]      never exceed dynamic pressure
    Vne         160         [kts]           never exceed speed
    """
    def setup(self):
        exec parse_variables(FlightState.__doc__)
        return [qne == 0.5*rho*Vne**2]
        
aircraft = Aircraft()
aircraft.cost = aircraft.m
aircraft.debug()

sol = aircraft.solve(solver="mosek_cli") 	
print sol.table()