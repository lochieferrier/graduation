from gpkit import Model, parse_variables
from gpkitmodels import g
pi = 3.1412
class Aircraft(Model):
	"""An aircraft
	Variables
	---------
	m 			 	[kg] 	mass
	m_empty 		[kg]
	m_max_empty 255	[lb]	maximum empty weight from 103
	"""
	def setup(self):
		exec parse_variables(Aircraft.__doc__)
		self.wing = Wing()
		self.battery = Battery()
		self.motor = Motor()
		self.fuse = Fuselage()
		self.pilot = Pilot()
		self.components = [self.wing,self.motor,self.fuse,self.battery,self.pilot]
		self.empty_components = [self.wing,self.motor,self.fuse]
		constraints = [self.m >= sum(c.topvar("m") for c in self.components),
					   self.m_empty >= sum(c.topvar("m")  for c in self.empty_components),
					   self.m_empty <= m_max_empty,
					   self.fuse.m >= 0.17*self.m]
		return self.components, self.empty_components,constraints
	def dynamic(self,state):
		return AircraftP(self,state)

class AircraftP(Model):
	"""AircraftP
	Variables
	---------
	P 		[kW]	total power draw
	CD		[-]		total CD, referenced to wing area
	"""
	def setup(self,aircraft,state):
		self.wing_perf = aircraft.wing.dynamic(state)
		self.batt_perf = aircraft.battery.dynamic(state)
		self.motor_perf = aircraft.motor.dynamic(state)
		fs = state
		self.perf_models = [self.wing_perf,self.batt_perf,self.motor_perf]
		constraints = [0.5*aircraft.wing.S*self.wing_perf.C_L*fs.rho*fs.V**2 >= aircraft.m*g,
					   self.motor_perf.P >= 0.5*aircraft.wing.S*self.wing_perf.C_D*fs.rho*fs.V**3/0.8,
					   self.batt_perf.P >= self.motor_perf.P/0.8]
		return constraints, self.perf_models

class Fuselage(Model):
	"""Fuselage
	Variables
	---------
	m 		[kg]
	"""
	def setup(self):
		exec parse_variables(Fuselage.__doc__)

class Pilot(Model):
	"""Pilot
	Variables
	---------
	m 	85	[kg]	mass of pilot
	"""
	def setup(self):
		exec parse_variables(Pilot.__doc__)

class Motor(Model):
	""" Motor
	Variables
	---------
	m 			2.53	 	[kg]
	P_max_cont 	9.8			[kW]
	P_sp_cont	3.64		[kW/kg]
	RPM_max		7700		[rpm]
	"""
	def setup(self):
		exec parse_variables(Motor.__doc__)
		constraints = [#m >= P_max_cont/P_sp_cont
		]
		return constraints
	def dynamic(self,state):
		return MotorP(self,state)

class MotorP(Model):
	"""MotorP
	Variables
	---------
	P 		[kW]
	"""
	def setup(self,motor,state):
		exec parse_variables(MotorP.__doc__)
		return [P <= motor.P_max_cont]
class Wing(Model):
	"""A wing, untapered
	Variables
	---------
	S 			[ft^2]		aero area
	b  			[m]			span
	AR 			[-]			aspect ratio
	rho			[kg/m^2]	
	m 			[kg]		mass
	"""
	def setup(self):
		exec parse_variables(Wing.__doc__)
		self.spar = SolidSpar()
		self.skin = Skin()
		self.components = [self.spar,self.skin]
		return [self.m >= sum(c.topvar("m") for c in self.components),
				AR == b**2/S,
				AR <= 12,
				self.spar.l >= self.b,
				self.skin.S >= self.S*2.3
				], self.components
	def dynamic(self,state):
		return WingP(self,state)

class SolidSpar(Model):
	"""A solid spar, untapered
	Variables
	---------
	l 			[ft]		length  of spar
	w 		1	[in]		chordwise width of spar
	h  		6	[in]		height of spar
	rho 	380	[kg/m^3]	density of spar
	m 			[kg]		mass of spar
	"""
	def setup(self):
		exec parse_variables(SolidSpar.__doc__)
		return [m >= l*w*h*rho]

class Skin(Model):
	"""Skin
	Variables
	---------
	rho 	0.07 	[lb/ft^2] skin areal density assuming 4 oz cloth
	S 				[ft^2]	  skin area
	m 				[kg]	  skin mass	
	"""
	def setup(self):
		exec parse_variables(Skin.__doc__)
		return [m >= S*rho]
		
class WingP(Model):
	#clark Y airfoil
	"""Wing performance model
	Variables
	---------
	C_L			[-]	operating C_L
	C_Lmax	1.5	[-]	maximum C_L
	C_D			[-]	operating CD
	C_Di		[-] induced drag
	C_f			[-] friction drag
	C_Dp		[-]	profile drag
	mfac	1.2	[-]	profile drag margin
	Kf    1.180 [-]             form factor  
	Re 			[-]	reynolds number
	e 		0.8	[-]
	LD			[-]
	"""	
	def setup(self,wing,state):
		exec parse_variables(WingP.__doc__)
		return [C_L<=C_Lmax,
				Re == state["V"]*state["rho"]*(wing.S/wing["AR"])**0.5/state["mu"],
				C_D	>= C_Di + C_Dp,
				C_Di >= C_L**2/(pi*wing["AR"]*e),
				C_f**5 == (mfac*0.074)**5 /(Re),
				C_Dp == C_f*2.1*Kf,
				LD == C_L/C_D]

class Battery(Model):
    """ Battery
    Variables
    ---------
    m                   [kg]            battery total mass
    Estar       200     [Wh/kg]         battery cell specific energy
    E_capacity          [Wh]            battery total energy capacity
    P_max_cont  2160    [W/kg]          battery cell continuous specific power
    P_max_burst 5190    [W/kg]          battery cell burst specific power
    eta_pack    0.8     [-]             battery packing efficiency
    E_rho		352		[Wh/L]			battery energy density
    V_lim		5		[gallon]		Part 103 limit
    m_lim		30		[lb]			Part 103 limit
    """

    def setup(self):
        exec parse_variables(Battery.__doc__)
        constraints = [m >= E_capacity/(eta_pack*Estar),
        			    # m <= m_lim,
        			   E_capacity/E_rho <= V_lim
                       # (P_max_cont/Variable("a",1,"W/kg") - 513.49)*(1+(Estar/Variable("b",40.9911,"Wh/kg"))**(11.79229)) <=  6.17e9,
                       # (P_max_burst/Variable("a",1,"W/kg") - 944.0619)*(1+(Estar/Variable("b",38.21934,"Wh/kg"))**(11.15887)) <=  1.02e10
                    ]

        return constraints
    def dynamic(self,state):
        return BatteryP(self,state)

class BatteryP(Model):
    """BatteryP
    Variables
    ---------
    P                   [kW]        battery power draw
    """
    def setup(self,batt,state):
        exec parse_variables(BatteryP.__doc__)
        constraints = [P <= batt.m*batt.P_max_burst*batt.eta_pack]
        return constraints

class FlightState(Model):
    """ Flight State

    Variables
    ---------
    rho         1.225       [kg/m**3]       air density
    mu          1.789e-5    [N*s/m^2]       air viscosity
    V                       [kts]           speed
    qne                     [kg/s^2/m]      never exceed dynamic pressure
    Vne         100         [kts]           never exceed speed
    """
    def setup(self):
        exec parse_variables(FlightState.__doc__)
        return [qne == 0.5*rho*Vne**2]

class Cruise(Model):
    """

    Variables
    ---------
    R           [mi]       cruise range
    t           [min]       cruise time
    """

    def setup(self,aircraft,hybrid=False):
        exec parse_variables(Cruise.__doc__)
        self.flightstate = FlightState()
        self.perf = aircraft.dynamic(self.flightstate)
        constraints = [R <= t*self.flightstate.V]
        return constraints, self.flightstate, self.perf

class Mission(Model):
	""" Mission
	Variables
	---------
	Vstall			24	[kts]	power off level stall speed
	Vcruise_max		55	[kts]	maximum full power cruise speed
	R_req			100	[mi]	cruise range requirement
	"""
	def setup(self):
		exec parse_variables(Mission.__doc__)
		self.aircraft = Aircraft()
		self.cruise = Cruise(self.aircraft)
		self.fs = [self.cruise]
		constraints = [self.aircraft.m*g <= 0.5*self.cruise.flightstate.rho*self.aircraft.wing.S*1.5*self.Vstall**2,
		self.cruise.flightstate.V <= Vcruise_max,
		self.aircraft.battery.E_capacity*0.8 >= self.cruise.perf.motor_perf.P*self.cruise.t,
		self.cruise.R >= R_req]
		return constraints,self.aircraft, self.fs

M = Mission()
M.cost = M.aircraft.m_empty/M.cruise.R
# M.debug()
sol = M.solve(solver="mosek_cli")
print sol.table()
# print sol.summary()
print sol(M.cruise.R)
print sol(M.cruise.perf.wing_perf.LD)
print sol(M.aircraft.battery.m/M.aircraft.m)
print sol(M.aircraft.m)
print sol(M.aircraft.m_empty)
