import numpy as np

def Euler2Quaternion(euler):
    """
    Converts an euler angle attitude to a quaternian attitude
    :param euler: Euler angle attitude in a np.matrix(phi, theta, psi)
    :return: Quaternian attitude in np.matrix(e0, e1, e2, e3)
    """
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]

    e0 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)
    e1 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0) - np.sin(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0)
    e2 = np.cos(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0)
    e3 = np.sin(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) - np.cos(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)

    return e0, e1, e2, e3

######################################################################################
                #   Initial Conditions
######################################################################################
#   State
Pn0 = 0.0
Pe0 = 0.0
Pd0 = -200.0
u0 = 3.49997970e+01
v0 = -2.76273829e-05
w0 = 1.19203683e-01
phi0 = -1.52263833e-06
theta0 =  3.40582609e-03
psi0 = 0.0
p0 = 0
q0 = 0
r0 = 0
states0 = [Pn0, Pe0, Pd0, u0, v0, w0, phi0, theta0, psi0, p0, q0, r0]
#   Quaternion State
e0, e1, e2, e3 = Euler2Quaternion([phi0, theta0, psi0])
# e0 = 1.0
# e1 = 0.0
# e2 = 0.0
# e3 = 0.0
# e0 = 0.70710678
# e1 = 0.0
# e2 = -0.70710678
# e3 = 0.0
#   Forces
Fx = 0.0
Fy = 0.0
Fz = 0.0
Ml = 0.0
Mm = 0.0
Mn = 0.0

#   Control Surfaces
deltaA0 = 1.9965483711245567e-06
deltaE0 = -0.049348427825
deltaR0 = -2.423298335322978e-06
deltaT0 = 0.463818766653

######################################################################################
                #   Plotting Parameters
######################################################################################
ts_plotting = 05.135
unit_length = 0.25
fuse_h = unit_length
fuse_w = unit_length
fuse_l1 = unit_length * 2
fuse_l2 = unit_length
fuse_l3 = unit_length * 4
wing_l = unit_length
wing_w = unit_length * 6
tail_h = unit_length
tail_l = unit_length
tail_w = unit_length * 2

#   define the points on the plane following diagram on page 26, Fig2.14
points = np.array([[fuse_l1, 0, 0],
                   [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],
                   [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],
                   [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],
                   [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],
                   [-fuse_l3, 0, 0],
                   [0, wing_w / 2.0, 0],
                   [-wing_l, wing_w / 2.0, 0],
                   [-wing_l, -wing_w / 2.0, 0],
                   [0, -wing_w / 2.0, 0],
                   [-fuse_l3 + tail_l, tail_w / 2.0, 0],
                   [-fuse_l3, tail_w / 2.0, 0],
                   [-fuse_l3, -tail_w / 2.0, 0],
                   [-fuse_l3 + tail_l, -tail_w / 2.0, 0],
                   [-fuse_l3 + tail_l, 0, 0],
                   [-fuse_l3, 0, tail_h]]) * 50

#   define colors to use for the plane
#   Body Colors
bodyColors = np.empty((13, 3, 4), dtype=np.float32)
for i in range(8):
    # bodyColors[i] = np.array([255/256.0, 215/256.0, 0/256.0, 1])
    bodyColors[i] = np.array([np.random.rand(), np.random.rand(), np.random.rand(), 1])
#   wing/tail colors
for i in range(8,13):
    # bodyColors[i] = np.array([0/256.0, 256/256.0, 0/256.0, 1])
    bodyColors[i] = np.array([np.random.rand(), np.random.rand(), np.random.rand(), 1])

######################################################################################
                #   Plane Parameters
######################################################################################
mass = 13.5 #kg
jx = 0.8244 #kg m^2
jy = 1.135
jz = 1.759
jxz = 0.1204
# jxz = 0.0
S = 0.55
b = 2.8956
c = 0.18994
Sprop = 0.2027
rho = 1.2682
kMotor = 80
kTp = 0.0
kOmega = 0.0
e = 0.9
AR = (b**2) / S

g = 9.8     #gravity

######################################################################################
                #   Longitudinal Coefficients
######################################################################################
CL0 = 0.28
Cd0 = 0.03
Cm0 = -0.02338
ClAlpha = 3.45
CdAlpha = 0.3
CmAlpha = -0.38
Clq = 0.0
Cdq = 0.0
Cmq = -3.6
ClDeltae = -0.36
CdDeltae = 0.0
CmDeltae = -0.5
Cprop = 1.0
M = 50.0
alpha0 = 0.4712
epsilon = 0.1592
Cdp = 0.0437

######################################################################################
                #   Lateral Coefficients
######################################################################################
Cy0 = 0.0
Cl0 = 0.0
Cn0 = 0.0
CyBeta = -0.98
ClBeta = -0.12
CnBeta = 0.25
CyP = 0.0
ClP = -0.26
CnP = 0.022
CYr = 0.0
Clr = 0.14
Cnr = -0.35
CyDeltaa = 0.0
ClDeltaa = 0.08
CnDeltaa = 0.06
CyDeltar = -0.17
ClDeltar = 0.105
CnDeltar = -0.032
######################################################################################
                #   Calculation Variables
######################################################################################
ts_simulation = 0.020
num_steps = 10
#   gamma parameters pulled from page 36 (dynamics)
gamma = jx * jz - (jxz**2)
gamma1 = (jxz * (jx - jy + jz)) / gamma
gamma2 = (jz * (jz - jy) + (jxz**2)) / gamma
gamma3 = jz / gamma
gamma4 = jxz / gamma
gamma5 = (jz - jx) / jy
gamma6 = jxz / jy
gamma7 = ((jx - jy) * jx + (jxz**2)) / gamma
gamma8 = jx / gamma

#   C values defines on pag 62
Cp0         = gamma3 * Cl0      + gamma4 * Cn0
CpBeta      = gamma3 * ClBeta   + gamma4 * CnBeta
CpP         = gamma3 * ClP      + gamma4 * CnP
CpR         = gamma3 * Clr      + gamma4 * Cnr
CpDeltaA    = gamma3 * ClDeltaa + gamma4 * CnDeltaa
CpDeltaR    = gamma3 * ClDeltar + gamma4 * CnDeltar
Cr0         = gamma4 * Cl0      + gamma8 * Cn0
CrBeta      = gamma4 * ClBeta   + gamma8 * CnBeta
CrP         = gamma4 * ClP      + gamma8 * CnP
CrR         = gamma4 * Clr      + gamma8 * Cnr
CrDeltaA    = gamma4 * ClDeltaa + gamma8 * CnDeltaa
CrDeltaR    = gamma4 * ClDeltar + gamma8 * CnDeltar
# Ts = 1/33.333

######################################################################################
                #   Stochastic Wind Variables
######################################################################################
vWindInertial = np.matrix([[3.00001],
                           [-3.00001],
                           [-0.05001]])
#   Dryden gust model parameters (pg 56 UAV book)
Lu = 200.0
Lv = 200.0
Lw = 50.0
sigmaU = 1.06
sigmav = 1.06
sigmaW = 0.7

attackAng = 0.0
sideSlipAng = 0.0


######################################################################################
                #   Control Parameters
######################################################################################

# ts_control = 0.1
ts_control = ts_simulation * 2.0
sigma = 0.05 # low pass filter gain for derivative

# Initial path following values
Va0 = np.sqrt(u0 ** 2 + v0 ** 2 + w0 ** 2) #m/s
altitude0 = -Pd0
heading0 = psi0
beta0 = 0.0
path_angle0 = 0.0

########## Lateral transfer function vales
# Roll control values
e_phi_max = np.radians(40)
delta_a_max = np.radians(50)
a_phi1 = -0.5 * rho * Va0 ** 2 * S * b * CpP * b / (2 * Va0)
a_phi2 = 0.5 * rho * Va0 ** 2 * S * b * CpDeltaA
zeta_phi = 0.707
omega_n_phi = np.sqrt(np.abs(a_phi2) * (delta_a_max / e_phi_max))
kp_phi = delta_a_max / (e_phi_max) * 0.37
kd_phi = (2. * zeta_phi * omega_n_phi - a_phi1) / a_phi2 * 4.0

# Course control values
phi_max = np.radians(30)
course_bandwidth_separation = 36.0
omega_n_chi = omega_n_phi / course_bandwidth_separation
zeta_chi = 3.0
kp_chi = 2.0 * zeta_chi * omega_n_chi * Va0 / g * 5.55
kd_chi = 0.0
ki_chi = omega_n_chi**2 * Va0 / g * 0.01

# Sideslip control values
delta_r_max = np.radians(30)
e_beta_max = np.radians(60)
zeta_beta = 3.0
a_beta1 = -rho * Va0 * S * CyBeta / (2.0 * mass)
a_beta2 = rho * Va0 * S * CyDeltar / (2.0 * mass)
kp_beta = delta_r_max / e_beta_max / 5
ki_beta = (1 / a_beta2) * ((a_beta1 + a_beta2 * kp_beta) / (2.0 * zeta_beta))**2 * 5

########## Longitudinal transfer function values
h_error_max = 10.0
# energy balance control values
theta_max = np.radians(15)
kp_bal = 0.00011
ki_bal = 0.00003

# Pitch control values
delta_e_max = np.radians(30)
e_theta_max = np.radians(30)

a_theta1 = -rho * Va0**2 * c**2 * S * Cmq / (4.0 * jy * Va0)
a_theta2 = -rho * Va0**2 * c * S * CmAlpha / (2.0 * jy)
a_theta3 = rho * Va0**2 * c * S * CmDeltae / (2.0 * jy)

# kp_theta = delta_e_max / e_theta_max * 0.80
kp_theta = delta_e_max / e_theta_max * 0.6
omega_n_theta = np.sqrt(a_theta2 + (delta_e_max / e_theta_max) * abs(a_theta3))
zeta_theta = 5.0
kd_theta = (2 * zeta_theta * omega_n_theta - a_theta1) / a_theta3 / 19.0

# Throttle control values
delta_t_max = 1.0
kp_throttle = 0.0004
ki_throttle = 0.001

######################################################################################
                #   Sensor Parameters
######################################################################################
########### Accelerometer
accel_alpha = 0.4
# accel_alpha = 0.0
accel_gain = 1.0
accel_range = 6
accel_bandwidth = 100
accel_noise_density = 250e-6
accel_std_dev = accel_noise_density * np.sqrt(accel_bandwidth) #0.0025
# accel_std_dev = 0.000000000000000000001

########### Rate Gyro
rate_alpha = 0.95
# rate_alpha = 0.0
rate_gyro_range = 350
rate_gyro_bandwidth = 80
rate_gyro_noise_density = 0.015
# rate_gyro_std_dev = rate_gyro_noise_density * np.sqrt(rate_gyro_bandwidth) #0.134
rate_gyro_std_dev = 0.05

########### Pressure Sensor(Altitude)
ps_alpha = 0.3
# ps_alpha = 0.0
ps_range_min = 15
ps_range_max = 115
ps_beta_abs_pressure = 0.125
ps_std_dev = 0.01

########### Pressure Sensor (Airspeed)
aspeed_alpha = 0.3
# aspeed_alpha = 0.0
aspeed_range_min = 0
aspeed_range_max = 4
aspeed_beta_diff_pressure = 0.02
aspeed_std_dev = 0.002

########### Digital Compass
dc_beta_mag = np.radians(1.0)
dc_std_dev = np.radians(0.03)

########### GPS
ts_gps = 1.0
gps_bias_ne = 4.7
gps_bias_alt = 9.2
gps_std_dev_ne = 0.21
gps_std_dev_alt = 0.4
# gps_bias_ne = 0.0
# gps_bias_alt = 0.0
# gps_std_dev_ne = 0.000000001
# gps_std_dev_alt = 0.000000001
gps_k = 1./1100.
gps_sample_time = ts_gps
#
gps_std_dev_vel = 0.025
# gps_std_dev_vel = 0.0000000000025


######################################################################################
                #   Kinematic Guidance Parameters
######################################################################################
bx_dot = 0.17
bx = .017
bh_dot = 0.4
bh = 0.12
bva = 1.1

######################################################################################
                #   Path Follower Parameters
######################################################################################
chi_inf = np.radians(80)
k_path = 0.005
k_orbit = 2
t_rad = 275

######################################################################################
                #   RRT Parameters
######################################################################################
map_width = 8000
max_height = 700
num_blocks = 6
street_width = 0.5
step_size = t_rad*3.1