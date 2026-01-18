

from cmath import sin
from matplotlib.pylab import double
LINEAR_ACCELERATION_Z_MAX =  -3.0
LINEAR_ACCELERATION_Z_MIN = -23.0
MASS_MAX = 3.5
MASS_MIN = 0.0
ALTITUDE_MAX = 120.0
ALTITUDE_MIN = 0.0
H_REF = 258.0  
WIND_SPEED_MAX = 19.0
WIND_SPEED_MIN = 0.0
WIND_ANGLE_MAX = 360.0
WIND_ANGLE_MIN = 0.0
SPEED_MAX = 12.0
SPEED_MIN = 0.0
POWER_ACTUATOR_MAX = 1005.0
POWER_ACTUATOR_MIN = 0.0


def predict_actuator_power(x, d):
    g = d[0]*(LINEAR_ACCELERATION_Z_MAX-LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN  # Adjust g to the range of linear acceleration z
    m_d = d[1] * (MASS_MAX - MASS_MIN) + MASS_MIN  # Adjust m_d to the range of drone mass
    m_p = d[2] * (MASS_MAX - MASS_MIN) + MASS_MIN  # Adjust m_p to the range of payload mass
    h = d[3] * (ALTITUDE_MAX - ALTITUDE_MIN) + ALTITUDE_MIN  # Adjust h to the range of altitude
    h_ref = H_REF  # Adjust h_ref to the range of altitude
    v_wind = d[5] * (WIND_SPEED_MAX - WIND_SPEED_MIN) + WIND_SPEED_MIN  # Adjust v_wind to the range of wind speed
    theta_wind = d[6] * (WIND_ANGLE_MAX - WIND_ANGLE_MIN) + WIND_ANGLE_MIN  # Adjust theta_wind to the range of wind angle
    v_i = d[7] * (SPEED_MAX - SPEED_MIN) + SPEED_MIN  # Adjust v_i to the range of speed

    mass_sum = m_d + m_p
    sin_th = sin(theta_wind)

    A = g*(mass_sum)*(v_wind*sin_th+v_i)
    B = pow((v_wind*sin_th+v_i),3)
    C = (h/h_ref)*(mass_sum)

    nominator = x[3]*A + x[1]*B + x[2]*C
    denominator = x[0]

    return ((nominator / (denominator + 1e-8)) - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN)

actuator_input = [0.659753, 0.694286, 0, -0.0784969, 2.15, 0.0631579, -5.14323, 0.000813904]
coeff_actuator = [0.316826, 0.548014, 0.0973497, 0.0019184]
d = actuator_input
x = coeff_actuator

test_result = predict_actuator_power(x, d)
print(test_result)





wp_theta_wind = 150.0
pa_eta = x[0]
pa_delta = x[1]
pa_alpha = x[2]
pa_beta = x[3]

PAYLOAD_MAX = 3.5
PAYLOAD_MIN = 0.0
linear_acceleration_z_normalized = d[0]
linear_acceleration_z = linear_acceleration_z_normalized*(LINEAR_ACCELERATION_Z_MAX - LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN
payload_mass_normalized = 0.0
payload_mass = payload_mass_normalized*(PAYLOAD_MAX - PAYLOAD_MIN) + PAYLOAD_MIN
drone_mass = 2.43
total_mass = drone_mass + payload_mass


pa_phi = abs(sin(wp_theta_wind * 3.14159 / 180.0))
pa_theta = 1/(pa_eta + 1e-5)
pa_C_0 = -1*pa_theta * linear_acceleration_z * total_mass * pa_phi + pa_theta*pa_alpha*pow(pa_phi,3) + pa_delta
pa_C_1 = -1*(pa_theta * (linear_acceleration_z* total_mass) + 3*pa_theta*pa_alpha*pow(pa_phi,2))
pa_C_2 = -1*(3 * pa_theta * pa_alpha * pa_phi)
pa_C_3 = -1*(pa_theta * pa_alpha)
pa_C_4 = -1*(pa_theta * pa_beta * total_mass)
drone_v = 4
drone_v_2 = drone_v**2
drone_v_3 = drone_v**3
wp_v_wind = 8
drone_h = 6
H_REF_INV = 1.0 / H_REF

pa_exprs = pa_C_0 + pa_C_1 * (drone_v+wp_v_wind) + pa_C_2 * (drone_v_2+wp_v_wind) + pa_C_3 * (drone_v_3+wp_v_wind) + pa_C_4 * drone_h*H_REF_INV
print(pa_exprs)